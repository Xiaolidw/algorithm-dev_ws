// Copyright 2026 moon_warehouse team.
//
// Implementation of PredictionLayer. See the header for design notes.

#include "moon_warehouse_prediction_layer/prediction_layer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/time.hpp"

namespace moon_warehouse_prediction_layer
{

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

PredictionLayer::PredictionLayer() : CostmapLayer()
{
}

void PredictionLayer::onInitialize()
{
  CostmapLayer::onInitialize();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("model_states_topic", rclcpp::ParameterValue("/gazebo/model_states"));
  declareParameter("robot_model_name", rclcpp::ParameterValue(std::string("six_arm")));
  declareParameter("model_name_prefixes", rclcpp::ParameterValue(std::vector<std::string>{}));
  declareParameter("prediction_horizon_sec", rclcpp::ParameterValue(2.0));
  declareParameter("obstacle_radius_m", rclcpp::ParameterValue(0.3));
  declareParameter("min_velocity_mps", rclcpp::ParameterValue(0.05));
  declareParameter("predicted_cost", rclcpp::ParameterValue(254));
  declareParameter("update_interval_sec", rclcpp::ParameterValue(0.2));

  // Layer 不继承 rclcpp::Node：通过 node_.lock() 访问节点功能与参数。
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "PredictionLayer: lifecycle node is unavailable");
    return;
  }
  auto read_bool = [this, &node](const std::string & name) {
    return node->get_parameter(getFullName(name)).as_bool();
  };
  auto read_double = [this, &node](const std::string & name) {
    return node->get_parameter(getFullName(name)).as_double();
  };
  auto read_int = [this, &node](const std::string & name) {
    return node->get_parameter(getFullName(name)).as_int();
  };
  auto read_string = [this, &node](const std::string & name) {
    return node->get_parameter(getFullName(name)).as_string();
  };
  auto read_string_array = [this, &node](const std::string & name) {
    return node->get_parameter(getFullName(name)).as_string_array();
  };

  enabled_ = read_bool("enabled");
  prediction_horizon_sec_ = read_double("prediction_horizon_sec");
  obstacle_radius_m_ = read_double("obstacle_radius_m");
  min_velocity_mps_ = read_double("min_velocity_mps");
  predicted_cost_ = static_cast<unsigned char>(
    std::clamp(static_cast<int>(read_int("predicted_cost")), 0, 254));
  update_interval_sec_ = std::max(0.0, read_double("update_interval_sec"));
  robot_name_ = read_string("robot_model_name");
  const auto prefixes = read_string_array("model_name_prefixes");
  tracked_prefixes_.insert(prefixes.begin(), prefixes.end());

  if (tracked_prefixes_.empty() && !robot_name_.empty()) {
    // Default: track the cargo cubes unless the user configured prefixes.
    tracked_prefixes_.insert("red_cube");
    tracked_prefixes_.insert("blue_cube");
  }

  // Optional topic override for worlds that publish /<model>/current_pose
  // (SimpleMovePlugin) instead of /gazebo/model_states.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const std::string topic = read_string("model_states_topic");
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  model_states_sub_ = node->create_subscription<gazebo_msgs::msg::ModelStates>(
    topic, qos,
    std::bind(&PredictionLayer::model_states_callback, this, std::placeholders::_1),
    subscription_options);

  RCLCPP_INFO(
    logger_, "PredictionLayer ready: topic=%s, tracking prefixes, horizon=%.2fs, "
    "radius=%.2fm", topic.c_str(), prediction_horizon_sec_, obstacle_radius_m_);
  current_ = false;  // force first update
}

bool PredictionLayer::should_track(const std::string & name) const
{
  if (name == robot_name_ || name == robot_name_ + "_base") {
    return false;
  }
  if (tracked_prefixes_.empty()) {
    return true;
  }
  for (const auto & prefix : tracked_prefixes_) {
    if (name.rfind(prefix, 0) == 0) {
      return true;
    }
  }
  return false;
}

void PredictionLayer::model_states_callback(
  const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
  const double now = clock_->now().seconds();
  const auto & names = msg->name;
  for (size_t i = 0; i < names.size(); ++i) {
    if (!should_track(names[i])) {
      continue;
    }
    TrackedModel & model = models_[names[i]];
    model.has_state = true;
    model.x = msg->pose[i].position.x;
    model.y = msg->pose[i].position.y;
    model.vx = msg->twist[i].linear.x;
    model.vy = msg->twist[i].linear.y;
    model.stamp = now;
  }
}

std::vector<PredictionLayer::PredictedObstacle> PredictionLayer::compute_predictions(
  double now) const
{
  std::vector<PredictedObstacle> predictions;
  predictions.reserve(models_.size());

  for (const auto & entry : models_) {
    const TrackedModel & model = entry.second;
    if (!model.has_state) {
      continue;
    }
    const double speed = std::hypot(model.vx, model.vy);
    if (speed < min_velocity_mps_) {
      continue;  // stationary obstacles are already covered by the obstacle layer
    }
    const double dt = std::max(0.0, now - model.stamp);
    // Extrapolate from the measured stamp, not from "now": stale data should
    // not march forward indefinitely.
    const double horizon = std::min(prediction_horizon_sec_, dt + prediction_horizon_sec_);
    PredictedObstacle obstacle;
    obstacle.x = model.x + model.vx * horizon;
    obstacle.y = model.y + model.vy * horizon;
    obstacle.radius_m = obstacle_radius_m_;
    predictions.push_back(obstacle);
  }
  return predictions;
}

void PredictionLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  const double now = clock_->now().seconds();
  if (last_update_stamp_ > 0.0 && (now - last_update_stamp_) < update_interval_sec_) {
    return;  // throttle painting; bounds are already covered
  }
  last_update_stamp_ = now;

  const auto predictions = compute_predictions(now);
  for (const auto & obstacle : predictions) {
    *min_x = std::min(*min_x, obstacle.x - obstacle.radius_m);
    *max_x = std::max(*max_x, obstacle.x + obstacle.radius_m);
    *min_y = std::min(*min_y, obstacle.y - obstacle.radius_m);
    *max_y = std::max(*max_y, obstacle.y + obstacle.radius_m);
  }
  current_ = true;
}

void PredictionLayer::paint_obstacle(
  nav2_costmap_2d::Costmap2D & master_grid,
  const PredictedObstacle & obstacle,
  int min_i, int min_j, int max_i, int max_j) const
{
  const double res = master_grid.getResolution();
  const int r_cells = std::max(1, static_cast<int>(std::ceil(obstacle.radius_m / res)));

  const unsigned int mx = static_cast<unsigned int>((obstacle.x - master_grid.getOriginX()) / res);
  const unsigned int my = static_cast<unsigned int>((obstacle.y - master_grid.getOriginY()) / res);
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  const int i0 = std::max(min_i, static_cast<int>(mx) - r_cells);
  const int i1 = std::min(max_i, static_cast<int>(mx) + r_cells);
  const int j0 = std::max(min_j, static_cast<int>(my) - r_cells);
  const int j1 = std::min(max_j, static_cast<int>(my) + r_cells);

  for (int i = i0; i <= i1; ++i) {
    for (int j = j0; j <= j1; ++j) {
      if (i < 0 || j < 0 ||
        static_cast<unsigned int>(i) >= size_x ||
        static_cast<unsigned int>(j) >= size_y)
      {
        continue;
      }
      const double dx = (static_cast<double>(i) + 0.5) * res - obstacle.x;
      const double dy = (static_cast<double>(j) + 0.5) * res - obstacle.y;
      if (std::hypot(dx, dy) <= obstacle.radius_m) {
        master_grid.setCost(i, j, predicted_cost_);
      }
    }
  }
}

void PredictionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  const double now = clock_->now().seconds();
  const auto predictions = compute_predictions(now);
  for (const auto & obstacle : predictions) {
    paint_obstacle(master_grid, obstacle, min_i, min_j, max_i, max_j);
  }
}

void PredictionLayer::reset()
{
  models_.clear();
  last_update_stamp_ = -1.0;
  current_ = false;
}

}  // namespace moon_warehouse_prediction_layer

PLUGINLIB_EXPORT_CLASS(
  moon_warehouse_prediction_layer::PredictionLayer,
  nav2_costmap_2d::Layer)
