#include "mppi_lstm_critic/lstm_dynamic_obstacle_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace mppi_lstm_critic
{

void LSTMDynamicObstacleCritic::initialize()
{
  auto node = parent_.lock();
  if (!node || !parameters_handler_) {
    throw std::runtime_error("LSTMDynamicObstacleCritic has no parent or parameter handler");
  }
  auto get_param = parameters_handler_->getParamGetter(name_);
  get_param(power_, "cost_power", power_);
  get_param(weight_, "cost_weight", weight_);
  get_param(safety_radius_, "safety_radius", safety_radius_);
  get_param(danger_radius_, "danger_radius", danger_radius_);
  get_param(collision_cost_, "collision_cost", collision_cost_);
  get_param(data_timeout_, "data_timeout", data_timeout_);
  get_param(exit_time_weight_, "exit_time_weight", exit_time_weight_);
  get_param(risk_tube_radius_, "risk_tube_radius", risk_tube_radius_);
  get_param(
    cruise_soft_cost_scale_, "cruise_soft_cost_scale", cruise_soft_cost_scale_);
  get_param(pass_soft_cost_scale_, "pass_soft_cost_scale", pass_soft_cost_scale_);
  get_param(trajectory_point_step_, "trajectory_point_step", trajectory_point_step_);
  get_param(feedback_publish_period_, "feedback_publish_period", feedback_publish_period_);
  get_param(blocked_safe_ratio_, "blocked_safe_ratio", blocked_safe_ratio_);
  get_param(infeasible_safe_ratio_, "infeasible_safe_ratio", infeasible_safe_ratio_);
  get_param(emergency_distance_, "emergency_distance", emergency_distance_);
  get_param(emergency_ttc_, "emergency_ttc", emergency_ttc_);
  get_param(prediction_topic_, "obstacle_prediction_topic", prediction_topic_);
  trajectory_point_step_ = std::max(1, trajectory_point_step_);
  danger_radius_ = std::clamp(danger_radius_, 0.05, safety_radius_);
  risk_tube_radius_ = std::max(danger_radius_, risk_tube_radius_);
  cruise_soft_cost_scale_ = std::clamp(cruise_soft_cost_scale_, 0.0, 1.0);
  pass_soft_cost_scale_ = std::clamp(pass_soft_cost_scale_, 0.0, 1.0);
  feedback_publish_period_ = std::max(0.05, feedback_publish_period_);
  infeasible_safe_ratio_ = std::clamp(infeasible_safe_ratio_, 0.0, 1.0);
  blocked_safe_ratio_ = std::clamp(
    blocked_safe_ratio_, infeasible_safe_ratio_, 1.0);

  subscription_ = node->create_subscription<
    moon_warehouse_interfaces::msg::ObstacleTrajectoryArray>(
    prediction_topic_, rclcpp::SystemDefaultsQoS(),
    [this](moon_warehouse_interfaces::msg::ObstacleTrajectoryArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ = std::move(msg);
    });
  state_subscription_ = node->create_subscription<std_msgs::msg::String>(
    "/sipp/state", rclcpp::SystemDefaultsQoS(),
    [this](std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_sipp_state_ = msg->data;
    });
  feedback_publisher_ = node->create_publisher<std_msgs::msg::String>(
    "/mppi/sipp_feedback", rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO(
    node->get_logger(),
    "LSTMDynamicObstacleCritic: topic=%s safety=%.2f danger=%.2f "
    "risk_tube=%.2f exit_weight=%.2f weight=%.1f",
    prediction_topic_.c_str(), safety_radius_, danger_radius_,
    risk_tube_radius_, exit_time_weight_, weight_);
}

void LSTMDynamicObstacleCritic::publishFeedback(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & status,
  const std::string & risk_level, double safe_ratio, double risk_ratio,
  double min_clearance, double earliest_collision_ttc,
  bool override_recommended, const std::string & reason)
{
  if (!feedback_publisher_) {
    return;
  }
  const double now_seconds = node->now().seconds();
  if (
    last_feedback_publish_seconds_ >= 0.0 &&
    now_seconds - last_feedback_publish_seconds_ < feedback_publish_period_)
  {
    return;
  }
  last_feedback_publish_seconds_ = now_seconds;
  if (!std::isfinite(min_clearance)) {
    min_clearance = -1.0;
  }
  if (!std::isfinite(earliest_collision_ttc)) {
    earliest_collision_ttc = -1.0;
  }
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(4);
  stream << "{\"status\":\"" << status
         << "\",\"risk_level\":\"" << risk_level
         << "\",\"safe_trajectory_ratio\":" << safe_ratio
         << ",\"risk_trajectory_ratio\":" << risk_ratio
         << ",\"collision_probability\":" << (1.0 - safe_ratio)
         << ",\"min_clearance\":" << min_clearance
         << ",\"earliest_collision_ttc\":" << earliest_collision_ttc
         << ",\"override_recommended\":"
         << (override_recommended ? "true" : "false")
         << ",\"sipp_state\":\"" << latest_sipp_state_
         << "\",\"reason\":\"" << reason << "\"}";
  std_msgs::msg::String message;
  message.data = stream.str();
  feedback_publisher_->publish(message);
}

void LSTMDynamicObstacleCritic::score(mppi::CriticData & data)
{
  if (!enabled_) {
    return;
  }
  auto node = parent_.lock();
  if (!node) {
    return;
  }
  moon_warehouse_interfaces::msg::ObstacleTrajectoryArray::SharedPtr prediction;
  std::string sipp_state;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prediction = latest_;
    sipp_state = latest_sipp_state_;
  }
  if (!prediction || prediction->trajectories.empty()) {
    publishFeedback(
      node, "CLEAR", "NORMAL", 1.0, 0.0,
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(), false, "no_dynamic_prediction");
    return;
  }
  const rclcpp::Time stamp(prediction->header.stamp);
  if (stamp.nanoseconds() > 0 && (node->now() - stamp).seconds() > data_timeout_) {
    publishFeedback(
      node, "STALE", "CAUTION", 1.0, 0.0,
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(), false, "prediction_timeout");
    return;
  }

  const auto & tx = data.trajectories.x;
  const auto & ty = data.trajectories.y;
  if (tx.dimension() != 2 || tx.shape() != ty.shape()) {
    return;
  }
  const size_t trajectory_count = std::min(tx.shape()[0], data.costs.size());
  const size_t time_steps = tx.shape()[1];
  const double soft_span = std::max(0.05, safety_radius_ - danger_radius_);
  const double safety_squared = safety_radius_ * safety_radius_;
  const double danger_squared = danger_radius_ * danger_radius_;
  const double risk_tube_squared = risk_tube_radius_ * risk_tube_radius_;
  double soft_cost_scale = 1.0;
  if (sipp_state == "PASS_COMMITTED" || sipp_state == "PREPARE_TO_PASS") {
    soft_cost_scale = pass_soft_cost_scale_;
  } else if (
    sipp_state == "CRUISE" || sipp_state == "FAST_CRUISE" ||
    sipp_state == "NORMAL_CRUISE" || sipp_state == "ENDPOINT_TURN" ||
    sipp_state == "DEGRADED" || sipp_state == "TOO_LATE_BYPASS")
  {
    soft_cost_scale = cruise_soft_cost_scale_;
  }
  const double horizon = std::max(
    static_cast<double>(data.model_dt),
    static_cast<double>(time_steps) * static_cast<double>(data.model_dt));

  // Obstacle interpolation depends only on time, not on the sampled robot
  // trajectory. Precompute it once instead of repeating it for all 2000 MPPI
  // samples; this keeps the 20 Hz controller loop practical in the VM.
  std::vector<std::vector<std::pair<double, double>>> obstacle_points(time_steps);
  std::vector<double> future_times(time_steps, 0.0);
  for (size_t step = 0; step < time_steps;
    step += static_cast<size_t>(trajectory_point_step_))
  {
    const double future_time = static_cast<double>(step) * data.model_dt;
    future_times[step] = future_time;
    for (const auto & obstacle : prediction->trajectories) {
      const size_t count = std::min(
        obstacle.future_poses.size(), obstacle.future_times.size());
      if (count == 0) {
        continue;
      }
      const auto end = obstacle.future_times.begin() + static_cast<std::ptrdiff_t>(count);
      const auto found = std::lower_bound(
        obstacle.future_times.begin(), end, static_cast<float>(future_time));
      if (found == end) {
        // The predictor has no information beyond its published horizon.
        // Holding the final pose forever creates a phantom static obstacle at
        // the end of every LSTM trajectory and can deadlock a committed pass.
        continue;
      }
      if (found == obstacle.future_times.begin()) {
        const auto & pose = obstacle.future_poses[0];
        obstacle_points[step].emplace_back(pose.position.x, pose.position.y);
        continue;
      }
      const size_t upper = static_cast<size_t>(
        std::distance(obstacle.future_times.begin(), found));
      const size_t lower = upper - 1;
      const double lower_time = obstacle.future_times[lower];
      const double upper_time = obstacle.future_times[upper];
      const double span = std::max(1e-6, upper_time - lower_time);
      const double ratio = std::clamp(
        (future_time - lower_time) / span, 0.0, 1.0);
      const auto & first = obstacle.future_poses[lower];
      const auto & second = obstacle.future_poses[upper];
      obstacle_points[step].emplace_back(
        first.position.x + ratio * (second.position.x - first.position.x),
        first.position.y + ratio * (second.position.y - first.position.y));
    }
  }

  size_t collision_count = 0;
  size_t risk_count = 0;
  double global_min_clearance = std::numeric_limits<double>::infinity();
  double earliest_collision_ttc = std::numeric_limits<double>::infinity();
  for (size_t sample = 0; sample < trajectory_count; ++sample) {
    double accumulated = 0.0;
    bool collision = false;
    bool risk_seen = false;
    double last_risk_time = 0.0;
    double initial_clearance = std::numeric_limits<double>::infinity();
    double previous_clearance = std::numeric_limits<double>::infinity();
    bool starts_in_danger = false;
    bool escaped_initial_danger = false;
    for (size_t step = 0; step < time_steps;
      step += static_cast<size_t>(trajectory_point_step_))
    {
      const double future_time = future_times[step];
      double nearest_squared = std::numeric_limits<double>::infinity();
      for (const auto & point : obstacle_points[step]) {
        const double dx = static_cast<double>(tx(sample, step)) - point.first;
        const double dy = static_cast<double>(ty(sample, step)) - point.second;
        nearest_squared = std::min(nearest_squared, dx * dx + dy * dy);
      }
      double nearest = std::numeric_limits<double>::infinity();
      if (std::isfinite(nearest_squared)) {
        nearest = std::sqrt(std::max(0.0, nearest_squared));
        global_min_clearance = std::min(
          global_min_clearance, nearest);
      }
      if (future_time <= 1e-9 && std::isfinite(nearest)) {
        initial_clearance = nearest;
        previous_clearance = nearest;
        starts_in_danger = nearest_squared <= danger_squared;
      }
      if (nearest_squared <= danger_squared) {
        bool hard_collision = !starts_in_danger || escaped_initial_danger;
        if (starts_in_danger && future_time > 1e-9) {
          // If prediction already encloses the robot at t=0, rejecting every
          // sample creates an inescapable deadlock.  Permit only trajectories
          // that increase clearance promptly and monotonically; stationary,
          // risk-deepening, or re-entering samples remain hard collisions.
          const bool regressing =
            std::isfinite(previous_clearance) &&
            nearest + 0.015 < previous_clearance;
          const bool insufficient_escape =
            future_time >= 0.40 &&
            nearest < initial_clearance + 0.06;
          hard_collision = regressing || insufficient_escape;
        }
        if (hard_collision) {
          earliest_collision_ttc = std::min(earliest_collision_ttc, future_time);
          collision = true;
        }
      } else if (nearest_squared < safety_squared) {
        const double normalized = (safety_radius_ - nearest) / soft_span;
        accumulated += normalized * normalized * std::exp(-0.5 * future_time);
      }
      if (starts_in_danger && nearest_squared > danger_squared) {
        escaped_initial_danger = true;
      }
      if (std::isfinite(nearest)) {
        previous_clearance = nearest;
      }
      if (nearest_squared <= risk_tube_squared) {
        risk_seen = true;
        last_risk_time = future_time;
      }
    }
    double trajectory_cost = soft_cost_scale * accumulated;
    if (!collision && risk_seen) {
      const double normalized_exit = std::clamp(last_risk_time / horizon, 0.0, 1.0);
      trajectory_cost += exit_time_weight_ * normalized_exit * normalized_exit;
    }
    if (trajectory_cost > 0.0) {
      data.costs(sample) += static_cast<float>(
        weight_ * std::pow(trajectory_cost, static_cast<double>(power_)));
    }
    if (collision) {
      ++collision_count;
      data.costs(sample) += static_cast<float>(collision_cost_);
    }
    if (risk_seen) {
      ++risk_count;
    }
  }
  const double denominator = std::max<size_t>(1, trajectory_count);
  const double safe_ratio = 1.0 - static_cast<double>(collision_count) / denominator;
  const double risk_ratio = static_cast<double>(risk_count) / denominator;
  std::string status = "ACCEPTED";
  std::string risk_level = "NORMAL";
  std::string reason = "safe_candidates_available";
  if (safe_ratio <= infeasible_safe_ratio_) {
    status = "INFEASIBLE";
    risk_level = "CAUTION";
    reason = "safe_trajectory_ratio_exhausted";
  } else if (safe_ratio <= blocked_safe_ratio_) {
    status = "TEMPORARILY_BLOCKED";
    risk_level = "CAUTION";
    reason = "few_safe_trajectories";
  }
  const bool emergency =
    safe_ratio <= infeasible_safe_ratio_ &&
    global_min_clearance <= emergency_distance_ &&
    earliest_collision_ttc >= 0.0 && earliest_collision_ttc <= emergency_ttc_;
  if (emergency) {
    status = "SAFETY_OVERRIDE";
    risk_level = "EMERGENCY";
    reason = "imminent_collision_high_confidence";
  }
  publishFeedback(
    node, status, risk_level, safe_ratio, risk_ratio,
    global_min_clearance, earliest_collision_ttc, emergency, reason);
}

}  // namespace mppi_lstm_critic

PLUGINLIB_EXPORT_CLASS(
  mppi_lstm_critic::LSTMDynamicObstacleCritic,
  mppi::critics::CriticFunction)
