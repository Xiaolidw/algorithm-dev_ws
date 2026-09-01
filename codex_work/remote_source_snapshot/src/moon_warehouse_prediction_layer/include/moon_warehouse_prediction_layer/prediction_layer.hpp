// Copyright 2026 moon_warehouse team.
//
// PredictionLayer: Nav2 costmap layer that feeds *predicted* obstacle
// positions to the planner.
//
// Background:
//   A*/Hybrid-A* plan on a static snapshot and MPPI optimises over the
//   current costmap. Neither one knows where a moving obstacle will be in
//   the next second. For the competition world, obstacles (SimpleMovePlugin)
//   move back and forth on known segments, so a short linear extrapolation
//   of their Gazebo velocity covers most of the predictive gain.
//
//   This layer subscribes to /gazebo/model_states, caches the current
//   pose+twist of every configured obstacle model, extrapolates each one by
//   prediction_horizon_sec along its linear velocity, and stamps the
//   predicted footprint into the master costmap. MPPI's optimisation window
//   then treats the oncoming obstacle as an immediate threat and steers
//   around it before it physically arrives.

#ifndef MOON_WAREHOUSE_PREDICTION_LAYER__PREDICTION_LAYER_HPP_
#define MOON_WAREHOUSE_PREDICTION_LAYER__PREDICTION_LAYER_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "gazebo_msgs/msg/model_states.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace moon_warehouse_prediction_layer
{

class PredictionLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  PredictionLayer();

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

  /// Predicted costs are temporary marks; never clear them via map-clearing ops.
  bool isClearable() override { return false; }

protected:
  /// One tracked obstacle with its last measured state.
  struct TrackedModel
  {
    bool has_state{false};
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
    double stamp{0.0};  // seconds, rcl time base
  };

  /// Projected footprint of one obstacle at its extrapolated position.
  struct PredictedObstacle
  {
    double x{0.0};
    double y{0.0};
    double radius_m{0.0};
  };

  void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

  /// Return true when the model name should be tracked (robot excluded).
  bool should_track(const std::string & name) const;

  /// Extrapolate cached states by prediction_horizon_sec.
  std::vector<PredictedObstacle> compute_predictions(double now) const;

  /// Draw one predicted obstacle onto the master costmap as a filled circle.
  void paint_obstacle(
    nav2_costmap_2d::Costmap2D & master_grid,
    const PredictedObstacle & obstacle,
    int min_i, int min_j, int max_i, int max_j) const;

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  std::map<std::string, TrackedModel> models_;
  std::set<std::string> tracked_prefixes_;
  std::string robot_name_;
  bool enabled_{true};
  double prediction_horizon_sec_{2.0};
  double obstacle_radius_m_{0.3};
  double min_velocity_mps_{0.05};
  unsigned char predicted_cost_{254};
  double last_update_stamp_{-1.0};
  double update_interval_sec_{0.2};  // throttle re-painting
  rclcpp::Logger logger_{rclcpp::get_logger("prediction_layer")};
};

}  // namespace moon_warehouse_prediction_layer

#endif  // MOON_WAREHOUSE_PREDICTION_LAYER__PREDICTION_LAYER_HPP_
