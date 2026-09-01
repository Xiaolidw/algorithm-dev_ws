#ifndef MPPI_LSTM_CRITIC__LSTM_DYNAMIC_OBSTACLE_CRITIC_HPP_
#define MPPI_LSTM_CRITIC__LSTM_DYNAMIC_OBSTACLE_CRITIC_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_mppi_controller/critic_function.hpp"
#include "moon_warehouse_interfaces/msg/obstacle_trajectory_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace mppi_lstm_critic
{

class LSTMDynamicObstacleCritic : public mppi::critics::CriticFunction
{
public:
  void initialize() override;
  void score(mppi::CriticData & data) override;

private:
  void publishFeedback(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & status,
    const std::string & risk_level, double safe_ratio, double risk_ratio,
    double min_clearance, double earliest_collision_ttc,
    bool override_recommended, const std::string & reason);

  double weight_{25.0};
  int power_{1};
  double safety_radius_{0.85};
  double danger_radius_{0.58};
  double collision_cost_{100000.0};
  double data_timeout_{0.5};
  double exit_time_weight_{2.0};
  double exit_time_beta_{2.0};
  double closing_speed_weight_{1.25};
  double receding_discount_{0.45};
  double relative_speed_scale_{0.60};
  double risk_tube_radius_{0.72};
  double cruise_soft_cost_scale_{0.25};
  double pass_soft_cost_scale_{0.0};
  int trajectory_point_step_{2};
  double feedback_publish_period_{0.10};
  double blocked_safe_ratio_{0.20};
  double infeasible_safe_ratio_{0.05};
  double emergency_distance_{0.45};
  double emergency_ttc_{0.45};
  std::string prediction_topic_{"/moon_warehouse/dynamic_obstacle_trajectories"};

  rclcpp::Subscription<
    moon_warehouse_interfaces::msg::ObstacleTrajectoryArray>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
  moon_warehouse_interfaces::msg::ObstacleTrajectoryArray::SharedPtr latest_;
  std::string latest_sipp_state_{"DEGRADED"};
  double last_feedback_publish_seconds_{-1.0};
  std::mutex mutex_;
};

}  // namespace mppi_lstm_critic

#endif  // MPPI_LSTM_CRITIC__LSTM_DYNAMIC_OBSTACLE_CRITIC_HPP_
