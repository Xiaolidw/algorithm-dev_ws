#ifndef MPPI_LSTM_CRITIC__LSTM_DYNAMIC_OBSTACLE_CRITIC_HPP_
#define MPPI_LSTM_CRITIC__LSTM_DYNAMIC_OBSTACLE_CRITIC_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_mppi_controller/critic_function.hpp"
#include "moon_warehouse_interfaces/msg/obstacle_trajectory_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mppi_lstm_critic
{

class LSTMDynamicObstacleCritic : public mppi::critics::CriticFunction
{
public:
  void initialize() override;
  void score(mppi::CriticData & data) override;

private:
  double weight_{25.0};
  int power_{1};
  double safety_radius_{0.85};
  double danger_radius_{0.58};
  double collision_cost_{100000.0};
  double data_timeout_{0.5};
  int trajectory_point_step_{2};
  std::string prediction_topic_{"/moon_warehouse/dynamic_obstacle_trajectories"};

  rclcpp::Subscription<
    moon_warehouse_interfaces::msg::ObstacleTrajectoryArray>::SharedPtr subscription_;
  moon_warehouse_interfaces::msg::ObstacleTrajectoryArray::SharedPtr latest_;
  std::mutex mutex_;
};

}  // namespace mppi_lstm_critic

#endif  // MPPI_LSTM_CRITIC__LSTM_DYNAMIC_OBSTACLE_CRITIC_HPP_
