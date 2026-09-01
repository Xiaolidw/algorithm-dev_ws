#pragma once

#include <mutex>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

#include "mybot/msg/obstacle_trajectory_array.hpp"

namespace mppi_lstm_critic
{

class LSTMDynamicObstacleCritic : public mppi::critics::CriticFunction
{
public:
  LSTMDynamicObstacleCritic() = default;
  ~LSTMDynamicObstacleCritic() override = default;

  /// nav2_mppi_controller 要求的基本接口实现
  void initialize() override;
  void score(mppi::CriticData & data) override;

private:
  /// 返回当前 critic 使用的 logger
  rclcpp::Logger get_logger() const;

  // ========== 基本参数 ==========
  double weight_{0.0};           // 该 critic 在总 cost 中的权重
  int power_{1};                 // cost 的幂次（非线性拉伸）
  std::string prediction_topic_; // 动态障碍物预测结果话题名

  // ========== 距离相关参数 ==========
  //
  // safety_radius_          : 外圈，进入后才开始考虑该障碍物
  // danger_radius_          : 内圈，认为存在真实碰撞风险
  // min_effective_distance_ : 下限距离，避免数值上除 0
  // trajectory_point_step_  : 时间步采样步长（>=1）
  //
  double safety_radius_{0.7};
  double danger_radius_{0.5};
  double min_effective_distance_{0.05};
  int trajectory_point_step_{1};

  // ========== TTR / TTC 相关参数 ==========
  //
  // ttr_beta_ : 时间相关惩罚系数，越大越倾向于惩罚“较早”发生的风险
  double ttr_beta_{2.0};

  // ========== 激活条件（是否启用该 critic） ==========
  //
  // activate_radius_ : 机器人当前位置距离任一预测轨迹点小于该值时，
  //                    才认为机器人“踩在未来轨迹上”，激活动态障碍评估
  double activate_radius_{0.45};

  // ========== 轨迹通道与运动方向偏好 ==========
  //
  // path_radius_           : 将动态障碍轨迹视为一条“管道”的半径
  // lateral_sigma_         : 与轨迹中心线的横向距离权重衰减宽度
  // forward_penalty_scale_ : 与障碍物同向时的额外惩罚系数
  // backward_discount_     : 与障碍物反向或避让时的折扣系数
  double path_radius_{0.45};
  double lateral_sigma_{0.3};
  double forward_penalty_scale_{2.0};
  double backward_discount_{0.5};

  // ========== 订阅数据缓存 ==========
  rclcpp::Subscription<mybot::msg::ObstacleTrajectoryArray>::SharedPtr sub_;
  mybot::msg::ObstacleTrajectoryArray::SharedPtr latest_msg_;
  std::mutex mutex_;

  // ========== 日志 ==========
  rclcpp::Logger logger_{rclcpp::get_logger("LSTMDynamicObstacleCritic")};
};

}  // namespace mppi_lstm_critic

