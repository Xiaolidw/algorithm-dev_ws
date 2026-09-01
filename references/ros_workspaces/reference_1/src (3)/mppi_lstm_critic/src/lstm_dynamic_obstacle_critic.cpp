#include "mppi_lstm_critic/lstm_dynamic_obstacle_critic.hpp"

#include <stdexcept>
#include <cmath>
#include <limits>
#include <algorithm>

#include <pluginlib/class_list_macros.hpp>
#include <xtensor/xtensor.hpp>

namespace mppi_lstm_critic
{

rclcpp::Logger LSTMDynamicObstacleCritic::get_logger() const
{
  return logger_;
}

void LSTMDynamicObstacleCritic::initialize()
{
  auto node = parent_.lock();

  if (!node) {
    RCLCPP_FATAL(
      get_logger(),
      "[LSTMCritic FATAL] parent_ node is null or failed to lock.");
    throw std::runtime_error(
      "[LSTM Critic] parent_ node is null. MPPI didn't inject parent.");
  }

  if (!parameters_handler_) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[LSTMCritic ERROR] parameters_handler_ is null.");
    throw std::runtime_error(
      "[LSTM Critic] parameters_handler_ is null after base init.");
  }

  auto getParam = parameters_handler_->getParamGetter(name_);

  // ===== 1. 参数读取 =====
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 25.0);
  getParam(
    prediction_topic_, "obstacle_prediction_topic",
    std::string("/obstacle_trajectory"));

  // 距离相关参数
  getParam(safety_radius_, "safety_radius", safety_radius_);
  getParam(safety_radius_, "safety_distance", safety_radius_);  // 兼容旧字段名
  getParam(danger_radius_, "danger_radius", danger_radius_);
  getParam(min_effective_distance_, "min_effective_distance", min_effective_distance_);
  getParam(trajectory_point_step_, "trajectory_point_step", trajectory_point_step_);

  // 时间相关项（惩罚“在危险区域停留太久或进入过早”）
  getParam(ttr_beta_, "ttr_beta", ttr_beta_);

  // 轨迹管道与方向偏好
  getParam(path_radius_, "path_radius", path_radius_);
  getParam(lateral_sigma_, "lateral_sigma", lateral_sigma_);
  getParam(forward_penalty_scale_, "forward_penalty_scale", forward_penalty_scale_);
  getParam(backward_discount_, "backward_discount", backward_discount_);

  // ===== 2. 参数检查与修正 =====
  if (trajectory_point_step_ <= 0) {
    RCLCPP_WARN(
      node->get_logger(),
      "[LSTMCritic] trajectory_point_step(%d) <= 0, reset to 1.",
      trajectory_point_step_);
    trajectory_point_step_ = 1;
  }

  if (safety_radius_ <= 0.0) {
    safety_radius_ = 0.7;
  }

  if (danger_radius_ <= 0.0 || danger_radius_ > safety_radius_) {
    RCLCPP_WARN(
      node->get_logger(),
      "[LSTMCritic] danger_radius(%.3f) invalid, reset to 0.8 * safety_radius(%.3f).",
      danger_radius_, safety_radius_);
    danger_radius_ = 0.8 * safety_radius_;
  }

  if (min_effective_distance_ <= 0.0) {
    min_effective_distance_ = 0.01;
  }

  if (path_radius_ <= 0.0) {
    // 轨道通道半径默认略大于 danger_radius
    path_radius_ = danger_radius_ + 0.1;
    RCLCPP_WARN(
      node->get_logger(),
      "[LSTMCritic] path_radius <= 0, reset to danger_radius + 0.1 = %.3f",
      path_radius_);
  }

  if (path_radius_ < danger_radius_) {
    path_radius_ = danger_radius_ + 0.05;
    RCLCPP_WARN(
      node->get_logger(),
      "[LSTMCritic] path_radius < danger_radius, reset to %.3f",
      path_radius_);
  }

  if (lateral_sigma_ <= 0.0) {
    lateral_sigma_ = 0.3;
    RCLCPP_WARN(
      node->get_logger(),
      "[LSTMCritic] lateral_sigma <= 0, reset to 0.3");
  }

  if (ttr_beta_ <= 0.0) {
    ttr_beta_ = 2.0;
    RCLCPP_WARN(
      node->get_logger(),
      "[LSTMCritic] ttr_beta <= 0, reset to 2.0");
  }

  if (forward_penalty_scale_ < 0.0) {
    forward_penalty_scale_ = 0.0;
    RCLCPP_WARN(
      node->get_logger(),
      "[LSTMCritic] forward_penalty_scale < 0, clamp to 0");
  }

  if (backward_discount_ <= 0.0 || backward_discount_ > 1.0) {
    backward_discount_ = std::min(1.0, std::max(0.1, backward_discount_));
    RCLCPP_WARN(
      node->get_logger(),
      "[LSTMCritic] backward_discount out of (0,1], clamp to %.3f",
      backward_discount_);
  }

  RCLCPP_INFO(
    node->get_logger(),
    "[LSTMCritic] params: topic=%s, weight=%.3f, power=%d, "
    "safety_radius=%.3f, danger_radius=%.3f, min_effective_distance=%.3f, "
    "trajectory_point_step=%d, ttr_beta=%.3f, "
    "path_radius=%.3f, lateral_sigma=%.3f, "
    "forward_penalty_scale=%.3f, backward_discount=%.3f",
    prediction_topic_.c_str(), weight_, power_,
    safety_radius_, danger_radius_, min_effective_distance_,
    trajectory_point_step_, ttr_beta_,
    path_radius_, lateral_sigma_,
    forward_penalty_scale_, backward_discount_);

  // ===== 3. 订阅 LSTM 预测的障碍物轨迹 =====
  sub_ = node->create_subscription<mybot::msg::ObstacleTrajectoryArray>(
    prediction_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this, node](mybot::msg::ObstacleTrajectoryArray::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_msg_ = msg;
      RCLCPP_DEBUG(
        node->get_logger(),
        "[LSTMCritic] got obstacle_trajectory, trajectories = %zu",
        latest_msg_ ? latest_msg_->trajectories.size() : 0);
    });

  RCLCPP_INFO(
    node->get_logger(),
    "[LSTMCritic] initialize() done, subscribed to: %s",
    prediction_topic_.c_str());
}

// ==================== score ====================

void LSTMDynamicObstacleCritic::score(mppi::CriticData & data)
{
  if (!enabled_) {
    return;
  }

  auto node = parent_.lock();
  if (!node) {
    RCLCPP_ERROR(
      get_logger(),
      "[LSTMCritic] Parent node expired in score(), skip.");
    return;
  }

  auto & trajectories = data.trajectories;
  auto & traj_x = trajectories.x;       // [num_traj, time_steps]
  auto & traj_y = trajectories.y;
  auto & costs = data.costs;            // [num_traj]
  const float model_dt = data.model_dt; // MPPI 模型内部时间步长

  if (traj_x.size() == 0 || traj_y.size() == 0 || costs.size() == 0) {
    RCLCPP_DEBUG(
      node->get_logger(),
      "[LSTMCritic] Trajectories or costs empty, skip.");
    return;
  }

  if (traj_x.shape() != traj_y.shape()) {
    RCLCPP_WARN(
      node->get_logger(),
      "[LSTMCritic] traj_x.shape != traj_y.shape, skip scoring.");
    return;
  }

  const auto & shape = traj_x.shape();
  const size_t num_traj = shape[0];
  const size_t time_steps = shape[1];

  if (num_traj == 0 || time_steps == 0) {
    return;
  }

  // 拷贝一份最新预测消息，避免评分时持锁
  std::shared_ptr<mybot::msg::ObstacleTrajectoryArray> local_msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_msg = latest_msg_;
  }

  if (!local_msg || local_msg->trajectories.empty()) {
    RCLCPP_DEBUG(
      node->get_logger(),
      "[LSTMCritic] No obstacle trajectories, skip.");
    return;
  }

  const size_t update_count = std::min(
    num_traj, static_cast<size_t>(costs.size()));

  const double danger_r = danger_radius_;
  const double tube_r = path_radius_;
  const double big_collision_factor = 1000.0;
  const double eps_vel = 1e-6;

  // 当前滚动窗口的时间长度
  const double horizon_T =
    static_cast<double>(time_steps) * static_cast<double>(model_dt) + 1e-6;

  // ===== 全局开关：仅在机器人当前位姿落在某条预测轨迹通道内时才启用本 critic =====
  bool robot_on_path = false;
  {
    // 所有 sample 的起点相同，这里用第 0 条 sample 的 t=0 作为当前位姿
    const double x0 = static_cast<double>(traj_x(0, 0));
    const double y0 = static_cast<double>(traj_y(0, 0));

    for (const auto & obs_traj : local_msg->trajectories) {
      const auto & poses = obs_traj.future_poses;
      const auto & times = obs_traj.future_times;
      const size_t n = poses.size();
      if (n == 0 || times.size() != n) {
        continue;
      }

      for (size_t j = 0; j < n; ++j) {
        const double t = static_cast<double>(times[j]);
        if (t > horizon_T) {
          break;
        }

        const double ox = static_cast<double>(poses[j].position.x);
        const double oy = static_cast<double>(poses[j].position.y);

        const double dx = ox - x0;
        const double dy = oy - y0;
        const double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < tube_r) {
          robot_on_path = true;
          break;
        }
      }

      if (robot_on_path) {
        break;
      }
    }
  }

  if (!robot_on_path) {
    RCLCPP_DEBUG(
      node->get_logger(),
      "[LSTMCritic] robot not inside any dynamic obstacle tube "
      "(dist > path_radius=%.3f), skip scoring.",
      tube_r);
    return;
  }

  // ===== 主循环：对每条采样轨迹打分 =====
  for (size_t i = 0; i < update_count; ++i) {
    if (i >= static_cast<size_t>(costs.size())) {
      break;
    }

    float & cost_ref = costs(i);

    bool has_collision = false;
    double earliest_collision_time = std::numeric_limits<double>::infinity();

    // tube 内停留时间相关：t_exit 表示第一次离开通道的时间
    bool in_tube = true;
    double t_exit = horizon_T;  // 默认认为一直在通道内

    // 用于统计运动方向偏好：累计 cos(theta)
    double cos_acc = 0.0;
    size_t cos_count = 0;

    // 估算机器人速度，使用相邻时间步差分
    double prev_rx = 0.0;
    double prev_ry = 0.0;
    bool has_prev = false;

    for (size_t t = 0; t < time_steps; t += static_cast<size_t>(trajectory_point_step_)) {
      const double rx = static_cast<double>(traj_x(i, t));
      const double ry = static_cast<double>(traj_y(i, t));
      const double t_now = static_cast<double>(t) * static_cast<double>(model_dt);

      // 利用相邻采样点估算当前速度向量
      double vrx = 0.0;
      double vry = 0.0;
      double speed = 0.0;

      if (has_prev) {
        const double dx_r = rx - prev_rx;
        const double dy_r = ry - prev_ry;
        const double ds = std::sqrt(dx_r * dx_r + dy_r * dy_r);
        speed = (model_dt > 1e-6) ? (ds / static_cast<double>(model_dt)) : 0.0;
        if (model_dt > 1e-6) {
          vrx = dx_r / static_cast<double>(model_dt);
          vry = dy_r / static_cast<double>(model_dt);
        }
      }
      prev_rx = rx;
      prev_ry = ry;
      has_prev = true;

      // 在当前时间 t_now，搜索最近的一个障碍物点及其速度
      double best_dist_sq = std::numeric_limits<double>::infinity();
      double best_ox = 0.0;
      double best_oy = 0.0;
      double best_vox = 0.0;
      double best_voy = 0.0;
      double best_obs_speed = 0.0;

      for (const auto & obs_traj : local_msg->trajectories) {
        const auto & poses = obs_traj.future_poses;
        const auto & times = obs_traj.future_times;

        const size_t n = poses.size();
        if (n == 0 || times.size() != n) {
          continue;
        }

        // 找到 times[k] >= t_now 的第一个 k
        size_t k = n - 1;
        for (size_t j = 0; j < n; ++j) {
          if (static_cast<double>(times[j]) >= t_now) {
            k = j;
            break;
          }
        }

        const auto & p = poses[k];
        const double ox = static_cast<double>(p.position.x);
        const double oy = static_cast<double>(p.position.y);

        const double dx = rx - ox;
        const double dy = ry - oy;
        const double dist_sq = dx * dx + dy * dy;

        if (dist_sq < best_dist_sq) {
          best_dist_sq = dist_sq;
          best_ox = ox;
          best_oy = oy;

          // 用 k 和 k-1 的差分估算该障碍物在该处速度
          double vox = 0.0;
          double voy = 0.0;
          double obs_speed = 0.0;

          if (k > 0) {
            const auto & p_prev = poses[k - 1];
            const double ox_prev = static_cast<double>(p_prev.position.x);
            const double oy_prev = static_cast<double>(p_prev.position.y);
            const double dt_o =
              static_cast<double>(times[k]) - static_cast<double>(times[k - 1]);
            if (std::fabs(dt_o) > 1e-6) {
              vox = (ox - ox_prev) / dt_o;
              voy = (oy - oy_prev) / dt_o;
              obs_speed = std::sqrt(vox * vox + voy * voy);
            }
          }

          best_vox = vox;
          best_voy = voy;
          best_obs_speed = obs_speed;
        }
      }  // for obs_traj

      if (!std::isfinite(best_dist_sq)) {
        continue;
      }

      const double dist = std::sqrt(best_dist_sq);

      // 1）碰撞判定：进入 danger_radius 视为发生碰撞
      if (dist <= danger_r) {
        has_collision = true;
        earliest_collision_time =
          std::min(earliest_collision_time, t_now);
        // 这里不直接 break，方便继续统计一些辅助信息
      }

      // 2）记录第一次离开通道的时间 t_exit
      if (in_tube) {
        if (dist > tube_r) {
          in_tube = false;
          t_exit = t_now;
        }
      }

      // 3）方向偏好统计：仅在通道内且机器人与障碍物都有明显速度时才参与计算
      if (dist <= tube_r &&
          speed > eps_vel &&
          best_obs_speed > eps_vel) {
        const double dot = vrx * best_vox + vry * best_voy;
        const double denom = speed * best_obs_speed;
        double cos_theta = dot / denom;
        // 数值安全范围约束
        if (cos_theta > 1.0) {
          cos_theta = 1.0;
        } else if (cos_theta < -1.0) {
          cos_theta = -1.0;
        }

        cos_acc += cos_theta;
        ++cos_count;
      }

    }  // for t

    // ===== 3. 硬碰撞：直接给出较大代价作为近似约束 =====
    if (has_collision) {
      double early_factor = 1.0;
      if (std::isfinite(earliest_collision_time) && earliest_collision_time > 0.0) {
        const double t_norm = earliest_collision_time / horizon_T;  // [0,1]
        // 越早碰撞，t_norm 越小，指数衰减越慢，early_factor 越大
        early_factor = std::exp(-ttr_beta_ * t_norm);
      }

      const double collision_delta = weight_ * big_collision_factor * early_factor;
      cost_ref += static_cast<float>(collision_delta);

      RCLCPP_DEBUG(
        node->get_logger(),
        "[LSTMCritic] sample %zu: collision predicted, earliest_t=%.3f, "
        "delta_cost=%.3f",
        i, earliest_collision_time, collision_delta);
      // 发生碰撞的 sample 不再叠加“通道脱离时间”和方向相关的软代价
      continue;
    }

    // ===== 4. 未进入危险圈的 sample：根据“通道脱离时间 + 运动方向”增加软惩罚 =====

    // 4.1 t_exit：第一次离开轨迹通道的时间，越早离开通道越好
    double t_exit_norm = t_exit / horizon_T;
    if (t_exit_norm < 0.0) {
      t_exit_norm = 0.0;
    } else if (t_exit_norm > 1.0) {
      t_exit_norm = 1.0;
    }

    // 使用指数函数构造脱离时间的权重
    const double J_exit = std::exp(ttr_beta_ * t_exit_norm) - 1.0;

    // 4.2 方向偏好：与障碍物同向“抢道”时罚得更重，反向或侧向避让时适当减轻
    double J_dir = 1.0;
    if (cos_count > 0) {
      const double mean_cos = cos_acc / static_cast<double>(cos_count);

      if (mean_cos > 0.0) {
        // >0 说明速度方向整体偏同向
        J_dir = 1.0 + forward_penalty_scale_ * mean_cos;  // >= 1
      } else {
        // <=0 说明存在一定程度的对向或侧向避让
        J_dir = 1.0 + backward_discount_ * mean_cos;      // mean_cos<=0 → <=1
      }

      // 下限保护，避免出现 0 或负值
      if (J_dir < 0.1) {
        J_dir = 0.1;
      }
    }

    // 4.3 综合风险指标
    const double risk = J_exit * J_dir;

    if (risk <= 0.0) {
      // 例如几乎行为就是立即远离通道且方向合理，则视为本 critic 对其不施加代价
      continue;
    }

    const double raised = std::pow(risk, static_cast<double>(power_));
    const double delta = weight_ * raised;
    cost_ref += static_cast<float>(delta);

    RCLCPP_DEBUG(
      node->get_logger(),
      "[LSTMCritic] sample %zu: t_exit_norm=%.3f, J_exit=%.3f, J_dir=%.3f, "
      "risk=%.3f, delta=%.3f",
      i, t_exit_norm, J_exit, J_dir, risk, delta);
  }  // for i

  RCLCPP_DEBUG(
    node->get_logger(),
    "[LSTMCritic] scoring done: num_traj=%zu, time_steps=%zu, obstacles=%zu",
    num_traj, time_steps, local_msg->trajectories.size());
}

}  // namespace mppi_lstm_critic

PLUGINLIB_EXPORT_CLASS(
  mppi_lstm_critic::LSTMDynamicObstacleCritic,
  mppi::critics::CriticFunction)

