#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LSTM 解析参数式动态障碍物轨迹预测节点（自包含版本）

功能：
- 订阅 /object_detection/object_poses (mybot/msg/ObjectPoseArray)
- 只关注 type == "obstacle_dynamic" 的模型（例如 obstacle2 / obstacle3 / obstacle4）
- 为每个障碍物维护最近一段观测轨迹（默认 16 帧）
- 使用训练好的 LSTMThetaModel，从观测 (x,y) 序列回归解析参数 θ：
    θ = [axis_flag, min_val, max_val, fixed_coord, speed, s_amp, s_waves]
- 再用 θ + 当前最新观测位置，按解析式（往返 + S 型）生成未来若干秒的预测轨迹
- 将结果发布到 /obstacle_trajectory (mybot/msg/ObstacleTrajectoryArray)
- 同时通过日志 + /obstacle_theta_debug 话题输出 θ 供调试
"""

import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, Float32MultiArray
from geometry_msgs.msg import Pose

from mybot.msg import (
    ObjectPoseArray,
    ObstacleTrajectory,
    ObstacleTrajectoryArray,
)

import torch
import torch.nn as nn


# ============================================================
# 自带 LSTMThetaModel 定义（与训练时保持一致）
# ============================================================

class LSTMThetaModel(nn.Module):
    """
    输入:  (batch, obs_len, 2)   ->  观测到的历史轨迹 (x, y)
    输出:  (batch, 7)            ->  解析参数 θ:
        [axis_flag, min_val, max_val, fixed, speed, s_amp, s_waves]
    """

    def __init__(
        self,
        input_dim: int = 2,
        hidden_dim: int = 64,
        num_layers: int = 2,
        param_dim: int = 7,
    ):
        super().__init__()
        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        self.param_dim = param_dim

        self.lstm = nn.LSTM(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
        )
        self.fc = nn.Linear(hidden_dim, param_dim)

    def forward(self, obs_seq: torch.Tensor) -> torch.Tensor:
        """
        obs_seq: (batch, obs_len, 2)
        return:  (batch, 7)
        """
        lstm_out, (h, c) = self.lstm(obs_seq)      # lstm_out: (B, obs_len, hidden_dim)
        last = lstm_out[:, -1, :]                  # (B, hidden_dim)
        theta = self.fc(last)                      # (B, 7)
        return theta


# ============================================================
# 主节点
# ============================================================

class LSTMPredictorNode(Node):
    def __init__(self):
        super().__init__("lstm_obstacle_predictor")

        # ---------------- ROS 参数 ----------------
        # 预测时间窗口 & 步长
        self.declare_parameter("prediction_horizon", 3.0)    # 预测 3 秒
        self.declare_parameter("prediction_dt", 0.1)         # 每 0.1s 一个预测点
        self.declare_parameter("frame_id", "map")            # 假定 map 与 world 对齐

        # LSTM 权重路径（放在 bot_navigation 包内部）
        self.declare_parameter(
            "model_path",
            "/home/ros/dev_ws/src/yzbot/training/models/theta_lstm.pth",
        )

        # 是否打印 θ 调试信息
        self.declare_parameter("debug_theta", True)

        self.prediction_horizon = (
            self.get_parameter("prediction_horizon").get_parameter_value().double_value
        )
        self.prediction_dt = (
            self.get_parameter("prediction_dt").get_parameter_value().double_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        self.debug_theta = (
            self.get_parameter("debug_theta").get_parameter_value().bool_value
        )

        if self.prediction_horizon <= 0.0:
            self.prediction_horizon = 3.0
        if self.prediction_dt <= 0.0:
            self.prediction_dt = 0.1

        self.num_steps = max(1, int(self.prediction_horizon / self.prediction_dt))

        # ---------------- 加载 LSTM 模型 ----------------
        self.device = torch.device("cpu")  # 为稳定起见，默认用 CPU
        self.model, self.obs_len, self.dt_model = self._load_lstm_model(self.model_path)
        self.min_history_len = self.obs_len  # 至少需要 obs_len 帧历史

        if abs(self.dt_model - self.prediction_dt) > 1e-3:
            self.get_logger().warn(
                f"模型训练 dt={self.dt_model:.3f}s 与当前 prediction_dt={self.prediction_dt:.3f}s 不一致，"
                f"预测时间轴会有一点缩放，一般问题不大。"
            )

        # 每个障碍物的历史观测：id -> List[(t, x, y)]
        self.histories: Dict[str, List[Tuple[float, float, float]]] = {}

        # ---------------- 订阅 & 发布 ----------------
        self.sub_poses = self.create_subscription(
            ObjectPoseArray,
            "/object_detection/object_poses",
            self.object_pose_callback,
            50,
        )

        self.pub_traj = self.create_publisher(
            ObstacleTrajectoryArray,
            "/obstacle_trajectory",
            10,
        )

        # θ 调试话题：Float32MultiArray
        # data = [axis_flag, min, max, fixed, speed, s_amp, s_waves]
        self.theta_pub = self.create_publisher(
            Float32MultiArray,
            "/obstacle_theta_debug",
            10,
        )

        # 定时预测发布：步长用 prediction_dt
        self.timer = self.create_timer(self.prediction_dt, self.timer_callback)

        self.get_logger().info(
            "🚀 LSTM 解析参数式障碍物轨迹预测节点已启动\n"
            f"  - prediction_horizon = {self.prediction_horizon:.2f} s\n"
            f"  - prediction_dt      = {self.prediction_dt:.3f} s\n"
            f"  - num_steps          = {self.num_steps}\n"
            f"  - frame_id           = {self.frame_id}\n"
            f"  - model_path         = {self.model_path}\n"
            f"  - LSTM obs_len       = {self.obs_len}\n"
            f"  - LSTM dt_model      = {self.dt_model:.3f} s\n"
            f"  - debug_theta        = {self.debug_theta}"
        )

    # ============================================================
    # 模型加载
    # ============================================================
    def _load_lstm_model(self, path: str):
        try:
            ckpt = torch.load(path, map_location=self.device)
        except Exception as e:
            self.get_logger().error(f"加载 LSTM 权重失败: {path}\n{e}")
            raise

        obs_len = ckpt.get("obs_len", 16)
        dt_model = ckpt.get("dt", 0.1)
        hidden_dim = ckpt.get("hidden_dim", 64)
        num_layers = ckpt.get("num_layers", 2)
        param_dim = ckpt.get("param_dim", 7)

        model = LSTMThetaModel(
            input_dim=2,
            hidden_dim=hidden_dim,
            num_layers=num_layers,
            param_dim=param_dim,
        ).to(self.device)

        model.load_state_dict(ckpt["model_state_dict"])
        model.eval()

        self.get_logger().info(
            f"成功加载 LSTM 权重: obs_len={obs_len}, dt={dt_model:.3f}, "
            f"hidden_dim={hidden_dim}, num_layers={num_layers}, param_dim={param_dim}"
        )

        return model, obs_len, dt_model

    # ============================================================
    # 工具函数：1D 线段往返反射运动
    # ============================================================
    @staticmethod
    def _reflect_1d(x0: float, v: float, min_val: float, max_val: float, t: float) -> float:
        """
        在 [min_val, max_val] 上做 1D 往返运动的解析式解：
        - 初始位置 x0
        - 匀速 v（m/s），可以为正或负
        - 时间 t >= 0
        返回：x(t)，始终处于 [min_val, max_val] 内
        """
        L = max_val - min_val
        if L <= 1e-6:
            return x0

        x0_clamped = min(max(x0, min_val), max_val)
        rel0 = x0_clamped - min_val

        s = rel0 + v * t
        cycle = 2.0 * L

        m = math.fmod(s, cycle)
        if m < 0.0:
            m += cycle

        if m <= L:
            rel = m
        else:
            rel = 2.0 * L - m

        return min_val + rel

    # ============================================================
    # 回调：接收 Gazebo 发布的障碍物位姿
    # ============================================================
    def object_pose_callback(self, msg: ObjectPoseArray):
        # 统一用 header 时间戳（秒）
        t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        for obj in msg.objects:
            if obj.type != "obstacle_dynamic":
                continue

            obs_id = obj.id
            x = float(obj.pose.position.x)
            y = float(obj.pose.position.y)

            history = self.histories.get(obs_id)
            if history is None:
                history = []
                self.histories[obs_id] = history

            history.append((t_msg, x, y))

            # 只保留最近 obs_len 帧
            if len(history) > self.obs_len:
                del history[0 : len(history) - self.obs_len]

    # ============================================================
    # 核心：利用 LSTM 输出解析 θ，再生成未来轨迹
    # ============================================================
    def _predict_theta_from_history(self, obs_id: str, history: List[Tuple[float, float, float]]):
        """
        输入：某个障碍物的历史观测列表 [(t,x,y), ...]，长度 >= obs_len
        输出：解析参数 θ = (axis, min_val, max_val, fixed, speed, s_amp, s_waves)
        做了基本的 clamp，保证参数稳定。
        """
        assert len(history) >= self.obs_len

        # 取最近 obs_len 帧，按时间顺序构造 (x,y) 序列
        recent = history[-self.obs_len :]
        coords = [[x, y] for (_, x, y) in recent]

        obs_tensor = torch.tensor(
            [coords], dtype=torch.float32, device=self.device
        )  # (1, obs_len, 2)

        with torch.no_grad():
            theta_pred = self.model(obs_tensor)[0]  # (7,)
        theta_pred = theta_pred.cpu().tolist()

        axis_flag = theta_pred[0]
        raw_min = theta_pred[1]
        raw_max = theta_pred[2]
        fixed = theta_pred[3]
        speed = abs(theta_pred[4])  # 速度取绝对值
        s_amp = max(0.0, theta_pred[5])
        s_waves = max(0.0, theta_pred[6])

        # 轴向：<0.5 → x 轴；>=0.5 → y 轴
        axis = "x" if axis_flag < 0.5 else "y"

        # min / max 排序，确保 min <= max
        min_val = min(raw_min, raw_max)
        max_val = max(raw_min, raw_max)

        # 基础 clamp：防止偶发异常
        min_val = max(-20.0, min_val)
        max_val = min(20.0, max_val)
        if min_val > max_val:
            min_val, max_val = max_val, min_val

        fixed = max(-10.0, min(10.0, fixed))

        # 速度 clamp：0.1 ~ 0.8 m/s
        speed = max(0.1, min(speed, 0.8))

        # S 型参数 clamp
        s_amp = max(0.0, min(s_amp, 2.0))
        s_waves = max(0.0, min(s_waves, 3.0))

        # 日志监控 θ
        if self.debug_theta:
            self.get_logger().info(
                f"[theta] id={obs_id}  axis={axis} "
                f"min={min_val:.3f}, max={max_val:.3f}, fixed={fixed:.3f}, "
                f"speed={speed:.3f}, s_amp={s_amp:.3f}, s_waves={s_waves:.3f}"
            )

        #发布到调试话题 /obstacle_theta_debug
        msg = Float32MultiArray()
        # data 顺序固定为 [axis_flag, min, max, fixed, speed, s_amp, s_waves]
        axis_flag_out = 0.0 if axis == "x" else 1.0
        msg.data = [
            float(axis_flag_out),
            float(min_val),
            float(max_val),
            float(fixed),
            float(speed),
            float(s_amp),
            float(s_waves),
        ]
        self.theta_pub.publish(msg)

        return axis, min_val, max_val, fixed, speed, s_amp, s_waves

    def _generate_future_trajectory(
        self,
        obs_id: str,
        history: List[Tuple[float, float, float]],
    ) -> ObstacleTrajectory:
        """
        核心函数：
        - 用历史轨迹 -> LSTM 拟合 θ
        - 用 θ + 当前最新位置 -> 生成未来轨迹
        """
        axis, min_val, max_val, fixed, speed, s_amp, s_waves = \
            self._predict_theta_from_history(obs_id, history)

        # 当前最新观测
        _, cur_x, cur_y = history[-1]

        # 根据主运动方向和最近两帧观测估计运动方向符号
        if len(history) >= 2:
            _, prev_x, prev_y = history[-2]
        else:
            prev_x, prev_y = cur_x, cur_y

        if axis == "x":
            d = cur_x - prev_x
            axis_pos0 = cur_x
        else:
            d = cur_y - prev_y
            axis_pos0 = cur_y

        if abs(d) < 1e-4:
            # 如果最近两帧几乎不动，就用“离哪头近就朝哪头”的规则估计方向
            if abs(max_val - axis_pos0) < abs(axis_pos0 - min_val):
                dir_sign = -1.0
            else:
                dir_sign = 1.0
        else:
            dir_sign = 1.0 if d >= 0.0 else -1.0

        v = dir_sign * speed  # 带符号速度

        traj_msg = ObstacleTrajectory()
        traj_msg.id = obs_id
        traj_msg.type = "obstacle_dynamic"

        L = max_val - min_val if max_val > min_val else 0.0

        # 从 t = prediction_dt 开始往前推
        for k in range(1, self.num_steps + 1):
            t = self.prediction_dt * float(k)

            # 1D 反射运动
            coord = self._reflect_1d(axis_pos0, v, min_val, max_val, t)

            pose = Pose()
            if axis == "x":
                x = coord
                y_center = fixed
            else:
                x = fixed
                y_center = coord

            # S 型偏移（只有 s_amp > 0 时生效，一般对应 obstacle4）
            if s_amp > 1e-3 and L > 1e-6:
                phase = (coord - min_val) / L
                phase = max(0.0, min(1.0, phase))
                angle = math.pi * (2.0 * phase - 1.0) * s_waves
                offset = s_amp * math.sin(angle)
            else:
                offset = 0.0

            if axis == "x":
                y = y_center + offset
            else:
                # 如果以后需要“沿 y 轴 + S 型”，可以在 x 上加 offset
                y = y_center

            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.3
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            traj_msg.future_poses.append(pose)
            traj_msg.future_times.append(float(t))

        return traj_msg

    # ============================================================
    # 定时器：遍历所有障碍物，根据历史生成预测轨迹并发布
    # ============================================================
    def timer_callback(self):
        if not self.histories:
            return

        traj_array_msg = ObstacleTrajectoryArray()
        traj_array_msg.header = Header()
        traj_array_msg.header.stamp = self.get_clock().now().to_msg()
        traj_array_msg.header.frame_id = self.frame_id

        for obs_id, history in self.histories.items():
            # 历史长度不够，先不预测
            if len(history) < self.min_history_len:
                continue

            try:
                traj_msg = self._generate_future_trajectory(obs_id, history)
                traj_array_msg.trajectories.append(traj_msg)
            except Exception as e:
                self.get_logger().warn(
                    f"生成障碍物 {obs_id} 未来轨迹时出错：{e}"
                )

        if traj_array_msg.trajectories:
            self.pub_traj.publish(traj_array_msg)
            ids = [t.id for t in traj_array_msg.trajectories]
            self.get_logger().debug(
                f"已发布 LSTM 解析未来轨迹: {ids}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LSTMPredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("LSTM 解析预测节点手动退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

