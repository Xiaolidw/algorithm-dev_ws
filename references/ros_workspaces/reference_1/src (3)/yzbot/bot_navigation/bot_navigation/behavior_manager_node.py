#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
behavior_manager_node.py

行为层（Supervisor）——在 Nav2 / MPPI 之外，再加一层“硬闸门 + 人性化等待”。

在本版本中，我们“串联”在最终速度链路中：

Nav2 原生链路保持不动：
  controller_server     -> /cmd_vel_nav
  nav2_velocity_smoother -> /cmd_vel
  behavior_server       -> /cmd_vel

BehaviorManager 串在底盘前面：
  订阅：/cmd_vel         （Nav2 最后的输出，包括平滑 + recovery 行为）
  发布：/cmd_vel_robot   （唯一真正喂给 Gazebo 差速插件的话题）

Gazebo 底盘：
  differential_drive_controller:
    command_topic = cmd_vel_robot

逻辑：
1. CRUISE 模式：
   - 每个周期把最新收到的 /cmd_vel 原样转发到 /cmd_vel_robot；
2. WAIT 模式：
   - 每个周期往 /cmd_vel_robot 发布 0 速度，实现“硬刹车”；
3. 动态障碍预测和 gap 等待逻辑与之前版本一致。
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from mybot.msg import ObstacleTrajectoryArray

from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quaternion(q):
    """从四元数计算 yaw。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class BehaviorManagerNode(Node):
    def __init__(self):
        super().__init__("behavior_manager")

        # ================== 参数 ==================
        # 与当前 nav2 / Gazebo 链路对应：
        #   - Nav2 最终输出到 /cmd_vel
        #   - Gazebo diff_drive 插件吃 /cmd_vel_robot（我们刚改过 URDF / world）
        self.declare_parameter("cmd_in_topic", "/cmd_vel")
        self.declare_parameter("cmd_out_topic", "/cmd_vel_robot")
        self.declare_parameter("state_topic", "/behavior_state")

        # 安全区
        self.declare_parameter("safety_radius", 0.7)
        self.declare_parameter("danger_radius", 0.55)
        self.declare_parameter("horizon_time", 3.0)
        self.declare_parameter("sim_dt", 0.05)

        # cmd 超时（无新命令视为需要停）
        self.declare_parameter("cmd_timeout", 0.5)

        # WAIT 抖动抑制
        self.declare_parameter("min_wait_time", 1.0)

        # 帧名
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        # 前方走廊 gap 等待参数
        self.declare_parameter("enable_gap_wait", True)
        self.declare_parameter("gap_front_min", 0.55)
        self.declare_parameter("gap_front_max", 2.0)
        self.declare_parameter("gap_lateral_half_width", 0.75)

        # 当前点在动障未来轨道上的判定半径
        self.declare_parameter("path_radius", 0.45)

        # WAIT 模式下用于“探测”的虚拟前向速度
        self.declare_parameter("probe_speed", 0.3)

        # 读取参数
        self.cmd_in_topic = self.get_parameter("cmd_in_topic").get_parameter_value().string_value
        self.cmd_out_topic = self.get_parameter("cmd_out_topic").get_parameter_value().string_value
        self.state_topic = self.get_parameter("state_topic").get_parameter_value().string_value

        self.safety_radius = float(self.get_parameter("safety_radius").value)
        self.danger_radius = float(self.get_parameter("danger_radius").value)
        self.horizon_time = float(self.get_parameter("horizon_time").value)
        self.sim_dt = float(self.get_parameter("sim_dt").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.min_wait_time = float(self.get_parameter("min_wait_time").value)

        self.global_frame = self.get_parameter("global_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value

        self.enable_gap_wait = bool(self.get_parameter("enable_gap_wait").value)
        self.gap_front_min = float(self.get_parameter("gap_front_min").value)
        self.gap_front_max = float(self.get_parameter("gap_front_max").value)
        self.gap_lateral_half_width = float(self.get_parameter("gap_lateral_half_width").value)

        self.path_radius = float(self.get_parameter("path_radius").value)
        self.probe_speed = float(self.get_parameter("probe_speed").value)

        if self.sim_dt <= 0.0:
            self.sim_dt = 0.05

        # 只打一遍 TF 警告用的 flag
        self.tf_warned = False

        self.get_logger().info(
            "[BehaviorManager] 启动\n"
            f"  cmd_in_topic  = {self.cmd_in_topic}\n"
            f"  cmd_out_topic = {self.cmd_out_topic}\n"
            f"  state_topic   = {self.state_topic}\n"
            f"  safety_radius = {self.safety_radius:.3f}\n"
            f"  danger_radius = {self.danger_radius:.3f}\n"
            f"  horizon_time  = {self.horizon_time:.2f}s, sim_dt={self.sim_dt:.3f}s\n"
            f"  cmd_timeout   = {self.cmd_timeout:.2f}s, min_wait_time={self.min_wait_time:.2f}s\n"
            f"  frames: {self.global_frame} -> {self.base_frame}\n"
            f"  enable_gap_wait={self.enable_gap_wait}, "
            f"gap_front=[{self.gap_front_min:.2f}, {self.gap_front_max:.2f}], "
            f"gap_lateral_half_width={self.gap_lateral_half_width:.2f}, "
            f"path_radius={self.path_radius:.2f}, "
            f"probe_speed={self.probe_speed:.2f}"
        )

        # ================== TF ==================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ================== 状态缓存 ==================
        self.last_cmd: Twist = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.has_cmd = False

        self.last_obstacles: Optional[ObstacleTrajectoryArray] = None

        # 行为状态 + 抖动抑制
        self.current_mode: str = "CRUISE"   # "CRUISE" or "WAIT"
        self.mode_change_time = self.get_clock().now()

        # ================== 通讯接口 ==================
        # Nav2 最终输出速度（/cmd_vel）
        self.sub_cmd = self.create_subscription(
            Twist,
            self.cmd_in_topic,
            self.cmd_cb,
            20,
        )

        # 动态障碍预测
        self.sub_traj = self.create_subscription(
            ObstacleTrajectoryArray,
            "/obstacle_trajectory",
            self.traj_cb,
            10,
        )

        # 输出给底盘（/cmd_vel_robot）
        self.pub_cmd = self.create_publisher(
            Twist,
            self.cmd_out_topic,
            20,
        )

        # 行为状态（字符串：CRUISE / WAIT）
        self.pub_state = self.create_publisher(
            String,
            self.state_topic,
            10,
        )

        # 定时器：固定频率做风险评估 + 下发速度
        self.timer = self.create_timer(self.sim_dt, self.timer_cb)

        self.get_logger().info("[BehaviorManager] 行为管理节点已就绪。")

    # ========== 回调：收到 /cmd_vel ========== #
    def cmd_cb(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        self.has_cmd = True

    # ========== 回调：收到动态障碍预测 ========== #
    def traj_cb(self, msg: ObstacleTrajectoryArray):
        self.last_obstacles = msg

    # ========== 定时器：核心决策逻辑 ========== #
    def timer_cb(self):
        now = self.get_clock().now()
        old_mode = self.current_mode

        # ---------- 1. 先算“候选模式” ----------
        proposed_mode = "CRUISE"

        # 1）没有有效 cmd：那就停
        if (not self.has_cmd) or ((now - self.last_cmd_time) > Duration(seconds=self.cmd_timeout)):
            proposed_mode = "WAIT"

        # 2）没有障碍预测：直接放行（不干预）
        elif self.last_obstacles is None or not self.last_obstacles.trajectories:
            proposed_mode = "CRUISE"

        else:
            # 3）获取机器人当前位姿
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.global_frame,
                    self.base_frame,
                    rclpy.time.Time()
                )
            except TransformException as ex:
                if not self.tf_warned:
                    self.get_logger().warn(
                        f"[BehaviorManager] TF {self.global_frame}->{self.base_frame} 查询失败：{ex}"
                    )
                    self.tf_warned = True
                proposed_mode = "CRUISE"
            else:
                x0 = tf.transform.translation.x
                y0 = tf.transform.translation.y
                yaw0 = yaw_from_quaternion(tf.transform.rotation)

                # A）当前点是否在动障未来轨道上
                robot_on_path = self._is_robot_on_future_obstacle_path(
                    x0, y0, self.last_obstacles)

                # B）用于碰撞预测的“仿真速度指令”
                sim_cmd = self.last_cmd
                if self.current_mode == "WAIT":
                    sim_cmd = Twist()
                    sim_cmd.linear.x = self.probe_speed
                    sim_cmd.angular.z = 0.0
                    self.get_logger().debug(
                        f"[BehaviorManager] WAIT 模式下使用探测速度进行预测: v={self.probe_speed:.2f} m/s"
                    )

                # C）安全硬闸门：未来 horizon 内是否会撞进 danger_radius
                will_hit = self._predict_collision(
                    x0, y0, yaw0, sim_cmd, self.last_obstacles
                )

                if will_hit and not robot_on_path:
                    proposed_mode = "WAIT"
                else:
                    proposed_mode = "CRUISE"

                # D）gap 等待：前方走廊是否被占用
                if self.enable_gap_wait and proposed_mode != "WAIT" and not robot_on_path:
                    if self._is_forward_corridor_blocked(x0, y0, yaw0, self.last_obstacles):
                        proposed_mode = "WAIT"

        # ---------- 2. 抖动抑制 + 最终模式 ----------
        final_mode = proposed_mode

        if old_mode == "WAIT" and proposed_mode == "CRUISE":
            dt = (now - self.mode_change_time).nanoseconds * 1e-9
            if dt < self.min_wait_time:
                final_mode = "WAIT"

        if final_mode != old_mode:
            self.current_mode = final_mode
            self.mode_change_time = now
            self.get_logger().info(f"[BehaviorManager] 模式切换: {old_mode} -> {final_mode}")

        # ---------- 3. 根据 final_mode 下发速度 ----------
        # 这里我们是真正“串联”在底盘前：
        #   - CRUISE：把 last_cmd 原样转发给 /cmd_vel_robot
        #   - WAIT：强制发 0 速度
        if final_mode == "WAIT":
            zero = Twist()
            self.pub_cmd.publish(zero)
        else:
            # 如果还没收到任何有效 cmd（例如 Nav2 还没开始），就发 0
            if self.has_cmd:
                self.pub_cmd.publish(self.last_cmd)
            else:
                zero = Twist()
                self.pub_cmd.publish(zero)

        # ---------- 4. 发布状态字符串 ----------
        state_msg = String()
        state_msg.data = final_mode
        self.pub_state.publish(state_msg)

    # ========== 碰撞预测：沿当前 cmd 仿真机器人轨迹 ==========
    def _predict_collision(
        self,
        x0: float,
        y0: float,
        yaw0: float,
        cmd: Twist,
        obs_array: ObstacleTrajectoryArray,
    ) -> bool:
        v = cmd.linear.x
        w = cmd.angular.z

        if abs(v) < 1e-3 and abs(w) < 1e-3:
            return False

        steps = max(1, int(self.horizon_time / self.sim_dt))
        x = x0
        y = y0
        yaw = yaw0

        for step in range(1, steps + 1):
            t_future = step * self.sim_dt

            x += v * math.cos(yaw) * self.sim_dt
            y += v * math.sin(yaw) * self.sim_dt
            yaw += w * self.sim_dt

            min_dist = float("inf")

            for obs_traj in obs_array.trajectories:
                poses = obs_traj.future_poses
                times = obs_traj.future_times
                n = len(poses)
                if n == 0:
                    continue

                idx = n - 1
                if len(times) == n:
                    for j in range(n):
                        if times[j] >= t_future:
                            idx = j
                            break

                px = poses[idx].position.x
                py = poses[idx].position.y

                dx = x - px
                dy = y - py
                dist = math.hypot(dx, dy)

                if dist < min_dist:
                    min_dist = dist

            if min_dist < self.danger_radius:
                self.get_logger().debug(
                    f"[BehaviorManager] 预测 t={t_future:.2f}s 会进入危险圈，"
                    f"min_dist={min_dist:.3f} < danger_radius={self.danger_radius:.3f}"
                )
                return True

        return False

    # ========== 判断“机器人当前这个点是否在动障未来轨道上” ==========
    def _is_robot_on_future_obstacle_path(
        self,
        x0: float,
        y0: float,
        obs_array: ObstacleTrajectoryArray,
    ) -> bool:
        if obs_array is None or not obs_array.trajectories:
            return False

        for obs_traj in obs_array.trajectories:
            poses = obs_traj.future_poses
            times = obs_traj.future_times
            n = len(poses)
            if n == 0 or len(times) != n:
                continue

            for pose, t in zip(poses, times):
                if t > self.horizon_time:
                    break

                ox = pose.position.x
                oy = pose.position.y

                dx = ox - x0
                dy = oy - y0
                dist = math.hypot(dx, dy)

                if dist < self.path_radius:
                    self.get_logger().debug(
                        "[BehaviorManager] robot_on_path: 当前点将在 t=%.2fs 内被动障扫过，dist=%.3f"
                        % (t, dist)
                    )
                    return True

        return False

    # ========== 前方走廊是否被动障占用（gap 等待） ==========
    def _is_forward_corridor_blocked(
        self,
        x0: float,
        y0: float,
        yaw0: float,
        obs_array: ObstacleTrajectoryArray,
    ) -> bool:
        if obs_array is None or not obs_array.trajectories:
            return False

        cos_yaw = math.cos(yaw0)
        sin_yaw = math.sin(yaw0)

        for obs_traj in obs_array.trajectories:
            poses = obs_traj.future_poses
            times = obs_traj.future_times
            n = len(poses)
            if n == 0 or len(times) != n:
                continue

            for pose, t in zip(poses, times):
                if t > self.horizon_time:
                    break

                ox = pose.position.x
                oy = pose.position.y

                dx = ox - x0
                dy = oy - y0

                forward = dx * cos_yaw + dy * sin_yaw
                lateral = -dx * sin_yaw + dy * cos_yaw

                if forward < self.gap_front_min or forward > self.gap_front_max:
                    continue

                if abs(lateral) > self.gap_lateral_half_width:
                    continue

                self.get_logger().debug(
                    "[BehaviorManager] gap_wait: 前方走廊被动障占用 - "
                    f"obs at t={t:.2f}s, forward={forward:.2f}, lateral={lateral:.2f}"
                )
                return True

        return False


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[BehaviorManager] 手动退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

