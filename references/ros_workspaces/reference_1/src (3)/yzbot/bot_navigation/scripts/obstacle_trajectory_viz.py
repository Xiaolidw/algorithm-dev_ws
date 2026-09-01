#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
obstacle_trajectory_viz.py

功能：
- 订阅 /obstacle_trajectory (mybot/msg/ObstacleTrajectoryArray)
- 把“预测出来的 future_poses”转成 MarkerArray
- 方便在 RViz 中直接看 LSTM 预测的轨迹（不是实际路径）
"""

from typing import List

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from mybot.msg import ObstacleTrajectoryArray


class ObstacleTrajectoryVizNode(Node):
    def __init__(self):
        super().__init__("obstacle_trajectory_viz")

        # ✅ 订阅的就是 LSTM 预测话题
        self.sub_traj = self.create_subscription(
            ObstacleTrajectoryArray,
            "/obstacle_trajectory",
            self.traj_callback,
            10,
        )

        # 发布给 RViz 的 MarkerArray
        self.pub_markers = self.create_publisher(
            MarkerArray,
            "/obstacle_trajectory_markers",
            10,
        )

        self.line_width = 0.03
        self.point_scale = 0.06
        self.traj_lifetime = 0.3  # 秒

        self.get_logger().info("✅ ObstacleTrajectory 预测可视化节点已启动（只显示 LSTM 预测）")

    def traj_callback(self, msg: ObstacleTrajectoryArray):
        """
        将每条 ObstacleTrajectory.future_poses（纯预测点）
        转成两种 Marker：
        1) LINE_STRIP：整条预测轨迹
        2) SPHERE_LIST：每个预测时刻的小点
        """
        markers = MarkerArray()

        # 清空之前的 marker，避免 ID 管理麻烦
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        frame_id = msg.header.frame_id if msg.header.frame_id else "map"

        for idx, traj in enumerate(msg.trajectories):
            color = self._color_for_index(idx)

            # ---- 1. 预测轨迹线条 ----
            line_marker = Marker()
            line_marker.header.frame_id = frame_id
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "pred_traj_line"
            line_marker.id = idx * 2
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD

            line_marker.scale.x = self.line_width
            line_marker.color = color
            line_marker.color.a = 0.9

            line_marker.lifetime.sec = int(self.traj_lifetime)
            line_marker.lifetime.nanosec = int(
                (self.traj_lifetime - int(self.traj_lifetime)) * 1e9
            )

            for pose in traj.future_poses:
                p = Point()
                p.x = pose.position.x
                p.y = pose.position.y
                p.z = pose.position.z
                line_marker.points.append(p)

            markers.markers.append(line_marker)

            # ---- 2. 预测点（小球） ----
            point_marker = Marker()
            point_marker.header.frame_id = frame_id
            point_marker.header.stamp = line_marker.header.stamp
            point_marker.ns = "pred_traj_points"
            point_marker.id = idx * 2 + 1
            point_marker.type = Marker.SPHERE_LIST
            point_marker.action = Marker.ADD

            point_marker.scale.x = self.point_scale
            point_marker.scale.y = self.point_scale
            point_marker.scale.z = self.point_scale
            point_marker.color = color
            point_marker.color.a = 0.9
            point_marker.lifetime = line_marker.lifetime

            for pose in traj.future_poses:
                p = Point()
                p.x = pose.position.x
                p.y = pose.position.y
                p.z = pose.position.z
                point_marker.points.append(p)

            markers.markers.append(point_marker)

        self.pub_markers.publish(markers)

    def _color_for_index(self, idx: int) -> ColorRGBA:
        color = ColorRGBA()
        palette: List[List[float]] = [
            [0.1, 0.8, 0.1],  # 绿
            [0.8, 0.1, 0.1],  # 红
            [0.1, 0.4, 0.9],  # 蓝
            [0.9, 0.7, 0.1],  # 黄
            [0.7, 0.1, 0.9],  # 紫
        ]
        r, g, b = palette[idx % len(palette)]
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = 0.9
        return color


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTrajectoryVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🔚 obstacle_trajectory_viz 手动退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
