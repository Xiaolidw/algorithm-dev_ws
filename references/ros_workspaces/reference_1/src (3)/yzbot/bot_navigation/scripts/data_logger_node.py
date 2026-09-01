#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
data_logger_node.py

记录机器人 + 动态障碍物 + 当前 Nav2 路径，用于训练 LSTM 轨迹预测模型。

输出格式（每 0.1 秒一行 JSON）：
{
    "t":  1763900000.123,
    "robot": {"x":..,"y":..},
    "obstacles":[{"id":..,"x":..,"y":..}, ...],
    "path":[ [x1,y1], [x2,y2], ... ]
}

文件存储路径：
~/dev_ws/src/yzbot/bot_navigation/data/log_xxxx.jsonl
"""

import json
import os
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from mybot.msg import ObjectPoseArray


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__("data_logger")

        # ===== 创建数据目录 =====
        self.data_dir = os.path.join(
            os.path.expanduser("~"),
            "dev_ws/src/yzbot/bot_navigation/data"
        )
        os.makedirs(self.data_dir, exist_ok=True)

        # ===== 日志文件 =====
        timestamp = int(time.time())
        self.file_path = os.path.join(self.data_dir, f"log_{timestamp}.jsonl")
        self.file = open(self.file_path, "w")
        self.get_logger().info(f"📁 数据记录文件: {self.file_path}")

        # ===== 状态缓存 =====
        self.robot_pose = None               # (x, y)
        self.obstacles = []                  # [{"id":..., "x":..., "y":...}]
        self.path_xy: List[Tuple[float, float]] = []

        # ===== 订阅 =====
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            20
        )

        self.sub_obj = self.create_subscription(
            ObjectPoseArray,
            "/object_detection/object_poses",
            self.object_callback,
            10
        )

        self.sub_path = self.create_subscription(
            Path,
            "/plan",
            self.path_callback,
            10
        )

        # 每 0.1 秒记录一次
        self.timer = self.create_timer(0.1, self.record_once)

        self.get_logger().info("📡 DataLoggerNode 已启动：开始记录轨迹数据")

    # ======================= 回调 =======================

    def amcl_callback(self, msg):
        self.robot_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y)
        )

    def object_callback(self, msg: ObjectPoseArray):
        """
        只记录动态障碍物：
        - id 包含 obstacle
        - or type 包含 dynamic
        """
        obs = []
        for o in msg.objects:
            oid = o.id.lower()
            otype = o.type.lower()

            if ("obstacle" in oid) or ("dynamic" in otype):
                obs.append({
                    "id": o.id,
                    "x": float(o.pose.position.x),
                    "y": float(o.pose.position.y)
                })

        self.obstacles = obs

    def path_callback(self, msg: Path):
        pts = []
        for p in msg.poses:
            pts.append((
                float(p.pose.position.x),
                float(p.pose.position.y)
            ))
        self.path_xy = pts

    # ======================= 定时记录 =======================

    def record_once(self):
        if self.robot_pose is None:
            return

        t = time.time()

        data = {
            "t": t,
            "robot": {"x": self.robot_pose[0], "y": self.robot_pose[1]},
            "obstacles": self.obstacles,
            "path": self.path_xy
        }

        self.file.write(json.dumps(data) + "\n")
        self.file.flush()

    # ======================= 清理 =======================

    def destroy_node(self):
        self.get_logger().info(f"💾 数据文件已关闭：{self.file_path}")
        try:
            self.file.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

