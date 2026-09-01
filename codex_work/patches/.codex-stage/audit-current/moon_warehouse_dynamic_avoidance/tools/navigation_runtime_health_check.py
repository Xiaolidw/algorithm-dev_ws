#!/usr/bin/env python3
"""Reject navigation tests when the simulator or required data chain is dead."""

import math
import time

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RuntimeHealthCheck(Node):
    def __init__(self):
        super().__init__('navigation_runtime_health_check')
        self.odom_count = 0
        self.decision_count = 0
        self.obstacle_positions = []
        self.map_size = None
        self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.create_subscription(
            Pose, '/moving_obstacle_1/current_pose', self.obstacle_callback, 20)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, 1)
        self.create_subscription(String, '/sipp/decision', self.decision_callback, 20)

    def odom_callback(self, _message):
        self.odom_count += 1

    def decision_callback(self, _message):
        self.decision_count += 1

    def obstacle_callback(self, message):
        self.obstacle_positions.append((
            float(message.position.x), float(message.position.y)))

    def map_callback(self, message):
        self.map_size = (int(message.info.width), int(message.info.height))

    def healthy(self):
        if len(self.obstacle_positions) < 2:
            return False
        first = self.obstacle_positions[0]
        last = self.obstacle_positions[-1]
        displacement = math.hypot(last[0] - first[0], last[1] - first[1])
        return (
            self.odom_count >= 5
            and self.decision_count >= 5
            and len(self.obstacle_positions) >= 5
            and displacement >= 0.03
            and self.map_size == (658, 822)
        )


def main():
    rclpy.init()
    node = RuntimeHealthCheck()
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.healthy():
            print(
                'RUNTIME_HEALTH=PASS '
                f'odom={node.odom_count} obstacle={len(node.obstacle_positions)} '
                f'sipp={node.decision_count} map={node.map_size}')
            node.destroy_node()
            rclpy.shutdown()
            return 0
    print(
        'RUNTIME_HEALTH=FAIL '
        f'odom={node.odom_count} obstacle={len(node.obstacle_positions)} '
        f'sipp={node.decision_count} map={node.map_size}')
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
