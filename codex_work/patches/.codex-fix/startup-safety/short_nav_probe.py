#!/usr/bin/env python3
"""One-shot guarded navigation direction probe for the competition stack."""

import json
import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Probe(Node):
    def __init__(self):
        super().__init__('short_navigation_probe')
        self.pose = None
        self.status = None
        self.goal_pub = self.create_publisher(
            PoseStamped, '/mission/navigation_goal', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(
            String, '/mission/navigation_status', self.status_cb, 10)
        self.cancel = self.create_client(
            Trigger, '/mission/cancel_navigation')

    def odom_cb(self, message):
        self.pose = message.pose.pose

    def status_cb(self, message):
        try:
            self.status = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            pass


def main():
    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + 5.0
    while node.pose is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.pose is None:
        raise RuntimeError('odometry unavailable')

    start_x = float(node.pose.position.x)
    start_y = float(node.pose.position.y)
    dx = float(sys.argv[1]) if len(sys.argv) > 1 else -1.0
    goal_yaw = 0.0 if dx >= 0.0 else math.pi
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = start_x + dx
    goal.pose.position.y = start_y
    goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
    goal.pose.orientation.w = math.cos(goal_yaw / 2.0)
    node.goal_pub.publish(goal)
    print(f'GOAL x={goal.pose.position.x:.3f} y={goal.pose.position.y:.3f}', flush=True)

    started = time.monotonic()
    next_report = started
    terminal = None
    while time.monotonic() - started < 25.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if node.pose is not None and now >= next_report:
            result = str((node.status or {}).get('result', 'NONE'))
            print(
                f'T={now-started:4.1f} '
                f'x={node.pose.position.x:+.3f} '
                f'y={node.pose.position.y:+.3f} result={result}',
                flush=True,
            )
            next_report = now + 1.0
        result = str((node.status or {}).get('result', '')).upper()
        if result in {'SUCCEEDED', 'ABORTED', 'FAILED', 'REJECTED'}:
            terminal = result
            break

    if node.cancel.service_is_ready():
        node.cancel.call_async(Trigger.Request())
        stop_until = time.monotonic() + 1.0
        while time.monotonic() < stop_until:
            rclpy.spin_once(node, timeout_sec=0.05)
    final_x = float(node.pose.position.x)
    final_y = float(node.pose.position.y)
    print(
        f'FINAL dx={final_x-start_x:+.3f} dy={final_y-start_y:+.3f} '
        f'terminal={terminal}',
        flush=True,
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
