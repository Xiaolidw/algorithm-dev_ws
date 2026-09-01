#!/usr/bin/env python3
"""Publish one selectable map pose through the mission coordinator."""

import argparse
import math
import sys
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class MissionGoalPublisher(Node):
    """One-shot publisher for the coordinator's formal navigation input."""

    def __init__(self):
        super().__init__('mission_navigation_goal_sender')
        self.publisher = self.create_publisher(
            PoseStamped,
            '/mission/navigation_goal',
            10,
        )

    def wait_for_coordinator(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.publisher.get_subscription_count() > 0

    def publish_goal(self, x, y, yaw):
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        self.publisher.publish(message)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            'Send an arbitrary map-frame goal through '
            '/mission/navigation_goal -> coordinator -> Nav2.'
        ),
    )
    parser.add_argument('x', type=float, help='Goal X in the map frame (m)')
    parser.add_argument('y', type=float, help='Goal Y in the map frame (m)')
    parser.add_argument(
        'yaw',
        type=float,
        nargs='?',
        default=0.0,
        help='Goal heading in radians (default: 0.0)',
    )
    parser.add_argument(
        '--wait-timeout',
        type=float,
        default=5.0,
        help='Seconds to wait for the coordinator subscriber',
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw)):
        print('ERROR: x, y and yaw must be finite numbers.', file=sys.stderr)
        return 2
    if args.wait_timeout <= 0.0:
        print('ERROR: --wait-timeout must be positive.', file=sys.stderr)
        return 2

    rclpy.init()
    node = MissionGoalPublisher()
    try:
        if not node.wait_for_coordinator(args.wait_timeout):
            node.get_logger().error(
                'No subscriber on /mission/navigation_goal. '
                'Start mission_system.launch.py first.'
            )
            return 3
        node.publish_goal(args.x, args.y, args.yaw)
        rclpy.spin_once(node, timeout_sec=0.2)
        node.get_logger().info(
            f'Published mission goal: x={args.x:.3f}, '
            f'y={args.y:.3f}, yaw={args.yaw:.3f}'
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
