#!/usr/bin/env python3
"""Compact read-only monitor for mission, chassis and obstacle safety state."""

import json
import math
import sys
import time

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Monitor(Node):
    def __init__(self):
        super().__init__('competition_runtime_monitor')
        self.odom = None
        self.models = None
        self.velocity = None
        self.execution = {}
        self.navigation = {}
        self.sipp = {}
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.create_subscription(ModelStates, '/gazebo/model_states', self._models, 10)
        self.create_subscription(Twist, '/cmd_vel', self._velocity, 20)
        self.create_subscription(String, '/mission/execution_status', self._execution, 10)
        self.create_subscription(String, '/mission/navigation_status', self._navigation, 10)
        self.create_subscription(String, '/sipp/decision', self._sipp, 10)

    def _odom(self, message):
        self.odom = message

    def _models(self, message):
        self.models = message

    def _velocity(self, message):
        self.velocity = message

    @staticmethod
    def _decode(message):
        try:
            return json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return {}

    def _execution(self, message):
        self.execution = self._decode(message)

    def _navigation(self, message):
        self.navigation = self._decode(message)

    def _sipp(self, message):
        self.sipp = self._decode(message)

    def robot_truth(self):
        if self.models is None:
            return None, []
        try:
            index = self.models.name.index('six_arm')
        except ValueError:
            return None, []
        pose = self.models.pose[index]
        nearest = []
        for name, other in zip(self.models.name, self.models.pose):
            lower = name.lower()
            if name == 'six_arm' or not any(
                key in lower for key in ('cube', 'obstacle', 'stone', 'wall')
            ):
                continue
            distance = math.hypot(
                other.position.x - pose.position.x,
                other.position.y - pose.position.y,
            )
            nearest.append((distance, name))
        nearest.sort()
        return pose, nearest[:3]


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = Monitor()
    started = time.monotonic()
    next_report = started
    while time.monotonic() - started < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now < next_report:
            continue
        truth, nearest = node.robot_truth()
        if truth is None:
            position = 'truth=NA'
        else:
            position = (
                f'p=({truth.position.x:+.2f},{truth.position.y:+.2f},'
                f'{truth.position.z:+.3f})'
            )
        velocity = node.velocity or Twist()
        feedback = node.navigation.get('feedback', {})
        near_text = ','.join(f'{name}:{distance:.2f}' for distance, name in nearest)
        print(
            f'T={now-started:4.0f} {position} '
            f'v={velocity.linear.x:+.2f} w={velocity.angular.z:+.2f} '
            f'state={node.execution.get("state", "?")} '
            f'phase={node.execution.get("current_phase", "?")} '
            f'nav={node.navigation.get("result", "?")} '
            f'rem={feedback.get("distance_remaining_m", "?")} '
            f'sipp={node.sipp.get("state", "?")} '
            f'near=[{near_text}]',
            flush=True,
        )
        next_report = now + 2.0
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
