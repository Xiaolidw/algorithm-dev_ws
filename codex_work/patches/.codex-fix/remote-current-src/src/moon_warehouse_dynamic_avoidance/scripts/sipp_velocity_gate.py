#!/usr/bin/env python3
"""Fail-open SIPP velocity gate placed before Nav2 velocity_smoother."""
import json
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SippVelocityGate(Node):
    def __init__(self):
        super().__init__('sipp_velocity_gate')
        self.declare_parameter('decision_timeout', 0.5)
        self.declare_parameter('comfortable_deceleration', 0.60)
        self.decision_timeout = max(
            0.1, float(self.get_parameter('decision_timeout').value))
        self.deceleration = max(
            0.05,
            float(self.get_parameter('comfortable_deceleration').value),
        )
        self.last_decision = None
        self.last_decision_received = 0.0
        self.create_subscription(
            String, '/sipp/decision', self.decision_callback, 10)
        self.create_subscription(
            Twist, '/cmd_vel_nav', self.velocity_callback, 20)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_sipp', 20)
        self.get_logger().info(
            'SIPP velocity gate ready: fail-open, input=/cmd_vel_nav, '
            'output=/cmd_vel_sipp')

    def decision_callback(self, message):
        try:
            self.last_decision = json.loads(message.data)
            self.last_decision_received = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError):
            self.last_decision = None

    @staticmethod
    def copy_velocity(message):
        output = Twist()
        output.linear.x = message.linear.x
        output.linear.y = message.linear.y
        output.linear.z = message.linear.z
        output.angular.x = message.angular.x
        output.angular.y = message.angular.y
        output.angular.z = message.angular.z
        return output

    def velocity_callback(self, message):
        output = self.copy_velocity(message)
        if (
            self.last_decision is None
            or time.monotonic() - self.last_decision_received
            > self.decision_timeout
        ):
            self.publisher.publish(output)
            return

        state = self.last_decision.get('state', 'DEGRADED')
        if bool(self.last_decision.get('shadow_mode', True)):
            self.publisher.publish(output)
            return
        if state == 'WAIT_AT_STOP_LINE':
            output = Twist()
        elif state == 'APPROACH_STOP_LINE':
            distance = max(
                0.0,
                float(self.last_decision.get('distance_to_stop', 0.0)),
            )
            speed_limit = math.sqrt(2.0 * self.deceleration * distance)
            if abs(output.linear.x) > speed_limit:
                output.linear.x = math.copysign(speed_limit, output.linear.x)
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = SippVelocityGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
