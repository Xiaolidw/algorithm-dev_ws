#!/usr/bin/env python3
"""Fail-open execution gate for deterministic SIPP decisions."""

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
        self.declare_parameter('comfortable_deceleration', 2.50)
        self.declare_parameter('approach_angular_zero_distance', 0.30)
        self.declare_parameter('prepare_max_angular_speed', 1.30)
        self.decision_timeout = max(
            0.1, float(self.get_parameter('decision_timeout').value))
        self.deceleration = max(
            0.05, float(self.get_parameter('comfortable_deceleration').value))
        self.angular_zero_distance = max(
            0.0,
            float(self.get_parameter('approach_angular_zero_distance').value),
        )
        self.prepare_max_angular = max(
            0.05,
            float(self.get_parameter('prepare_max_angular_speed').value),
        )
        self.last_decision = None
        self.last_decision_received = 0.0
        self.create_subscription(
            String, '/sipp/decision', self.decision_callback, 10)
        self.create_subscription(
            Twist, '/cmd_vel_nav', self.velocity_callback, 20)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_sipp', 20)
        self.get_logger().info(
            'SIPP gate ready: /cmd_vel_nav -> /cmd_vel_sipp (fail-open)')

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

    @staticmethod
    def clamp_linear(output, limit):
        if limit >= 0.0 and abs(output.linear.x) > limit:
            output.linear.x = math.copysign(limit, output.linear.x)

    def velocity_callback(self, message):
        output = self.copy_velocity(message)
        decision = self.last_decision
        if (
            decision is None
            or time.monotonic() - self.last_decision_received > self.decision_timeout
            or bool(decision.get('shadow_mode', True))
        ):
            self.publisher.publish(output)
            return

        state = str(decision.get('state', 'DEGRADED'))
        speed_limit = float(decision.get('speed_limit', math.inf))
        self.clamp_linear(output, speed_limit)

        if state in {'STOP_COMMITTED', 'APPROACH_STOP_LINE'}:
            distance = max(0.0, float(decision.get('distance_to_stop', 0.0)))
            braking_limit = math.sqrt(2.0 * self.deceleration * distance)
            self.clamp_linear(output, braking_limit)
            if distance <= self.angular_zero_distance:
                output.angular.z = 0.0
        elif state == 'WAIT_AT_STOP_LINE':
            output = Twist()
        elif state == 'PREPARE_TO_PASS':
            output = Twist()
            angular = float(decision.get('prepare_angular_z', 0.0))
            output.angular.z = max(
                -self.prepare_max_angular,
                min(self.prepare_max_angular, angular),
            )
        # SAFETY_OVERRIDE deliberately passes MPPI's chosen command through.
        # Collision Monitor, downstream of the smoother, owns the hard stop;
        # forcing zero here can strand the robot inside a crossing path.
        # CRUISE, PASS_COMMITTED, DETECTED, SAFETY_OVERRIDE,
        # TOO_LATE_BYPASS, ENDPOINT_TURN and DEGRADED remain fail-open apart
        # from speed_limit.
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
