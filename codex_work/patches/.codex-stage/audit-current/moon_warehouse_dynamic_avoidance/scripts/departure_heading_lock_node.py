#!/usr/bin/env python3
"""Lock the departure heading once for each navigation goal."""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    siny = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cosy = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(siny, cosy)


class DepartureHeadingLock(Node):
    def __init__(self):
        super().__init__('departure_heading_lock')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('goal_change_distance', 0.25)
        self.declare_parameter('heading_lookahead', 0.65)
        self.declare_parameter('heading_lock_entry_angle', 0.60)
        self.declare_parameter('heading_kp', 1.5)
        self.declare_parameter('heading_max_angular_speed', 0.9)
        self.declare_parameter('heading_min_angular_speed', 0.25)
        self.declare_parameter('heading_exit_tolerance', 0.10)
        self.declare_parameter('heading_settle_time', 0.20)
        self.declare_parameter('tf_timeout', 0.05)

        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('robot_base_frame').value)
        self.goal_change_distance = float(
            self.get_parameter('goal_change_distance').value)
        self.lookahead = float(self.get_parameter('heading_lookahead').value)
        self.entry_angle = float(
            self.get_parameter('heading_lock_entry_angle').value)
        self.kp = float(self.get_parameter('heading_kp').value)
        self.max_angular = float(
            self.get_parameter('heading_max_angular_speed').value)
        self.min_angular = float(
            self.get_parameter('heading_min_angular_speed').value)
        self.exit_tolerance = float(
            self.get_parameter('heading_exit_tolerance').value)
        self.settle_time = float(
            self.get_parameter('heading_settle_time').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.plan = None
        self.goal_position = None
        self.locked_yaw = None
        self.alignment_complete = True
        self.settle_since = None
        self.state = 'PASSTHROUGH'

        self.output_publisher = self.create_publisher(
            Twist, '/cmd_vel_nav', 20)
        self.state_publisher = self.create_publisher(
            String, '/departure_heading/state', 10)
        self.target_publisher = self.create_publisher(
            Float32, '/departure_heading/target', 10)
        self.error_publisher = self.create_publisher(
            Float32, '/departure_heading/error', 10)
        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.create_subscription(
            Twist, '/cmd_vel_nav_raw', self.velocity_callback, 20)
        self.get_logger().info(
            'Departure heading lock ready: '
            '/cmd_vel_nav_raw -> /cmd_vel_nav')

    def robot_yaw_and_position(self):
        transform = self.tf_buffer.lookup_transform(
            self.global_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=self.tf_timeout),
        )
        return (
            yaw_from_quaternion(transform.transform.rotation),
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def plan_callback(self, message):
        if len(message.poses) < 2:
            return
        self.plan = message
        endpoint = message.poses[-1].pose.position
        new_goal = self.goal_position is None or math.hypot(
            endpoint.x - self.goal_position[0],
            endpoint.y - self.goal_position[1],
        ) > self.goal_change_distance
        if new_goal:
            self.goal_position = (float(endpoint.x), float(endpoint.y))
            self.locked_yaw = None
            self.alignment_complete = False
            self.settle_since = None
            self.state = 'PENDING_PATH_HEADING'
        if self.locked_yaw is None:
            self.try_lock_heading()

    def try_lock_heading(self):
        if self.plan is None or len(self.plan.poses) < 2:
            return False
        try:
            robot_yaw, robot_x, robot_y = self.robot_yaw_and_position()
        except TransformException:
            return False
        poses = self.plan.poses
        nearest = min(
            range(len(poses)),
            key=lambda index: math.hypot(
                poses[index].pose.position.x - robot_x,
                poses[index].pose.position.y - robot_y,
            ),
        )
        target = nearest
        travelled = 0.0
        while target + 1 < len(poses) and travelled < self.lookahead:
            first = poses[target].pose.position
            second = poses[target + 1].pose.position
            travelled += math.hypot(second.x - first.x, second.y - first.y)
            target += 1
        start = poses[nearest].pose.position
        finish = poses[target].pose.position
        if math.hypot(finish.x - start.x, finish.y - start.y) < 0.05:
            return False
        self.locked_yaw = math.atan2(finish.y - start.y, finish.x - start.x)
        error = normalize_angle(self.locked_yaw - robot_yaw)
        if abs(error) <= self.entry_angle:
            self.alignment_complete = True
            self.state = 'ALIGNED'
        else:
            self.state = 'ALIGNING'
        self.publish_diagnostics(error)
        return True

    def publish_diagnostics(self, error):
        self.state_publisher.publish(String(data=self.state))
        self.target_publisher.publish(Float32(
            data=float(self.locked_yaw if self.locked_yaw is not None else math.nan)))
        self.error_publisher.publish(Float32(data=float(error)))

    @staticmethod
    def copy_twist(message):
        output = Twist()
        output.linear.x = message.linear.x
        output.linear.y = message.linear.y
        output.linear.z = message.linear.z
        output.angular.x = message.angular.x
        output.angular.y = message.angular.y
        output.angular.z = message.angular.z
        return output

    def velocity_callback(self, message):
        if self.alignment_complete:
            self.output_publisher.publish(self.copy_twist(message))
            self.publish_diagnostics(0.0)
            return
        if self.locked_yaw is None and not self.try_lock_heading():
            self.state = 'DEGRADED_PASSTHROUGH'
            self.output_publisher.publish(self.copy_twist(message))
            self.publish_diagnostics(math.nan)
            return
        try:
            robot_yaw, _, _ = self.robot_yaw_and_position()
        except TransformException:
            self.state = 'DEGRADED_PASSTHROUGH'
            self.output_publisher.publish(self.copy_twist(message))
            self.publish_diagnostics(math.nan)
            return
        error = normalize_angle(self.locked_yaw - robot_yaw)
        output = Twist()
        if abs(error) <= self.exit_tolerance:
            if self.settle_since is None:
                self.settle_since = time.monotonic()
            if time.monotonic() - self.settle_since >= self.settle_time:
                self.alignment_complete = True
                self.state = 'ALIGNED'
                output = self.copy_twist(message)
            else:
                self.state = 'SETTLING'
        else:
            self.settle_since = None
            self.state = 'ALIGNING'
            angular = min(self.max_angular, self.kp * abs(error))
            angular = max(self.min_angular, angular)
            output.angular.z = math.copysign(angular, error)
        self.output_publisher.publish(output)
        self.publish_diagnostics(error)


def main(args=None):
    rclpy.init(args=args)
    node = DepartureHeadingLock()
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
