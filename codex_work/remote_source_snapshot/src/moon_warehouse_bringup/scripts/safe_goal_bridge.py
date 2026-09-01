#!/usr/bin/env python3
"""Validate RViz goal poses before forwarding them to Nav2."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class SafeGoalBridge(Node):
    """Turn a bounded /goal_pose request into a Nav2 action goal.

    RViz's standard SetGoal tool only publishes a pose. This node enforces
    a small, parameterized testing envelope before the pose reaches Nav2.
    """

    def __init__(self):
        super().__init__('safe_goal_bridge')
        self.declare_parameter('max_goal_distance_m', 3.0)
        self.declare_parameter('min_goal_distance_m', 0.15)
        self.declare_parameter('min_x_m', -20.0)
        self.declare_parameter('max_x_m', 20.0)
        self.declare_parameter('min_y_m', -15.0)
        self.declare_parameter('max_y_m', 15.0)

        self.current_pose = None
        self.goal_active = False
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/moon_warehouse/goal_guard/status', 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10)
        self.get_logger().info(
            'Safe goal bridge ready. RViz /goal_pose is limited to a '
            f'{self.get_parameter("max_goal_distance_m").value:.1f} m step.')

    def _amcl_callback(self, message):
        self.current_pose = message.pose.pose

    def _publish_status(self, message, level='info'):
        self.status_pub.publish(String(data=message))
        getattr(self.get_logger(), level)(message)

    def _reject(self, reason):
        self._publish_status(f'REJECTED: {reason}', 'warn')

    def _goal_callback(self, message):
        """Keep malformed or interrupted requests from killing the bridge."""
        try:
            self._handle_goal(message)
        except Exception as error:
            self.goal_active = False
            self._publish_status(
                f'FAILED: unexpected goal-processing error: {error}', 'error')

    def _handle_goal(self, message):
        if message.header.frame_id != 'map':
            self._reject(f'goal frame must be map, got {message.header.frame_id!r}')
            return
        if self.current_pose is None:
            self._reject('AMCL pose is unavailable; set/verify initial pose first')
            return
        if self.goal_active:
            self._reject('a navigation goal is still active; cancel or wait before sending another')
            return
        if not self.nav_client.server_is_ready():
            self._reject('Nav2 action server is not ready')
            return

        x = float(message.pose.position.x)
        y = float(message.pose.position.y)
        min_x = float(self.get_parameter('min_x_m').value)
        max_x = float(self.get_parameter('max_x_m').value)
        min_y = float(self.get_parameter('min_y_m').value)
        max_y = float(self.get_parameter('max_y_m').value)
        if not (min_x <= x <= max_x and min_y <= y <= max_y):
            self._reject(
                f'goal ({x:.2f}, {y:.2f}) is outside configured map bounds '
                f'[{min_x:.1f}, {max_x:.1f}] x [{min_y:.1f}, {max_y:.1f}]')
            return

        dx = x - float(self.current_pose.position.x)
        dy = y - float(self.current_pose.position.y)
        distance = math.hypot(dx, dy)
        min_distance = float(self.get_parameter('min_goal_distance_m').value)
        max_distance = float(self.get_parameter('max_goal_distance_m').value)
        if distance < min_distance:
            self._reject(f'goal is too close ({distance:.2f} m < {min_distance:.2f} m)')
            return
        if distance > max_distance:
            self._reject(
                f'goal is too far ({distance:.2f} m > {max_distance:.2f} m); '
                'send a shorter waypoint step')
            return

        goal = NavigateToPose.Goal()
        goal.pose = message
        self.goal_active = True
        try:
            future = self.nav_client.send_goal_async(goal)
        except Exception as error:
            self.goal_active = False
            self._reject(f'Nav2 goal request could not be sent: {error}')
            return
        future.add_done_callback(self._goal_response_callback)
        self._publish_status(f'FORWARDED: goal ({x:.2f}, {y:.2f}), distance={distance:.2f} m')

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.goal_active = False
                self._reject('Nav2 rejected the validated goal')
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._result_callback)
        except Exception as error:
            self.goal_active = False
            self._reject(f'Nav2 goal response handling failed: {error}')

    def _result_callback(self, future):
        self.goal_active = False
        try:
            status = future.result().status
        except Exception as error:
            self._publish_status(f'FAILED: Nav2 result error: {error}', 'error')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status('SUCCEEDED: validated navigation goal completed')
        elif status == GoalStatus.STATUS_CANCELED:
            self._publish_status('CANCELED: validated navigation goal canceled', 'warn')
        else:
            self._publish_status(f'FAILED: Nav2 completed with status={status}', 'warn')


def main():
    rclpy.init()
    node = SafeGoalBridge()
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
