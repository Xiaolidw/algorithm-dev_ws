#!/usr/bin/env python3
"""Small CLI client for the fixed manipulation acceptance action."""

import argparse

import rclpy
from moon_warehouse_interfaces.action import ExecuteManipulation
from rclpy.action import ActionClient
from rclpy.node import Node


class FixedTaskClient(Node):
    def __init__(self, operation, object_id):
        super().__init__('fixed_task_client')
        self._operation = operation
        self._object_id = object_id
        self._client = ActionClient(
            self, ExecuteManipulation, '/manipulation/execute')

    def run(self):
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('/manipulation/execute action is unavailable.')
        goal = ExecuteManipulation.Goal()
        goal.operation = self._operation
        goal.object_id = self._object_id
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError('Manipulation goal was rejected.')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        result = wrapped.result
        self.get_logger().info(
            f'success={result.success}, error_code={result.error_code}, '
            f'message={result.message}')
        return 0 if result.success else 1

    def _feedback(self, message):
        feedback = message.feedback
        self.get_logger().info(
            f'stage={feedback.stage}, progress={feedback.progress:.0%}')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'operation',
        choices=(
            'pick', 'place',
            'home', 'pick_pose', 'lift', 'place_pose', 'open', 'close',
        ),
    )
    parser.add_argument('object_id', nargs='?', default='red_cube_1')
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = FixedTaskClient(parsed.operation, parsed.object_id)
    try:
        exit_code = node.run()
    except (KeyboardInterrupt, RuntimeError) as error:
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
