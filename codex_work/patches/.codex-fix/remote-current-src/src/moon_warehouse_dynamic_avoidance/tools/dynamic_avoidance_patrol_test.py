#!/usr/bin/env python3

import argparse
import math
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class DynamicAvoidancePatrolTest(Node):

    def __init__(self, arguments):
        super().__init__(
            'dynamic_avoidance_patrol_test'
        )

        self.arguments = arguments

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )

        self.last_feedback_print_time = 0.0

    @staticmethod
    def yaw_to_quaternion(yaw):
        return (
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )

    def create_goal(
        self,
        x,
        y,
        yaw,
    ):
        goal = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        quaternion_z, quaternion_w = (
            self.yaw_to_quaternion(yaw)
        )

        pose.pose.orientation.z = quaternion_z
        pose.pose.orientation.w = quaternion_w

        goal.pose = pose

        return goal

    def feedback_callback(
        self,
        feedback_message,
    ):
        current_time = time.monotonic()

        if (
            current_time
            - self.last_feedback_print_time
            < 1.0
        ):
            return

        self.last_feedback_print_time = (
            current_time
        )

        feedback = feedback_message.feedback

        self.get_logger().info(
            'Navigation feedback: '
            f'distance_remaining='
            f'{feedback.distance_remaining:.2f} m, '
            f'recoveries='
            f'{feedback.number_of_recoveries}'
        )

    def navigate_to(
        self,
        point_name,
        x,
        y,
        yaw,
    ):
        self.get_logger().info(
            f'Waiting for /navigate_to_pose...'
        )

        if not self.action_client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                '/navigate_to_pose is unavailable.'
            )

            return False, 'SERVER_UNAVAILABLE'

        goal = self.create_goal(
            x,
            y,
            yaw,
        )

        self.get_logger().info(
            f'Sending goal {point_name}: '
            f'x={x:.2f}, y={y:.2f}, '
            f'yaw={yaw:.2f}'
        )

        send_future = (
            self.action_client.send_goal_async(
                goal,
                feedback_callback=(
                    self.feedback_callback
                ),
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=10.0,
        )

        if not send_future.done():
            self.get_logger().error(
                'Timed out while sending goal.'
            )

            return False, 'SEND_TIMEOUT'

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                'Goal handle is None.'
            )

            return False, 'INVALID_GOAL_HANDLE'

        if not goal_handle.accepted:
            self.get_logger().error(
                f'Goal {point_name} was rejected.'
            )

            return False, 'REJECTED'

        self.get_logger().info(
            f'Goal {point_name} accepted.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=(
                self.arguments.goal_timeout
            ),
        )

        if not result_future.done():
            self.get_logger().error(
                f'Goal {point_name} timed out; '
                'requesting cancellation.'
            )

            cancel_future = (
                goal_handle.cancel_goal_async()
            )

            rclpy.spin_until_future_complete(
                self,
                cancel_future,
                timeout_sec=5.0,
            )

            return False, 'TIMEOUT'

        wrapped_result = result_future.result()

        if wrapped_result is None:
            return False, 'NO_RESULT'

        status = wrapped_result.status

        status_name = STATUS_NAMES.get(
            status,
            f'UNRECOGNIZED_{status}',
        )

        succeeded = (
            status
            == GoalStatus.STATUS_SUCCEEDED
        )

        if succeeded:
            self.get_logger().info(
                f'Goal {point_name} succeeded.'
            )
        else:
            self.get_logger().error(
                f'Goal {point_name} finished '
                f'with status {status_name}.'
            )

        return succeeded, status_name

    def run(self):
        point_a = (
            self.arguments.point_a_x,
            self.arguments.point_a_y,
        )

        point_b = (
            self.arguments.point_b_x,
            self.arguments.point_b_y,
        )

        yaw_a_to_b = math.atan2(
            point_b[1] - point_a[1],
            point_b[0] - point_a[0],
        )

        yaw_b_to_a = math.atan2(
            point_a[1] - point_b[1],
            point_a[0] - point_b[0],
        )

        test_results = []

        self.get_logger().info(
            'Starting dynamic-avoidance patrol test.'
        )

        self.get_logger().info(
            f'Point A: {point_a}'
        )

        self.get_logger().info(
            f'Point B: {point_b}'
        )

        self.get_logger().info(
            f'Cycles: {self.arguments.cycles}'
        )

        for cycle_index in range(
            1,
            self.arguments.cycles + 1,
        ):
            self.get_logger().info(
                '================================'
            )

            self.get_logger().info(
                f'Cycle {cycle_index}/'
                f'{self.arguments.cycles}'
            )

            succeeded_b, status_b = (
                self.navigate_to(
                    'B',
                    point_b[0],
                    point_b[1],
                    yaw_a_to_b,
                )
            )

            test_results.append({
                'cycle': cycle_index,
                'target': 'B',
                'succeeded': succeeded_b,
                'status': status_b,
            })

            if not succeeded_b:
                if (
                    not self.arguments
                    .continue_on_failure
                ):
                    break

            time.sleep(
                self.arguments.pause_seconds
            )

            succeeded_a, status_a = (
                self.navigate_to(
                    'A',
                    point_a[0],
                    point_a[1],
                    yaw_b_to_a,
                )
            )

            test_results.append({
                'cycle': cycle_index,
                'target': 'A',
                'succeeded': succeeded_a,
                'status': status_a,
            })

            if not succeeded_a:
                if (
                    not self.arguments
                    .continue_on_failure
                ):
                    break

            time.sleep(
                self.arguments.pause_seconds
            )

        successful_goals = sum(
            1
            for result in test_results
            if result['succeeded']
        )

        total_goals = len(test_results)

        success_rate = (
            successful_goals / total_goals
            if total_goals
            else 0.0
        )

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            'Patrol test completed.'
        )

        self.get_logger().info(
            f'Successful goals: '
            f'{successful_goals}/{total_goals}'
        )

        self.get_logger().info(
            f'Success rate: '
            f'{success_rate * 100.0:.1f}%'
        )

        for result in test_results:
            self.get_logger().info(
                f'Cycle {result["cycle"]}, '
                f'target {result["target"]}: '
                f'{result["status"]}'
            )

        return (
            total_goals > 0
            and successful_goals == total_goals
        )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            'Repeatedly navigate across a dynamic '
            'obstacle route.'
        )
    )

    parser.add_argument(
        '--point-a-x',
        type=float,
        default=0.0,
    )

    parser.add_argument(
        '--point-a-y',
        type=float,
        default=1.3,
    )

    parser.add_argument(
        '--point-b-x',
        type=float,
        default=0.0,
    )

    parser.add_argument(
        '--point-b-y',
        type=float,
        default=4.3,
    )

    parser.add_argument(
        '--cycles',
        type=int,
        default=5,
    )

    parser.add_argument(
        '--pause-seconds',
        type=float,
        default=1.0,
    )

    parser.add_argument(
        '--goal-timeout',
        type=float,
        default=90.0,
    )

    parser.add_argument(
        '--continue-on-failure',
        action='store_true',
    )

    return parser.parse_args()


def main():
    arguments = parse_arguments()

    if arguments.cycles <= 0:
        raise ValueError(
            'cycles must be greater than zero.'
        )

    rclpy.init()

    node = DynamicAvoidancePatrolTest(
        arguments
    )

    try:
        passed = node.run()

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Test interrupted by user.'
        )

        passed = False

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
