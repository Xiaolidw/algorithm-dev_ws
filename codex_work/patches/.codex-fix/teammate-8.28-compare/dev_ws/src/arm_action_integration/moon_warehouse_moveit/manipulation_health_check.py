#!/usr/bin/env python3
"""Check phase-1 manipulation graph dependencies and print one verdict."""

import time

import rclpy
from rclpy.action.graph import get_action_names_and_types
from rclpy.node import Node


class ManipulationHealthCheck(Node):
    """Read-only ROS graph checker used before real manipulation goals."""

    REQUIRED_ACTIONS = {
        '/manipulation/execute':
            'moon_warehouse_interfaces/action/ExecuteManipulation',
        '/arm_controller/follow_joint_trajectory':
            'control_msgs/action/FollowJointTrajectory',
        '/gripper_controller/follow_joint_trajectory':
            'control_msgs/action/FollowJointTrajectory',
    }
    REQUIRED_SERVICES = {
        '/ATTACHLINK': 'linkattacher_msgs/srv/AttachLink',
        '/DETACHLINK': 'linkattacher_msgs/srv/DetachLink',
    }
    REQUIRED_TOPICS = {
        '/clock': 'rosgraph_msgs/msg/Clock',
        '/joint_states': 'sensor_msgs/msg/JointState',
    }

    def __init__(self):
        super().__init__('manipulation_health_check')

    def wait_and_check(self, timeout_s=15.0):
        deadline = time.monotonic() + timeout_s
        missing = []
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            missing = self._missing_interfaces()
            if not missing or time.monotonic() >= deadline:
                break

        print('=== Phase-1 manipulation health check ===')
        if not missing:
            for label in (
                    *self.REQUIRED_ACTIONS,
                    *self.REQUIRED_SERVICES,
                    *self.REQUIRED_TOPICS):
                print(f'PASS  {label}')
            print('RESULT: PASS - manipulation interfaces are ready for dry-run/real acceptance.')
            return 0

        for description in missing:
            print(f'FAIL  {description}')
        print('RESULT: FAIL - do not send a real manipulation goal.')
        return 1

    def _missing_interfaces(self):
        actions = {
            name: types for name, types in get_action_names_and_types(self)}
        services = {
            name: types for name, types in self.get_service_names_and_types()}
        topics = {
            name: types for name, types in self.get_topic_names_and_types()}
        missing = []
        self._compare('action', self.REQUIRED_ACTIONS, actions, missing)
        self._compare('service', self.REQUIRED_SERVICES, services, missing)
        self._compare('topic', self.REQUIRED_TOPICS, topics, missing)
        return missing

    @staticmethod
    def _compare(kind, required, actual, missing):
        for name, expected_type in required.items():
            actual_types = actual.get(name, [])
            if not actual_types:
                missing.append(f'{kind} {name} is missing')
            elif expected_type not in actual_types:
                missing.append(
                    f'{kind} {name} type mismatch: expected {expected_type}, '
                    f'got {actual_types}')


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationHealthCheck()
    try:
        exit_code = node.wait_and_check()
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
