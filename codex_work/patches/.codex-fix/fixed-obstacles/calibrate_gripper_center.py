#!/usr/bin/env python3
"""Numerically align link6's fingertip center to a stationary Gazebo cube."""

import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import GetEntityState
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
TARGET = np.array([0.0, 0.0, 0.045], dtype=float)


class Calibrator(Node):
    def __init__(self):
        super().__init__('gripper_center_calibrator')
        self.arm = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self.state = self.create_client(
            GetEntityState, '/gazebo/get_entity_state')

    def wait(self, future, timeout=8.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('ROS request timed out')
        return future.result()

    def move(self, q, duration=0.6):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in q]
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int(duration % 1 * 1_000_000_000))
        goal.trajectory.points = [point]
        handle = self.wait(self.arm.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Arm trajectory rejected')
        result = self.wait(handle.get_result_async(), timeout=duration + 5.0)
        if result.result.error_code != 0:
            raise RuntimeError(result.result.error_string)
        time.sleep(0.15)

    def relative(self, cube):
        request = GetEntityState.Request()
        request.name = cube
        request.reference_frame = 'six_arm::link6'
        response = self.wait(self.state.call_async(request))
        if not response.success:
            raise RuntimeError('Gazebo relative-pose query failed')
        p = response.state.pose.position
        return np.array([p.x, p.y, p.z], dtype=float)


def main():
    rclpy.init()
    node = Calibrator()
    cube = 'red_cube_1'
    q = np.array([0.0, 1.2, 1.17, 0.0, -0.3, 0.0], dtype=float)
    try:
        if not node.arm.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('arm controller unavailable')
        if not node.state.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Gazebo state service unavailable')
        node.move(q, 1.0)
        for iteration in range(5):
            base = node.relative(cube)
            error = TARGET - base
            print(f'ITER {iteration}: q={q.tolist()} relative={base.tolist()} '
                  f'error={error.tolist()} norm={np.linalg.norm(error):.6f}',
                  flush=True)
            if np.linalg.norm(error) < 0.008:
                break
            jacobian = np.zeros((3, 6), dtype=float)
            step = 0.035
            for joint in range(6):
                trial = q.copy()
                trial[joint] += step
                node.move(trial)
                jacobian[:, joint] = (node.relative(cube) - base) / step
                node.move(q)
            damping = 0.01
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(3), error)
            delta = np.clip(delta, -0.22, 0.22)
            q += delta
            # Respect the arm's declared joint limits with a small margin.
            lower = np.array([-3.0, -1.45, -1.45, -1.45, -2.30, -3.0])
            upper = np.array([3.0, 3.05, 1.45, 1.45, 2.30, 3.0])
            q = np.clip(q, lower, upper)
            node.move(q, 1.0)
        final = node.relative(cube)
        print(f'FINAL_Q={q.tolist()}', flush=True)
        print(f'FINAL_RELATIVE={final.tolist()}', flush=True)
        print(f'FINAL_ERROR={np.linalg.norm(TARGET-final):.6f}', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
