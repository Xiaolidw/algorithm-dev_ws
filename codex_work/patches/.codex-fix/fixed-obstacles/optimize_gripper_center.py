#!/usr/bin/env python3
"""Find a bounded arm pose that puts a stationary cube between both fingers."""

import time

import numpy as np
from scipy.optimize import least_squares
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import GetEntityState
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
TARGET = np.array([0.0, 0.0, 0.045], dtype=float)
LOWER = np.array([-3.0, -2.30, -2.57, -3.0, -2.30, -3.0])
UPPER = np.array([3.0, 2.30, 2.57, 3.0, 2.30, 3.0])


class Optimizer(Node):
    def __init__(self):
        super().__init__('gripper_center_optimizer')
        self.arm = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self.state = self.create_client(
            GetEntityState, '/gazebo/get_entity_state')
        self.best_q = None
        self.best_relative = None
        self.best_norm = float('inf')
        self.evaluations = 0

    def wait(self, future, timeout=8.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('ROS request timed out')
        return future.result()

    def move(self, q, duration=0.42):
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
        wrapped = self.wait(handle.get_result_async(), timeout=duration + 5.0)
        if wrapped.result.error_code != 0:
            raise RuntimeError(wrapped.result.error_string)
        time.sleep(0.10)

    def relative(self):
        request = GetEntityState.Request()
        request.name = 'red_cube_1'
        request.reference_frame = 'six_arm::link6'
        response = self.wait(self.state.call_async(request))
        if not response.success:
            raise RuntimeError('Gazebo relative-pose query failed')
        p = response.state.pose.position
        return np.array([p.x, p.y, p.z], dtype=float)

    def residual(self, q):
        self.move(q)
        relative = self.relative()
        error = relative - TARGET
        norm = float(np.linalg.norm(error))
        self.evaluations += 1
        if norm < self.best_norm:
            self.best_norm = norm
            self.best_q = np.array(q, dtype=float)
            self.best_relative = relative.copy()
        print(
            f'EVAL {self.evaluations:02d}: norm={norm:.6f} '
            f'relative={relative.tolist()} q={np.asarray(q).tolist()}',
            flush=True,
        )
        return error


def main():
    rclpy.init()
    node = Optimizer()
    # Best stable sample from the coarse search, before its final divergent step.
    start = np.array([
        -0.2169075027, 1.2467101986, 1.1483727166,
        0.2695638229, 0.0826044097, 0.3701916358,
    ])
    try:
        if not node.arm.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('arm controller unavailable')
        if not node.state.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Gazebo state service unavailable')
        result = least_squares(
            node.residual,
            start,
            bounds=(LOWER, UPPER),
            method='trf',
            diff_step=0.025,
            x_scale='jac',
            max_nfev=45,
            ftol=1e-5,
            xtol=1e-5,
            gtol=1e-5,
            verbose=1,
        )
        if node.best_q is None:
            raise RuntimeError('optimizer produced no valid sample')
        node.move(node.best_q, duration=1.0)
        final = node.relative()
        print(f'OPT_STATUS={result.status}:{result.message}', flush=True)
        print(f'BEST_Q={node.best_q.tolist()}', flush=True)
        print(f'BEST_RELATIVE={final.tolist()}', flush=True)
        print(f'BEST_ERROR={np.linalg.norm(final-TARGET):.6f}', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
