#!/usr/bin/env python3
"""Execute configuration-driven fixed pick/place sequences."""

import re
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from linkattacher_msgs.srv import AttachLink, DetachLink
from moon_warehouse_interfaces.action import ExecuteManipulation
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ManipulationError(RuntimeError):
    """Expected manipulation failure carrying a stable error code."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class FixedManipulationServer(Node):
    """Action server for the phase-1 known-coordinate manipulation test."""

    SUCCESS = 0
    INVALID_GOAL = 1
    DEPENDENCY_UNAVAILABLE = 2
    ARM_FAILED = 3
    GRIPPER_FAILED = 4
    ATTACH_FAILED = 5
    CANCELLED = 6
    INTERNAL_ERROR = 99
    SUPPORTED_OPERATIONS = (
        'pick', 'place',
        'home', 'pick_pose', 'lift', 'place_pose', 'open', 'close', 'move_arm',
    )

    def __init__(self):
        super().__init__('fixed_manipulation_server')
        self._callback_group = ReentrantCallbackGroup()
        self._busy = False
        self._declare_parameters()
        self._load_parameters()

        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            self._arm_action_name,
            callback_group=self._callback_group,
        )
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            self._gripper_action_name,
            callback_group=self._callback_group,
        )
        self._attach_client = self.create_client(
            AttachLink, self._attach_service_name,
            callback_group=self._callback_group)
        self._detach_client = self.create_client(
            DetachLink, self._detach_service_name,
            callback_group=self._callback_group)

        self._server = ActionServer(
            self,
            ExecuteManipulation,
            self._action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f'Fixed manipulation server ready on {self._action_name}; '
            f'dry_run={self._dry_run}.')

    def _declare_parameters(self):
        defaults = {
            'action_name': '/manipulation/execute',
            'arm_action_name': '/arm_controller/follow_joint_trajectory',
            'gripper_action_name': '/gripper_controller/follow_joint_trajectory',
            'attach_service_name': '/ATTACHLINK',
            'detach_service_name': '/DETACHLINK',
            'dependency_timeout_s': 10.0,
            'robot_model': 'six_arm',
            'tool_link': 'link6',
            'default_object': 'red_cube_1',
            'object_link': 'link',
            'arm_joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            'gripper_joints': ['finger_joint1'],
            'arm_home': [0.0, 0.6102, 1.2593, 0.0, -1.4931, 0.0],
            'arm_pick': [0.0, 1.2, 1.17, 0.0, -0.3, 0.0],
            'arm_lift': [0.0, 0.6102, 1.2593, 0.0, -1.4931, 0.0],
            'arm_place': [0.0, 1.2, 1.17, 0.0, -0.3, 0.0],
            'gripper_open': [0.04],
            'gripper_closed': [0.0],
            'arm_motion_duration_s': 3.0,
            'gripper_motion_duration_s': 1.5,
            'settle_time_s': 0.5,
            'dry_run': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self):
        value = lambda name: self.get_parameter(name).value
        self._action_name = str(value('action_name'))
        self._arm_action_name = str(value('arm_action_name'))
        self._gripper_action_name = str(value('gripper_action_name'))
        self._attach_service_name = str(value('attach_service_name'))
        self._detach_service_name = str(value('detach_service_name'))
        self._dependency_timeout = float(value('dependency_timeout_s'))
        self._robot_model = str(value('robot_model'))
        self._tool_link = str(value('tool_link'))
        self._default_object = str(value('default_object'))
        self._object_link = str(value('object_link'))
        self._arm_joints = list(value('arm_joints'))
        self._gripper_joints = list(value('gripper_joints'))
        self._arm_home = list(value('arm_home'))
        self._arm_pick = list(value('arm_pick'))
        self._arm_lift = list(value('arm_lift'))
        self._arm_place = list(value('arm_place'))
        self._gripper_open = list(value('gripper_open'))
        self._gripper_closed = list(value('gripper_closed'))
        self._arm_duration = float(value('arm_motion_duration_s'))
        self._gripper_duration = float(value('gripper_motion_duration_s'))
        self._settle_time = float(value('settle_time_s'))
        self._dry_run = bool(value('dry_run'))
        self._validate_configuration()

    def _validate_configuration(self):
        vectors = (
            ('arm_home', self._arm_home, self._arm_joints),
            ('arm_pick', self._arm_pick, self._arm_joints),
            ('arm_lift', self._arm_lift, self._arm_joints),
            ('arm_place', self._arm_place, self._arm_joints),
            ('gripper_open', self._gripper_open, self._gripper_joints),
            ('gripper_closed', self._gripper_closed, self._gripper_joints),
        )
        for name, positions, joints in vectors:
            if len(positions) != len(joints):
                raise ValueError(
                    f'{name} has {len(positions)} values but requires {len(joints)}.')
        if self._arm_duration <= 0.0 or self._gripper_duration <= 0.0:
            raise ValueError('Trajectory durations must be positive.')

    def _goal_callback(self, goal):
        operation = goal.operation.strip().lower()
        object_id = goal.object_id.strip() or self._default_object
        if self._busy:
            self.get_logger().warning('Rejecting manipulation goal: server is busy.')
            return GoalResponse.REJECT
        if operation not in self.SUPPORTED_OPERATIONS:
            self.get_logger().warning(f'Rejecting unsupported operation: {operation}.')
            return GoalResponse.REJECT
        if not re.fullmatch(r'(red|blue)_cube_[1-9][0-9]*', object_id):
            self.get_logger().warning(f'Rejecting invalid object id: {object_id}.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        self._busy = True
        operation = goal_handle.request.operation.strip().lower()
        object_id = goal_handle.request.object_id.strip() or self._default_object
        try:
            if self._dry_run:
                await self._execute_dry_run(goal_handle, operation, object_id)
            else:
                self._wait_for_dependencies(operation)
                await self._execute_operation(goal_handle, operation, object_id)
            goal_handle.succeed()
            return self._result(True, self.SUCCESS, f'{operation} completed for {object_id}.')
        except ManipulationError as error:
            if error.code == self.CANCELLED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return self._result(False, error.code, str(error))
        except Exception as error:  # Defensive boundary around hardware callbacks.
            self.get_logger().error(f'Unexpected manipulation failure: {error}')
            goal_handle.abort()
            return self._result(False, self.INTERNAL_ERROR, str(error))
        finally:
            self._busy = False

    async def _execute_operation(self, handle, operation, object_id):
        if operation == 'pick':
            await self._pick(handle, object_id)
        elif operation == 'place':
            await self._place(handle, object_id)
        elif operation == 'home':
            await self._move_arm_once(handle, 'return_home', self._arm_home)
        elif operation == 'pick_pose':
            await self._move_arm_once(handle, 'move_to_pick_pose', self._arm_pick)
        elif operation == 'lift':
            await self._move_arm_once(handle, 'lift_object', self._arm_lift)
        elif operation == 'place_pose':
            await self._move_arm_once(handle, 'move_to_place_pose', self._arm_place)
        elif operation == 'open':
            await self._move_gripper_once(
                handle, 'open_gripper', self._gripper_open)
        elif operation == 'close':
            await self._move_gripper_once(
                handle, 'close_gripper', self._gripper_closed)

    async def _move_arm_once(self, handle, stage, positions):
        await self._stage(handle, stage, 0.25)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, positions,
            self._arm_duration, self.ARM_FAILED, stage.replace('_', ' '))
        await self._stage(handle, f'{stage}_complete', 1.0)

    async def _move_gripper_once(self, handle, stage, positions):
        await self._stage(handle, stage, 0.25)
        await self._send_trajectory(
            self._gripper_client, self._gripper_joints, positions,
            self._gripper_duration, self.GRIPPER_FAILED,
            stage.replace('_', ' '))
        await self._stage(handle, f'{stage}_complete', 1.0)

    def _wait_for_dependencies(self, operation):
        checks = []
        if operation in ('pick', 'place', 'home', 'pick_pose', 'lift', 'place_pose'):
            checks.append((
                self._arm_action_name,
                self._arm_client.wait_for_server(
                    timeout_sec=self._dependency_timeout)))
        if operation in ('pick', 'place', 'open', 'close'):
            checks.append((
                self._gripper_action_name,
                self._gripper_client.wait_for_server(
                    timeout_sec=self._dependency_timeout)))
        if operation in ('pick', 'place'):
            checks.extend((
                (self._attach_service_name,
                 self._attach_client.wait_for_service(
                     timeout_sec=self._dependency_timeout)),
                (self._detach_service_name,
                 self._detach_client.wait_for_service(
                     timeout_sec=self._dependency_timeout)),
            ))
        unavailable = [name for name, ready in checks if not ready]
        if unavailable:
            raise ManipulationError(
                self.DEPENDENCY_UNAVAILABLE,
                f'Required interfaces are unavailable: {unavailable}.')

    async def _pick(self, handle, object_id):
        await self._stage(handle, 'open_gripper', 0.10)
        await self._send_trajectory(
            self._gripper_client, self._gripper_joints, self._gripper_open,
            self._gripper_duration, self.GRIPPER_FAILED, 'open gripper')
        await self._stage(handle, 'move_to_pick_pose', 0.30)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, self._arm_pick,
            self._arm_duration, self.ARM_FAILED, 'move arm to pick pose')
        await self._stage(handle, 'close_gripper', 0.50)
        await self._send_trajectory(
            self._gripper_client, self._gripper_joints, self._gripper_closed,
            self._gripper_duration, self.GRIPPER_FAILED, 'close gripper')
        await self._stage(handle, 'attach_object', 0.70)
        await self._call_attach(object_id, attach=True)
        await self._stage(handle, 'lift_object', 0.85)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, self._arm_lift,
            self._arm_duration, self.ARM_FAILED, 'lift arm')
        await self._stage(handle, 'pick_complete', 1.0)

    async def _place(self, handle, object_id):
        await self._stage(handle, 'move_to_place_pose', 0.20)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, self._arm_place,
            self._arm_duration, self.ARM_FAILED, 'move arm to place pose')
        await self._stage(handle, 'open_gripper', 0.50)
        await self._send_trajectory(
            self._gripper_client, self._gripper_joints, self._gripper_open,
            self._gripper_duration, self.GRIPPER_FAILED, 'open gripper')
        await self._stage(handle, 'detach_object', 0.70)
        await self._call_attach(object_id, attach=False)
        await self._stage(handle, 'return_home', 0.85)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, self._arm_home,
            self._arm_duration, self.ARM_FAILED, 'return arm home')
        await self._stage(handle, 'place_complete', 1.0)

    async def _execute_dry_run(self, handle, operation, object_id):
        stages = (
            ('validate_goal', 0.10),
            (f'{operation}_{object_id}', 0.50),
            (f'{operation}_complete', 1.0),
        )
        for stage, progress in stages:
            await self._stage(handle, stage, progress)

    async def _stage(self, handle, name, progress):
        if handle.is_cancel_requested:
            raise ManipulationError(self.CANCELLED, f'Cancelled during {name}.')
        feedback = ExecuteManipulation.Feedback()
        feedback.stage = name
        feedback.progress = float(progress)
        handle.publish_feedback(feedback)
        self.get_logger().info(f'Stage: {name} ({progress:.0%})')
        # rclpy coroutines are driven by its executor rather than an asyncio
        # event loop.  Sleeping in this executor thread is therefore more
        # portable on ROS 2 Humble; the remaining executor threads continue
        # serving cancellation and controller/service responses.
        time.sleep(0.05 if self._dry_run else self._settle_time)

    async def _send_trajectory(
            self, client, joints, positions, duration_s, error_code, description):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = self._duration(duration_s)
        goal.trajectory.points = [point]

        goal_handle = await client.send_goal_async(goal)
        if not goal_handle.accepted:
            raise ManipulationError(error_code, f'Controller rejected: {description}.')
        wrapped_result = await goal_handle.get_result_async()
        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise ManipulationError(
                error_code,
                f'Controller failed: {description}; error_code={result.error_code}.')

    async def _call_attach(self, object_id, attach):
        request_type = AttachLink.Request if attach else DetachLink.Request
        request = request_type()
        request.model1_name = self._robot_model
        request.link1_name = self._tool_link
        request.model2_name = object_id
        request.link2_name = self._object_link
        client = self._attach_client if attach else self._detach_client
        response = await client.call_async(request)
        if not response.success:
            verb = 'attach' if attach else 'detach'
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Failed to {verb} {object_id}: {response.message}')

    @staticmethod
    def _duration(seconds):
        whole = int(seconds)
        return Duration(sec=whole, nanosec=int((seconds - whole) * 1_000_000_000))

    @staticmethod
    def _result(success, error_code, message):
        result = ExecuteManipulation.Result()
        result.success = success
        result.error_code = error_code
        result.message = message
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FixedManipulationServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
