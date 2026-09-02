#!/usr/bin/env python3
"""Execute configuration-driven fixed pick/place sequences."""

import math
import re
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import GetEntityState, SetEntityState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
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
    GRASP_WINDOW_FAILED = 7
    RELEASE_FAILED = 8
    INTERNAL_ERROR = 99
    SUPPORTED_OPERATIONS = (
        'pick', 'place',
        'home', 'pick_pose', 'lift', 'place_pose', 'open', 'close', 'move_arm',
    )

    def __init__(self):
        super().__init__('fixed_manipulation_server')
        self._callback_group = ReentrantCallbackGroup()
        self._busy = False
        self._pending_pick = None
        self._last_attach = None
        self._active_placement = None
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
        self._state_client = self.create_client(
            GetEntityState, self._get_state_service_name,
            callback_group=self._callback_group)
        self._set_state_client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state',
            callback_group=self._callback_group)

        # 孤儿跟随防护: 运动学载运跨越 pick/place 两个动作存活, 但若
        # 执行器流程中途失败/停止, 没有任何动作会再来停止跟随, 方块就会
        # 一直"跟"着机器人到处打转。执行器在流程终止时调用本服务释放。
        self.release_service = self.create_service(
            Trigger, '/manipulation/release', self._release_callback)
        # 载运话题: 由 Gazebo 世界插件(CubeCarryPlugin)在物理循环内
        # 刚性跟随工具端, 替代服务调用式跟随
        self.carry_pub = self.create_publisher(
            String, '/manipulation/carry_set', 10)
        # Freeze only the chassis world pose while the arm is moving.  Arm
        # reaction forces otherwise shift the light mobile base by several
        # centimetres between IK measurement and finger closure.
        self.base_lock_pub = self.create_publisher(
            Bool, '/manipulation/base_lock', 10)
        # The physics-loop plugin confirms the exact update step in which it
        # captured the cube-to-tool transform.  Never start an arm lift until
        # that acknowledgement arrives; otherwise a loaded simulator can
        # capture the transform after the arm has already begun moving.
        self._carry_status = ''
        self.carry_status_sub = self.create_subscription(
            String, '/manipulation/carry_status',
            self._carry_status_callback, 10,
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

    def _publish_carry(self, command):
        msg = String()
        msg.data = command
        self.carry_pub.publish(msg)

    def _publish_base_lock(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        self.base_lock_pub.publish(msg)

    def _carry_status_callback(self, message):
        self._carry_status = str(message.data)

    def _await_carry_capture(self, object_id, mode='tool', timeout_s=1.0):
        expected = f'{mode}:{object_id}'
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._carry_status == expected:
                return
            time.sleep(0.02)
        raise ManipulationError(
            self.ATTACH_FAILED,
            f'Cube carry plugin did not confirm {expected} before motion.')

    def _await_carry_release(self, timeout_s=1.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._carry_status == 'none':
                return
            time.sleep(0.02)
        raise ManipulationError(
            self.ATTACH_FAILED,
            'Cube carry plugin did not confirm release before settling.')

    def _await_prepared_release(self, object_id, timeout_s=1.0):
        expected = f'prepared:{object_id}'
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._carry_status == expected:
                return
            time.sleep(0.02)
        raise ManipulationError(
            self.ATTACH_FAILED,
            f'Cube carry plugin did not latch the aligned pose for {object_id}.')


    def _declare_parameters(self):
        defaults = {
            'action_name': '/manipulation/execute',
            'arm_action_name': '/arm_controller/follow_joint_trajectory',
            'gripper_action_name': '/gripper_controller/follow_joint_trajectory',
            'attach_service_name': '/ATTACHLINK',
            'detach_service_name': '/DETACHLINK',
            'get_state_service_name': '/gazebo/get_entity_state',
            'dependency_timeout_s': 10.0,
            'robot_model': 'six_arm',
            'tool_link': 'link6',
            'default_object': 'red_cube_1',
            'object_link': 'link',
            # The scoring zones are static Gazebo models.  After release, the
            # cube is attached to the nearest one so it remains visibly and
            # physically stationary inside the A/B/C placement area.
            'placement_zone_models': ['zone_a', 'zone_b', 'zone_c'],
            'placement_zone_link': 'base',
            'placement_drop_height': 0.07,
            # Each slot is expressed in the scoring-zone model frame. A/B are
            # approached from the north and C from the east; the rows remain
            # inside the 1.0 x 0.5 m signs with five non-overlapping slots.
            'placement_slot_specs': [
                'zone_a:-0.16:0.10', 'zone_a:-0.10:0.10',
                'zone_a:-0.04:0.10', 'zone_a:0.02:0.10',
                'zone_a:0.08:0.10',
                'zone_b:-0.08:0.10', 'zone_b:-0.02:0.10',
                'zone_b:0.04:0.10', 'zone_b:0.10:0.10',
                'zone_b:0.16:0.10',
                'zone_c:0.10:-0.20', 'zone_c:0.10:-0.14',
                'zone_c:0.10:-0.08', 'zone_c:0.10:-0.02',
                'zone_c:0.10:0.04',
            ],
            'placement_objects': [
                'red_cube_1', 'red_cube_2', 'red_cube_3', 'red_cube_4',
                'red_cube_5', 'blue_cube_1', 'blue_cube_2',
                'blue_cube_3', 'blue_cube_4', 'blue_cube_5',
            ],
            'placement_zone_half_width': 0.50,
            'placement_zone_half_depth': 0.25,
            'placement_boundary_margin': 0.025,
            'placement_slot_clearance': 0.055,
            # Release is physical: after the carry plugin lets go, verify
            # that the cube has actually reached the zone floor and ceased
            # moving before the scoring joint is created.  These are bounded
            # service samples, not a high-rate service-following mechanism.
            'placement_settle_samples': 4,
            'placement_settle_sample_period_s': 0.20,
            'placement_ground_tolerance_m': 0.030,
            'placement_position_jitter_tolerance_m': 0.004,
            'placement_speed_tolerance_mps': 0.025,
            'placement_slot_correction_limit_m': 0.035,
            'dynamic_place_ik': True,
            'dynamic_place_max_target_distance_m': 0.55,
            'arm_joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            'gripper_joints': ['finger_joint1'],
            'arm_home': [0.0, 0.6102, 1.2593, 0.0, -1.4931, 0.0],
            'arm_pregrasp': [0.0, 1.0, 1.2, 0.0, -0.6, 0.0],
            'arm_pick': [0.0, 1.2, 1.17, 0.0, -0.3, 0.0],
            'arm_lift': [0.0, 0.6102, 1.2593, 0.0, -1.4931, 0.0],
            'arm_place': [0.0, 1.2, 1.17, 0.0, -0.3, 0.0],
            'gripper_open': [0.04],
            'gripper_closed': [0.0],
            'arm_motion_duration_s': 3.0,
            'gripper_motion_duration_s': 1.5,
            'settle_time_s': 0.5,
            'validate_grasp_window': True,
            'grasp_target_x': 0.0,
            'grasp_target_y': 0.0,
            'grasp_target_z': 0.045,
            'grasp_position_tolerance': 0.08,
            'grasp_xy_tolerance': 0.08,
            'grasp_z_min': 0.0,
            'grasp_z_max': 0.15,
            'attached_drift_tolerance': 0.005,
            # Navigation intentionally stops before touching the target cube.
            # Solve the final arm posture from the measured cube pose, rather
            # than assuming a perfectly repeatable differential-drive stop.
            'dynamic_grasp_ik': True,
            'dynamic_grasp_max_target_distance_m': 0.48,
            # Gazebo merges arm_base_link into the fixed base chain, so the
            # queryable reference is base_footprint. arm_base_link is 0.1634m
            # above it in the URDF (base 0.1034m + arm offset 0.0600m).
            'arm_ik_reference_link': 'base_footprint',
            'arm_ik_reference_to_base_z': 0.1634,
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
        self._get_state_service_name = str(value('get_state_service_name'))
        self._dependency_timeout = float(value('dependency_timeout_s'))
        self._robot_model = str(value('robot_model'))
        self._tool_link = str(value('tool_link'))
        self._default_object = str(value('default_object'))
        self._object_link = str(value('object_link'))
        self._placement_zone_models = list(value('placement_zone_models'))
        self._placement_zone_link = str(value('placement_zone_link'))
        self._placement_drop_height = float(value('placement_drop_height'))
        self._placement_objects = list(value('placement_objects'))
        self._placement_zone_half_width = float(
            value('placement_zone_half_width'))
        self._placement_zone_half_depth = float(
            value('placement_zone_half_depth'))
        self._placement_boundary_margin = float(
            value('placement_boundary_margin'))
        self._placement_slot_clearance = float(
            value('placement_slot_clearance'))
        self._placement_settle_samples = int(
            value('placement_settle_samples'))
        self._placement_settle_sample_period = float(
            value('placement_settle_sample_period_s'))
        self._placement_ground_tolerance = float(
            value('placement_ground_tolerance_m'))
        self._placement_position_jitter_tolerance = float(
            value('placement_position_jitter_tolerance_m'))
        self._placement_speed_tolerance = float(
            value('placement_speed_tolerance_mps'))
        self._placement_slot_correction_limit = float(
            value('placement_slot_correction_limit_m'))
        self._dynamic_place_ik = bool(value('dynamic_place_ik'))
        self._dynamic_place_max_target_distance = float(
            value('dynamic_place_max_target_distance_m'))
        self._placement_slots = {
            zone: [] for zone in self._placement_zone_models
        }
        for spec in value('placement_slot_specs'):
            try:
                zone, xs, ys = str(spec).split(':')
                self._placement_slots[zone].append((float(xs), float(ys)))
            except (KeyError, TypeError, ValueError):
                raise ValueError(f'Invalid placement_slot_specs entry: {spec!r}')
        self._arm_joints = list(value('arm_joints'))
        self._gripper_joints = list(value('gripper_joints'))
        self._arm_home = list(value('arm_home'))
        self._arm_pregrasp = list(value('arm_pregrasp'))
        self._arm_pick = list(value('arm_pick'))
        self._arm_lift = list(value('arm_lift'))
        self._arm_place = list(value('arm_place'))
        self._gripper_open = list(value('gripper_open'))
        self._gripper_closed = list(value('gripper_closed'))
        self._arm_duration = float(value('arm_motion_duration_s'))
        self._gripper_duration = float(value('gripper_motion_duration_s'))
        self._settle_time = float(value('settle_time_s'))
        self._validate_window = bool(value('validate_grasp_window'))
        self._grasp_target_x = float(value('grasp_target_x'))
        self._grasp_target_y = float(value('grasp_target_y'))
        self._grasp_target_z = float(value('grasp_target_z'))
        self._grasp_position_tolerance = float(
            value('grasp_position_tolerance'))
        self._grasp_xy_tolerance = float(value('grasp_xy_tolerance'))
        self._grasp_z_min = float(value('grasp_z_min'))
        self._grasp_z_max = float(value('grasp_z_max'))
        self._attached_drift_tolerance = float(
            value('attached_drift_tolerance')
        )
        self._dynamic_grasp_ik = bool(value('dynamic_grasp_ik'))
        self._dynamic_grasp_max_target_distance = float(
            value('dynamic_grasp_max_target_distance_m'))
        self._arm_ik_reference_link = str(value('arm_ik_reference_link'))
        self._arm_ik_reference_to_base_z = float(
            value('arm_ik_reference_to_base_z'))
        self._dry_run = bool(value('dry_run'))
        self._validate_configuration()

    def _validate_configuration(self):
        vectors = (
            ('arm_home', self._arm_home, self._arm_joints),
            ('arm_pregrasp', self._arm_pregrasp, self._arm_joints),
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
        if self._grasp_xy_tolerance <= 0.0:
            raise ValueError('grasp_xy_tolerance must be positive.')
        if self._grasp_position_tolerance <= 0.0:
            raise ValueError('grasp_position_tolerance must be positive.')
        if self._grasp_z_min >= self._grasp_z_max:
            raise ValueError('grasp_z_min must be less than grasp_z_max.')
        if self._placement_boundary_margin < 0.0:
            raise ValueError('placement_boundary_margin cannot be negative.')
        if self._placement_slot_clearance <= 0.03:
            raise ValueError('placement_slot_clearance must exceed cube width.')
        if self._placement_settle_samples < 3:
            raise ValueError('placement_settle_samples must be at least 3.')
        if self._placement_settle_sample_period <= 0.0:
            raise ValueError('placement_settle_sample_period_s must be positive.')
        if self._placement_ground_tolerance <= 0.0:
            raise ValueError('placement_ground_tolerance_m must be positive.')
        if self._placement_position_jitter_tolerance <= 0.0:
            raise ValueError(
                'placement_position_jitter_tolerance_m must be positive.')
        if self._placement_speed_tolerance <= 0.0:
            raise ValueError('placement_speed_tolerance_mps must be positive.')
        if self._placement_slot_correction_limit <= 0.0:
            raise ValueError('placement_slot_correction_limit_m must be positive.')
        for zone in self._placement_zone_models:
            slots = self._placement_slots.get(zone, [])
            if not slots:
                raise ValueError(f'No placement slots configured for {zone}.')
            for x, y in slots:
                if (abs(x) + self._placement_boundary_margin
                        > self._placement_zone_half_width
                        or abs(y) + self._placement_boundary_margin
                        > self._placement_zone_half_depth):
                    raise ValueError(
                        f'Placement slot ({x:.3f}, {y:.3f}) leaves {zone}.')

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
        # pick 失败回滚记录: 焊接已建立时必须解除并把方块放回抓取前位姿,
        # 否则物理残留会让后续任务与重跑全部失准。
        self._pending_pick = None
        lock_base = (not self._dry_run and operation in ('pick', 'place'))
        try:
            if self._dry_run:
                await self._execute_dry_run(goal_handle, operation, object_id)
            else:
                self._wait_for_dependencies(operation)
                if lock_base:
                    self._publish_base_lock(True)
                    # Give the Gazebo physics-loop subscriber time to capture
                    # the exact planar pose before the first arm trajectory.
                    time.sleep(0.12)
                await self._execute_operation(goal_handle, operation, object_id)
            goal_handle.succeed()
            return self._result(True, self.SUCCESS, f'{operation} completed for {object_id}.')
        except ManipulationError as error:
            self.get_logger().error(
                f'{operation} {object_id} rejected: {error}')
            if error.code == self.CANCELLED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            # 失败后必须收回机械臂：伸展的臂会让轻量差速底盘持续蠕变
            # （实测 1 分钟内偏航漂移超过 30°），并可能卡住后续导航。
            await self._best_effort_recover()
            return self._result(False, error.code, str(error))
        except Exception as error:  # Defensive boundary around hardware callbacks.
            self.get_logger().error(f'Unexpected manipulation failure: {error}')
            goal_handle.abort()
            await self._best_effort_recover()
            return self._result(False, self.INTERNAL_ERROR, str(error))
        finally:
            if lock_base:
                self._publish_base_lock(False)
            self._busy = False

    async def _best_effort_recover(self):
        """Return the arm home after a failure; recovery errors are ignored."""
        if self._pending_pick is not None:
            await self._rollback_failed_pick()
        try:
            await self._send_trajectory(
                self._arm_client, self._arm_joints, self._arm_home,
                self._arm_duration, self.ARM_FAILED, 'recovery home')
            self.get_logger().info('Recovery: arm returned to home pose.')
        except Exception as recovery_error:
            self.get_logger().warning(
                f'Recovery home failed (ignored): {recovery_error}')

    async def _rollback_failed_pick(self):
        pending, self._pending_pick = self._pending_pick, None
        object_id = pending['object_id']
        self._publish_carry('release')
        try:
            await self._detach_stale(object_id)
            if pending.get('world_pose') is None:
                return
            restore = SetEntityState.Request()
            restore.state.name = object_id
            restore.state.reference_frame = 'world'
            restore.state.pose = pending['world_pose']
            restore.state.twist = Twist()
            response = await self._set_state_client.call_async(restore)
            if response.success:
                self.get_logger().info(
                    f'Rollback: {object_id} restored to its pre-pick pose.')
            else:
                self.get_logger().warning(
                    f'Rollback of {object_id} failed: '
                    f'{response.status_message}')
        except Exception as rollback_error:
            self.get_logger().warning(
                f'Rollback of {object_id} raised (ignored): {rollback_error}')

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
                ('/gazebo/set_entity_state',
                 self._set_state_client.wait_for_service(
                    timeout_sec=self._dependency_timeout)),
            ))
        if (operation == 'pick' and self._validate_window) or operation == 'place':
            checks.append((
                self._get_state_service_name,
                self._state_client.wait_for_service(
                    timeout_sec=self._dependency_timeout),
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
        # Approach through a waypoint whose fingertip is 12 cm vertically
        # above the cube.  A single home->pick joint interpolation sweeps the
        # forearm through the object before validation and physically pushes
        # it away; the two-stage path keeps that first sweep above the scene.
        pregrasp = self._arm_pregrasp
        pick = self._arm_pick
        if self._dynamic_grasp_ik:
            await self._stage(handle, 'solve_close_range_grasp', 0.18)
            pregrasp, pick = await self._solve_dynamic_grasp(object_id)
        await self._stage(handle, 'move_to_pregrasp_pose', 0.22)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, pregrasp,
            self._arm_duration, self.ARM_FAILED, 'move arm to pregrasp pose')
        await self._stage(handle, 'descend_to_pick_pose', 0.34)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, pick,
            self._arm_duration, self.ARM_FAILED, 'move arm to pick pose')
        world_pose = None
        if self._validate_window:
            await self._stage(handle, 'validate_grasp_window', 0.42)
            await self._validate_grasp_pose(object_id)
            world_pose = await self._get_world_pose(object_id)
            self._pending_pick = {
                'object_id': object_id,
                'world_pose': world_pose,
            }
        await self._stage(handle, 'close_gripper', 0.50)
        await self._send_trajectory(
            self._gripper_client, self._gripper_joints, self._gripper_closed,
            self._gripper_duration, self.GRIPPER_FAILED, 'close gripper')
        await self._stage(handle, 'physical_grasp_confirmed', 0.62)
        # 不再调用 SetEntityState 把方块瞬移到 link6。Gazebo 世界插件会
        # 从当前已验证的接触位姿捕获相对变换，随后在物理循环内刚性保持。
        await self._stage(handle, 'attach_object', 0.70)
        # CubeCarryPlugin 在物理循环内把方块刚性绑定到工具端: 无焊接、
        # 无跨模型约束、无接触求解互搏, 从机理上杜绝载运弹飞
        self._carry_status = ''
        self._publish_carry(f'carry:{object_id}')
        self._await_carry_capture(object_id)
        # Closing the physical fingers can seat the cube by a few centimetres.
        # That fixed seating offset is not attachment drift: capture the
        # reference only after the plugin has acknowledged the final transform.
        await self._stage(handle, 'lift_object', 0.85)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, self._arm_lift,
            self._arm_duration, self.ARM_FAILED, 'lift arm')
        # Borrow the supplied SafeAutoGrasper's strongest invariant: after
        # lifting, the cube must still occupy the same link6-relative pose.
        # CubeCarryPlugin should keep this drift below a few millimetres; a
        # larger value means the object was beside the fingers or the carry
        # command failed, so navigation must never start with that object.
        if self._validate_window:
            await self._stage(handle, 'validate_attached_drift', 0.92)
            await self._validate_attachment_drift(object_id)
        # Navigation carries the cube relative to the chassis, so arm-servo
        # motion cannot produce vertical jitter while the robot is moving.
        self._carry_status = ''
        self._publish_carry(f'base:{object_id}')
        self._await_carry_capture(object_id, mode='base')
        # 成功后不再回滚
        self._pending_pick = None
        await self._stage(handle, 'pick_complete', 1.0)

    @staticmethod
    def _rotate_by_quaternion(q, v):
        """用四元数 q 旋转向量 v, 返回 (x, y, z)。"""
        qx, qy, qz, qw = q
        vx, vy, vz = v
        # t = 2 * cross(q.xyz, v)
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return (
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        )

    def _stop_kinematic_carry(self):
        self._kinematic_carry = None
        self._carry_offset = None
        self._carry_anchor = None
        if self._carry_timer is not None:
            self._carry_timer.cancel()
            self.destroy_timer(self._carry_timer)
            self._carry_timer = None

    async def _seat_cube_at_grasp_pose(self, object_id):
        """把方块安置到夹爪口正中(参考工程 link6+Z 0.045m 抓取点)。

        导航停车残差(cm级)通过这次安置消除, 闭爪时从视觉上就是夹爪
        夹住方块本身。
        """
        request = GetEntityState.Request()
        request.name = f'{self._robot_model}::{self._tool_link}'
        request.reference_frame = 'world'
        tool_future = self._state_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while not tool_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not tool_future.done() or not tool_future.result().success:
            self.get_logger().warning(
                'seat_cube: tool pose unavailable; skipping')
            return
        tool = tool_future.result().state.pose
        qx, qy, qz, qw = (tool.orientation.x, tool.orientation.y,
                          tool.orientation.z, tool.orientation.w)

        def qrot(vx, vy, vz):
            tx = 2 * (qy * vz - qz * vy)
            ty = 2 * (qz * vx - qx * vz)
            tz = 2 * (qx * vy - qy * vx)
            return (vx + qw * tx + (qy * tz - qz * ty),
                    vy + qw * ty + (qz * tx - qx * tz),
                    vz + qw * tz + (qx * ty - qy * tx))

        ox, oy, oz = qrot(0.0, 0.0, self._grasp_target_z)
        set_req = SetEntityState.Request()
        set_req.state.name = object_id
        set_req.state.reference_frame = 'world'
        set_req.state.pose.position.x = tool.position.x + ox
        set_req.state.pose.position.y = tool.position.y + oy
        set_req.state.pose.position.z = tool.position.z + oz
        set_req.state.pose.orientation.w = 1.0
        set_req.state.twist = Twist()
        await self._set_state_client.call_async(set_req)

    async def _snap_cube_to_gripper(self, object_id):
        """把方块传送到夹爪指尖(工具系 +Z 0.045m), 姿态取工具偏航。
        软接触参数使其与指/地的重叠不产生可感冲量。"""
        tool_pose = await self._get_tool_pose()
        if tool_pose is None:
            return
        q = (tool_pose.orientation.x, tool_pose.orientation.y,
             tool_pose.orientation.z, tool_pose.orientation.w)
        ox, oy, oz = self._rotate_by_quaternion(q, (0.0, 0.0, 0.045))
        request = SetEntityState.Request()
        request.state.name = object_id
        request.state.reference_frame = 'world'
        request.state.pose.position.x = tool_pose.position.x + ox
        request.state.pose.position.y = tool_pose.position.y + oy
        request.state.pose.position.z = tool_pose.position.z + oz
        request.state.pose.orientation.w = 1.0
        request.state.twist = Twist()
        await self._set_state_client.call_async(request)
        self.get_logger().info(
            f'{object_id} snapped into the gripper fingertips.')

    async def _get_tool_pose(self):
        request = GetEntityState.Request()
        request.name = f'{self._robot_model}::{self._tool_link}'
        request.reference_frame = 'world'
        response = await self._state_client.call_async(request)
        if not response.success:
            return None
        return response.state.pose

    @staticmethod
    def _rotate_by_quaternion(q, v):
        """用四元数 q 旋转向量 v, 返回 (x, y, z)。"""
        qx, qy, qz, qw = q
        vx, vy, vz = v
        # t = 2 * cross(q.xyz, v)
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return (
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        )

    def _stop_kinematic_carry(self):
        self._kinematic_carry = None
        self._carry_offset = None
        self._carry_anchor = None
        if self._carry_timer is not None:
            self._carry_timer.cancel()
            self.destroy_timer(self._carry_timer)
            self._carry_timer = None

    def _release_callback(self, request, response):
        self._publish_carry('release')
        response.success = True
        response.message = 'carry release requested'
        return response

    def _carry_follow(self):
        """载运跟随: 方块刚接在底盘上。

        抬臂完成瞬间锁存方块世界位姿, 之后每个周期按底盘位移平移方块。
        不用工具姿态旋转偏置: 工具四元数在关节伺服下微抖, 反映到方块上
        就是用户看到的"夹爪竖直方向一直运动"。底盘锁存 = 方块只在
        小车移动时移动, 与真实搬运一致。
        """
        if self._kinematic_carry is None:
            return
        base_req = GetEntityState.Request()
        base_req.name = self._robot_model
        base_req.reference_frame = 'world'
        base_future = self._state_client.call_async(base_req)

        def on_base(fut):
            try:
                result = fut.result()
                if not result.success:
                    return
                base = result.state.pose
                if self._carry_anchor is None:
                    # 首个周期: 锁存方块与底盘的相对关系
                    cube_req = GetEntityState.Request()
                    cube_req.name = self._kinematic_carry
                    cube_req.reference_frame = 'world'
                    cube_future = self._state_client.call_async(cube_req)

                    def on_cube(cube_fut, base=base):
                        try:
                            cube_res = cube_fut.result()
                            if not cube_res.success:
                                return
                            cube = cube_res.state.pose
                            self._carry_anchor = {
                                'base': (base.position.x,
                                         base.position.y,
                                         base.position.z),
                                'cube': (cube.position.x,
                                         cube.position.y,
                                         cube.position.z),
                                'orientation': cube.orientation,
                            }
                        except Exception as error:
                            self.get_logger().warning(
                                f'carry anchor skipped: {error}')
                    cube_future.add_done_callback(on_cube)
                    return
                dx = (base.position.x - self._carry_anchor['base'][0])
                dy = (base.position.y - self._carry_anchor['base'][1])
                dz = (base.position.z - self._carry_anchor['base'][2])
                set_req = SetEntityState.Request()
                set_req.state.name = self._kinematic_carry
                set_req.state.reference_frame = 'world'
                set_req.state.pose.position.x = (
                    self._carry_anchor['cube'][0] + dx)
                set_req.state.pose.position.y = (
                    self._carry_anchor['cube'][1] + dy)
                set_req.state.pose.position.z = (
                    self._carry_anchor['cube'][2] + dz)
                set_req.state.pose.orientation = (
                    self._carry_anchor['orientation'])
                set_req.state.twist = Twist()
                self._set_state_client.call_async(set_req)
            except Exception as error:
                self.get_logger().warning(
                    f'carry follow skipped: {error}')
        base_future.add_done_callback(on_base)

    async def _get_world_pose(self, object_id):
        request = GetEntityState.Request()
        request.name = object_id
        request.reference_frame = 'world'
        response = await self._state_client.call_async(request)
        if not response.success:
            return None
        return response.state.pose

    async def _get_entity_pose(self, entity, reference_frame='world'):
        request = GetEntityState.Request()
        request.name = entity
        request.reference_frame = reference_frame
        response = await self._state_client.call_async(request)
        if not response.success:
            return None
        return response.state.pose

    async def _get_entity_state(self, entity, reference_frame='world'):
        """Return one Gazebo truth sample, including the physical twist."""
        request = GetEntityState.Request()
        request.name = entity
        request.reference_frame = reference_frame
        response = await self._state_client.call_async(request)
        if not response.success:
            return None
        return response.state

    @staticmethod
    def _rotate_by_quaternion(q, v):
        """Rotate vector *v* by quaternion *q*=(x,y,z,w)."""
        qx, qy, qz, qw = q
        vx, vy, vz = v
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return (
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        )

    async def _solve_dynamic_place(self, object_id):
        """Choose a free in-zone slot and solve the real arm pose for it.

        The reference project demonstrates a useful staged place trajectory,
        but its single fixed joint pose makes every delivery overlap.  Here the
        floor signs are treated as finite storage bays: Gazebo truth determines
        the active bay and already occupied slots, then the same verified FK/IK
        model used for grasping places the carried cube into a free slot.
        """
        robot_pose = await self._get_entity_pose(self._robot_model)
        if robot_pose is None:
            raise ManipulationError(
                self.ARM_FAILED, 'Cannot locate the robot for dynamic place IK.')

        nearest_zone = None
        nearest_distance = float('inf')
        for zone in self._placement_zone_models:
            pose = await self._get_entity_pose(zone)
            if pose is None:
                continue
            distance = math.hypot(
                robot_pose.position.x - pose.position.x,
                robot_pose.position.y - pose.position.y)
            if distance < nearest_distance:
                nearest_zone = zone
                nearest_distance = distance
        if nearest_zone is None or nearest_distance > 2.5:
            raise ManipulationError(
                self.ARM_FAILED,
                'Robot is not close enough to an A/B/C zone for placement.')

        occupied = []
        for candidate in self._placement_objects:
            if candidate == object_id:
                continue
            pose = await self._get_entity_pose(candidate, nearest_zone)
            if pose is None or pose.position.z > 0.20:
                continue
            if (abs(pose.position.x) <= self._placement_zone_half_width
                    and abs(pose.position.y)
                    <= self._placement_zone_half_depth):
                occupied.append((pose.position.x, pose.position.y, candidate))

        reference = f'{self._robot_model}::{self._arm_ik_reference_link}'
        zone_in_base = await self._get_entity_pose(nearest_zone, reference)
        if zone_in_base is None:
            raise ManipulationError(
                self.ARM_FAILED,
                f'Cannot locate {nearest_zone} relative to {reference}.')
        orientation = zone_in_base.orientation
        quaternion = (
            orientation.x, orientation.y, orientation.z, orientation.w)

        best = None
        for slot_index, (slot_x, slot_y) in enumerate(
                self._placement_slots[nearest_zone]):
            if any(math.hypot(slot_x - x, slot_y - y)
                   < self._placement_slot_clearance
                   for x, y, _name in occupied):
                continue
            ox, oy, oz = self._rotate_by_quaternion(
                quaternion,
                (slot_x, slot_y, self._placement_drop_height),
            )
            target = np.array((
                zone_in_base.position.x + ox,
                zone_in_base.position.y + oy,
                zone_in_base.position.z + oz
                - self._arm_ik_reference_to_base_z,
            ), dtype=float)
            target_distance = float(np.linalg.norm(target))
            if target_distance > self._dynamic_place_max_target_distance:
                continue
            joints, residual = self._solve_grasp_center_ik(
                target, self._arm_place)
            # Prefer the configured fill order.  Residual only breaks ties and
            # rejects a slot that the physical arm cannot actually reach.
            score = float(slot_index) + min(0.99, 20.0 * residual)
            if residual <= 0.012 and (best is None or score < best['score']):
                best = {
                    'zone': nearest_zone,
                    'slot_index': slot_index,
                    'slot': (slot_x, slot_y),
                    'target': target,
                    'joints': joints,
                    'residual': residual,
                    'score': score,
                }

        if best is None:
            raise ManipulationError(
                self.ARM_FAILED,
                f'No free reachable placement slot remains in {nearest_zone}.')
        self._active_placement = best
        self.get_logger().info(
            f'Dynamic place {object_id}: {nearest_zone} slot '
            f'{best["slot_index"] + 1}, local=({best["slot"][0]:.3f},'
            f'{best["slot"][1]:.3f}), IK residual='
            f'{best["residual"]:.4f}m, occupied={len(occupied)}.')
        return best['joints']

    @staticmethod
    def _grasp_fk_center(joints):
        """Return link6 + 45mm fingertip centre in arm_base_link.

        The transform chain mirrors arm.xacro.  It is deliberately limited
        to position IK: joint4/joint6 retain their collision-safe seed values
        while the six available joints provide the small correction needed
        after collision-monitored base docking.
        """
        origins = (
            (0.0, 0.0, 0.07), (0.0, 0.0, 0.05),
            (0.0, 0.0, 0.14), (0.0, 0.0, 0.22),
            (0.0, 0.0, 0.06), (0.0, 0.0, 0.06),
        )
        axes = (
            (0.0, 0.0, 1.0), (0.0, 1.0, 0.0),
            (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
            (0.0, -1.0, 0.0), (0.0, 0.0, 1.0),
        )
        matrix = np.eye(4)
        for origin, axis, angle in zip(origins, axes, joints):
            translate = np.eye(4)
            translate[:3, 3] = origin
            ax, ay, az = axis
            skew = np.array([
                (0.0, -az, ay), (az, 0.0, -ax), (-ay, ax, 0.0),
            ])
            rotate = np.eye(4)
            rotate[:3, :3] = (
                np.eye(3) + math.sin(angle) * skew
                + (1.0 - math.cos(angle)) * (skew @ skew)
            )
            matrix = matrix @ translate @ rotate
        return (matrix @ np.array((0.0, 0.0, 0.045, 1.0)))[:3]

    def _solve_grasp_center_ik(self, target, seed):
        """Position IK with a fixed, verified vertical gripper attitude."""
        joints = np.asarray(seed, dtype=float).copy()
        lower = np.array((-2.30, -2.30, -2.57, -2.30, -2.30, -2.30))
        upper = np.array((2.30, 2.30, 2.57, 2.30, 2.30, 2.30))
        target = np.asarray(target, dtype=float)
        # For this arm chain, q2 + q3 - q5 is the pitch of the tool's
        # approach axis.  A pure position solve can rotate the fingers
        # sideways while still placing their centre on the cube.  Keep the
        # tested downward attitude as a soft fourth IK equation.
        pitch_axis = np.array((0.0, 1.0, 1.0, 0.0, -1.0, 0.0))
        reference_pitch = float(pitch_axis @ np.asarray(seed, dtype=float))
        for _ in range(260):
            centre = self._grasp_fk_center(joints)
            error = target - centre
            if float(np.linalg.norm(error)) <= 0.0015:
                break
            epsilon = 1e-5
            columns = []
            for index in range(6):
                perturbed = joints.copy()
                perturbed[index] += epsilon
                columns.append(
                    (self._grasp_fk_center(perturbed) - centre) / epsilon)
            jacobian = np.column_stack(columns)
            pitch_error = reference_pitch - float(pitch_axis @ joints)
            augmented_jacobian = np.vstack((
                jacobian,
                0.22 * pitch_axis,
            ))
            augmented_error = np.append(error, 0.22 * pitch_error)
            try:
                delta = np.linalg.solve(
                    augmented_jacobian.T @ augmented_jacobian
                    + 0.004 * np.eye(6),
                    augmented_jacobian.T @ augmented_error,
                )
            except np.linalg.LinAlgError:
                break
            length = float(np.linalg.norm(delta))
            if length > 0.055:
                delta *= 0.055 / length
            joints = np.clip(joints + delta, lower, upper)
        residual = float(np.linalg.norm(
            self._grasp_fk_center(joints) - target))
        return [float(value) for value in joints], residual

    async def _solve_dynamic_grasp(self, object_id):
        request = GetEntityState.Request()
        request.name = object_id
        request.reference_frame = (
            f'{self._robot_model}::{self._arm_ik_reference_link}')
        response = await self._state_client.call_async(request)
        if not response.success:
            raise ManipulationError(
                self.GRASP_WINDOW_FAILED,
                f'Cannot measure {object_id} in '
                f'{self._arm_ik_reference_link} for IK.',
            )
        position = response.state.pose.position
        cube = np.array((
            position.x,
            position.y,
            position.z - self._arm_ik_reference_to_base_z,
        ), dtype=float)
        distance = float(np.linalg.norm(cube))
        if distance > self._dynamic_grasp_max_target_distance:
            raise ManipulationError(
                self.GRASP_WINDOW_FAILED,
                f'{object_id} is {distance:.3f}m from arm base; '
                'outside safe dynamic-grasp reach.',
            )
        pick, pick_error = self._solve_grasp_center_ik(cube, self._arm_pick)
        # A vertical 12cm clearance keeps the forearm from sweeping through
        # the cube during the final approach; only the final descent enters
        # the finger closing volume.
        pre_target = cube + np.array((0.0, 0.0, 0.12))
        pregrasp, pre_error = self._solve_grasp_center_ik(pre_target, pick)
        if pick_error > 0.008 or pre_error > 0.010:
            raise ManipulationError(
                self.GRASP_WINDOW_FAILED,
                f'Dynamic IK residual too high for {object_id}: '
                f'pick={pick_error:.4f}m, pregrasp={pre_error:.4f}m.',
            )
        self.get_logger().info(
            f'Dynamic IK {object_id}: arm-base target='
            f'({cube[0]:.3f},{cube[1]:.3f},{cube[2]:.3f}), '
            f'pick_error={pick_error:.4f}m, pre_error={pre_error:.4f}m.')
        return pregrasp, pick

    async def _get_relative_pose(self, object_id):
        request = GetEntityState.Request()
        request.name = object_id
        request.reference_frame = f'{self._robot_model}::{self._tool_link}'
        response = await self._state_client.call_async(request)
        if not response.success:
            raise ManipulationError(
                self.GRASP_WINDOW_FAILED,
                f'Cannot query {object_id} relative to {request.reference_frame}.',
            )
        return response.state.pose

    async def _validate_grasp_pose(self, object_id):
        pose = await self._get_relative_pose(object_id)
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z)
        dx = x - self._grasp_target_x
        dy = y - self._grasp_target_y
        dz = z - self._grasp_target_z
        position_error = math.sqrt(dx * dx + dy * dy + dz * dz)
        in_window = (
            abs(x) <= self._grasp_xy_tolerance
            and abs(y) <= self._grasp_xy_tolerance
            and self._grasp_z_min <= z <= self._grasp_z_max
            and position_error <= self._grasp_position_tolerance
        )
        self.get_logger().info(
            f'{object_id} relative to {self._tool_link}: '
            f'x={x:.4f}, y={y:.4f}, z={z:.4f}, '
            f'calibrated_error={position_error:.4f} m, '
            f'in_grasp_window={in_window}'
        )
        if not in_window:
            raise ManipulationError(
                self.GRASP_WINDOW_FAILED,
                f'{object_id} is outside the grasp window: '
                f'x={x:.4f}, y={y:.4f}, z={z:.4f}, '
                f'calibrated_error={position_error:.4f} m.',
            )
        return pose

    async def _validate_attachment_drift(self, object_id):
        """Validate a real lift and post-lift stability.

        Gazebo's GetEntityState service and WorldUpdateBegin execute in
        different physics phases.  Comparing a pre-lift service sample with a
        post-lift sample therefore contains a stable 2--4 cm phase offset even
        when CubeCarryPlugin is perfectly rigid.  Measure what matters:
        plugin ownership, actual off-ground height, and jitter between settled
        post-lift samples.
        """
        expected = f'tool:{object_id}'
        if self._carry_status != expected:
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Cube carry ownership lost after lift: expected {expected}, '
                f'got {self._carry_status or "none"}.')

        world_pose = await self._get_world_pose(object_id)
        if world_pose is None or float(world_pose.position.z) < 0.08:
            height = -1.0 if world_pose is None else world_pose.position.z
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'{object_id} did not lift clear of the floor: z={height:.4f}m.')

        samples = []
        for attempt in range(3):
            if attempt:
                time.sleep(0.40)
            pose = await self._get_relative_pose(object_id)
            samples.append((
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ))
        reference = samples[0]
        max_jitter = max(
            math.sqrt(sum((value - base) ** 2
                          for value, base in zip(sample, reference)))
            for sample in samples[1:]
        )
        self.get_logger().info(
            f'{object_id} lifted z={world_pose.position.z:.4f}m; '
            f'post-lift relative jitter={max_jitter:.6f}m')
        if max_jitter > self._attached_drift_tolerance:
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'{object_id} post-lift jitter {max_jitter:.6f}m exceeds '
                f'{self._attached_drift_tolerance:.6f}m.')

    async def _align_held_cube_to_selected_slot(self, object_id, seed_joints):
        """Close the 3-D gap between model IK and the physically held cube.

        Dynamic place IK positions the gripper centre, while CubeCarryPlugin
        preserves the measured cube-to-tool transform.  Before opening the
        fingers, verify the *actual held cube* in the zone frame and use up to
        two bounded, smooth arm corrections.  XY-only calibration can leave a
        cube 10--15 cm above the visual-only zone floor, so the actual cube
        centre must also reach the configured ground-contact height before the
        fingers open.  This is deliberately a low-rate pre-release calibration,
        never a service-based carry loop.
        """
        active = self._active_placement
        if active is None:
            return seed_joints
        zone = active['zone']
        slot_x, slot_y = active['slot']
        reference = f'{self._robot_model}::{self._arm_ik_reference_link}'
        target = np.asarray(active['target'], dtype=float).copy()
        joints = list(seed_joints)

        for correction_index in range(2):
            cube_in_zone = await self._get_entity_pose(object_id, zone)
            if cube_in_zone is None:
                raise ManipulationError(
                    self.ATTACH_FAILED,
                    f'Cannot measure held {object_id} relative to {zone}.')
            error_x = slot_x - float(cube_in_zone.position.x)
            error_y = slot_y - float(cube_in_zone.position.y)
            error_z = (
                self._placement_drop_height
                - float(cube_in_zone.position.z)
            )
            planar_error = math.hypot(error_x, error_y)
            self.get_logger().info(
                f'{object_id} held-slot alignment {correction_index}/2: '
                f'actual=({cube_in_zone.position.x:.3f},'
                f'{cube_in_zone.position.y:.3f},'
                f'{cube_in_zone.position.z:.3f}), '
                f'target=({slot_x:.3f},{slot_y:.3f},'
                f'{self._placement_drop_height:.3f}), '
                f'planar_error={planar_error:.4f}m, '
                f'vertical_error={abs(error_z):.4f}m.')
            if (
                planar_error <= self._placement_slot_correction_limit
                and abs(error_z) <= 0.012
            ):
                return joints

            zone_in_base = await self._get_entity_pose(zone, reference)
            if zone_in_base is None:
                raise ManipulationError(
                    self.ATTACH_FAILED,
                    f'Cannot express {zone} correction in {reference}.')
            q = zone_in_base.orientation
            correction_x, correction_y, correction_z = (
                self._rotate_by_quaternion(
                    (q.x, q.y, q.z, q.w),
                    (error_x, error_y, error_z),
                )
            )
            target += (correction_x, correction_y, correction_z)
            if (float(np.linalg.norm(target))
                    > self._dynamic_place_max_target_distance):
                raise ManipulationError(
                    self.ATTACH_FAILED,
                    f'{object_id} held-slot correction exceeds safe arm reach '
                    f'({float(np.linalg.norm(target)):.3f}m).')
            corrected_joints, residual = self._solve_grasp_center_ik(
                target, joints)
            if residual > 0.012:
                raise ManipulationError(
                    self.ATTACH_FAILED,
                    f'{object_id} held-slot correction IK residual '
                    f'{residual:.4f}m is unsafe.')
            await self._send_trajectory(
                self._arm_client, self._arm_joints, corrected_joints,
                self._arm_duration, self.ARM_FAILED,
                'align held cube to selected placement slot')
            joints = corrected_joints

        cube_in_zone = await self._get_entity_pose(object_id, zone)
        if cube_in_zone is None:
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Cannot verify final held {object_id} alignment in {zone}.')
        final_planar_error = math.hypot(
            slot_x - float(cube_in_zone.position.x),
            slot_y - float(cube_in_zone.position.y))
        final_vertical_error = abs(
            self._placement_drop_height - float(cube_in_zone.position.z))
        if (
            final_planar_error > self._placement_slot_correction_limit
            or final_vertical_error > 0.012
        ):
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'{object_id} remains outside the selected {zone} release '
                f'window while held: planar_error={final_planar_error:.4f}m, '
                f'vertical_error={final_vertical_error:.4f}m; refusing an '
                'out-of-zone or suspended release.')
        return joints

    async def _validate_release_settle(self, object_id, zone_world):
        """Accept a place only after a released cube is grounded and quiet.

        The CubeCarryPlugin owns the object during arm motion and is released
        exactly once.  This bounded post-release check gives placement the
        same evidence standard as grasp validation without restoring the old
        high-frequency Gazebo service follower.
        """
        samples = []
        for attempt in range(self._placement_settle_samples):
            if attempt:
                time.sleep(self._placement_settle_sample_period)
            state = await self._get_entity_state(object_id)
            if state is None:
                raise ManipulationError(
                    self.RELEASE_FAILED,
                    f'Cannot sample released {object_id} for settle check.')
            position = state.pose.position
            velocity = state.twist.linear
            samples.append((
                float(position.x), float(position.y), float(position.z),
                math.sqrt(
                    float(velocity.x) ** 2
                    + float(velocity.y) ** 2
                    + float(velocity.z) ** 2),
            ))

        first = samples[0]
        planar_jitter = max(
            math.hypot(sample[0] - first[0], sample[1] - first[1])
            for sample in samples[1:]
        )
        vertical_jitter = max(
            abs(sample[2] - first[2]) for sample in samples[1:])
        max_speed = max(sample[3] for sample in samples)
        final = samples[-1]
        # placement_drop_height is the selected cube-centre height in the
        # zone frame.  The scoring-zone graphic has no collision geometry,
        # so validating against an invented sign thickness would accept a
        # falling/hovering cube instead of a genuine ground-contact release.
        expected_z = (
            float(zone_world.position.z) + self._placement_drop_height)
        grounded = abs(final[2] - expected_z) <= self._placement_ground_tolerance
        stable = (
            planar_jitter <= self._placement_position_jitter_tolerance
            and vertical_jitter <= self._placement_position_jitter_tolerance
            and max_speed <= self._placement_speed_tolerance
        )
        self.get_logger().info(
            f'{object_id} physical release: xy=({final[0]:.4f},'
            f'{final[1]:.4f}), z={final[2]:.4f}m '
            f'(floor target={expected_z:.4f}m), planar_jitter='
            f'{planar_jitter:.6f}m, vertical_jitter={vertical_jitter:.6f}m, '
            f'max_speed={max_speed:.6f}m/s, grounded={grounded}, '
            f'stable={stable}.')
        if not grounded:
            raise ManipulationError(
                self.RELEASE_FAILED,
                f'{object_id} was released but is not grounded: '
                f'z={final[2]:.4f}m, expected={expected_z:.4f}m.')
        if not stable:
            raise ManipulationError(
                self.RELEASE_FAILED,
                f'{object_id} release has residual motion: planar_jitter='
                f'{planar_jitter:.6f}m, vertical_jitter='
                f'{vertical_jitter:.6f}m, max_speed={max_speed:.6f}m/s.')
        return final

    async def _place(self, handle, object_id):
        # The final placement is an arm operation, so temporarily return to
        # the tool-fixed mode only while the chassis is stationary.  This
        # mode switch is asynchronous and captures the current cube-to-tool
        # transform.  Wait for its acknowledgement before moving the arm;
        # otherwise the plugin can capture midway through the trajectory and
        # preserve a metre-scale offset until release.
        self._carry_status = ''
        self._publish_carry(f'tool:{object_id}')
        self._await_carry_capture(object_id, mode='tool')
        place_joints = self._arm_place
        if self._dynamic_place_ik:
            await self._stage(handle, 'solve_free_zone_slot', 0.12)
            place_joints = await self._solve_dynamic_place(object_id)
        await self._stage(handle, 'move_to_place_pose', 0.20)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, place_joints,
            self._arm_duration, self.ARM_FAILED, 'move arm to place pose')
        if self._dynamic_place_ik:
            await self._stage(handle, 'align_held_cube_to_slot', 0.42)
            place_joints = await self._align_held_cube_to_selected_slot(
                object_id, place_joints)
        # Latch the physically measured, already-validated landing pose before
        # the fingers move.  The release guard will use this Gazebo pose rather
        # than sampling again after a light cube may have been disturbed by the
        # opening contact.  This is one bounded command per place, not a
        # service-based carry loop and not a nominal-slot teleport.
        self._carry_status = ''
        self._publish_carry(f'prepare_release:{object_id}')
        self._await_prepared_release(object_id)
        await self._stage(handle, 'open_gripper', 0.50)
        await self._send_trajectory(
            self._gripper_client, self._gripper_joints, self._gripper_open,
            self._gripper_duration, self.GRIPPER_FAILED, 'open gripper')
        await self._stage(handle, 'detach_object', 0.70)
        # 视觉真实化: 张爪已完成, 此刻发送 release 让方块从夹爪间
        # 物理下落, 软接触让它缓缓落定, 而非瞬移到区中心。
        self._carry_status = ''
        self._publish_carry('release')
        self._await_carry_release()
        await self._detach_stale(object_id)
        await self._stage(handle, 'settle_in_placement_zone', 0.78)
        time.sleep(1.2)
        await self._anchor_object_in_nearest_zone(object_id, teleport=False)
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

        deadline = time.monotonic() + 30.0
        goal_future = client.send_goal_async(goal)
        while not goal_future.done():
            if time.monotonic() > deadline:
                raise ManipulationError(
                    error_code,
                    f'Controller goal response timed out: {description}.')
            time.sleep(0.05)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            raise ManipulationError(error_code, f'Controller rejected: {description}.')
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if time.monotonic() > deadline:
                goal_handle.cancel_goal_async()
                raise ManipulationError(
                    error_code,
                    f'Controller result timed out: {description}.')
            time.sleep(0.05)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise ManipulationError(
                error_code,
                f'Controller failed: {description}; error_code={result.error_code}.')

    async def _call_attach(self, object_id, attach):
        if attach:
            # LinkAttacher 维护全局 IsAttached 标志(不分配对): 上一任务的
            # 遗留附着不解除, 新附着会被 "Both links have already been
            # attached" 拒绝。先精确解除最近一次附着, 再按配对试探清理。
            await self._detach_stale(object_id)
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
        if attach:
            self._last_attach = (
                self._robot_model, self._tool_link,
                object_id, self._object_link,
            )

    async def _detach_stale(self, object_id):
        # 1) 最近一次成功附着的精确配对(全局标志的持有者)
        candidates = []
        if self._last_attach is not None:
            candidates.append(self._last_attach)
            self._last_attach = None
        # 2) 与本物块可能存在的配对
        candidates.append((self._robot_model, self._tool_link,
                           object_id, self._object_link))
        candidates.extend(
            (zone, self._placement_zone_link, object_id, self._object_link)
            for zone in self._placement_zone_models
        )
        for model1, link1, model2, link2 in candidates:
            request = DetachLink.Request()
            request.model1_name = model1
            request.link1_name = link1
            request.model2_name = model2
            request.link2_name = link2
            future = self._detach_client.call_async(request)
            deadline = time.monotonic() + 1.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)

    async def _anchor_object_in_nearest_zone(self, object_id, teleport=True):
        """Validate the physical drop and lock it to the scoring-zone floor."""
        robot = await self._get_entity_pose(self._robot_model)
        if robot is None:
            raise ManipulationError(
                self.ATTACH_FAILED,
                'Cannot locate the robot for placement-zone anchoring.')

        nearest_zone = (
            self._active_placement['zone']
            if self._active_placement is not None else None)
        nearest_distance = float('inf')
        for zone_model in self._placement_zone_models:
            zone_pose = await self._get_entity_pose(zone_model)
            if zone_pose is None:
                continue
            distance = math.hypot(
                robot.position.x - zone_pose.position.x,
                robot.position.y - zone_pose.position.y)
            if (nearest_zone is None and distance < nearest_distance
                    or zone_model == nearest_zone):
                nearest_zone = zone_model
                nearest_distance = distance
                if self._active_placement is not None:
                    break

        # 站位距 zone 中心可达 2.5m: 放置站位与 zone 边缘保持安全距离
        # (防 MPPI 死区), 锚定服务以 zone 中心为参照。
        if nearest_zone is None or nearest_distance > 2.5:
            raise ManipulationError(
                self.ATTACH_FAILED,
                'Robot is not inside a recognised A/B/C placement zone.')

        # The ordinary path is a real gripper release and gravity drop.  Query
        # the settled cube in the zone frame and require its complete 3 cm body
        # to be inside the 1.0 x 0.5 m scoring rectangle before it is secured.
        zone_world = await self._get_entity_pose(nearest_zone)
        if zone_world is None:
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Cannot query {nearest_zone} for placement settling.')
        released_world = await self._validate_release_settle(
            object_id, zone_world)
        cube_in_zone = await self._get_entity_pose(object_id, nearest_zone)
        if cube_in_zone is None:
            raise ManipulationError(
                self.RELEASE_FAILED,
                f'Cannot verify {object_id} inside {nearest_zone}.')
        x = float(cube_in_zone.position.x)
        y = float(cube_in_zone.position.y)
        x_limit = (
            self._placement_zone_half_width
            - self._placement_boundary_margin)
        y_limit = (
            self._placement_zone_half_depth
            - self._placement_boundary_margin)
        inside = abs(x) <= x_limit and abs(y) <= y_limit
        if not inside:
            raise ManipulationError(
                self.RELEASE_FAILED,
                f'{object_id} settled outside {nearest_zone}: '
                f'local=({x:.3f},{y:.3f}).')

        # Slot centres guide the arm to different parts of the finite zone;
        # they are not a licence to reject an otherwise valid physical drop.
        # Decide collisions from Gazebo truth at the actual landing point, so
        # a stable in-zone cube is never picked up again merely to chase a
        # nominal XY coordinate.
        for candidate in self._placement_objects:
            if candidate == object_id:
                continue
            candidate_pose = await self._get_entity_pose(candidate, nearest_zone)
            if candidate_pose is None or candidate_pose.position.z > 0.20:
                continue
            candidate_x = float(candidate_pose.position.x)
            candidate_y = float(candidate_pose.position.y)
            candidate_inside = (
                abs(candidate_x) <= self._placement_zone_half_width
                and abs(candidate_y) <= self._placement_zone_half_depth)
            if (candidate_inside and math.hypot(x - candidate_x, y - candidate_y)
                    < self._placement_slot_clearance):
                raise ManipulationError(
                    self.RELEASE_FAILED,
                    f'{object_id} landed {math.hypot(x - candidate_x, y - candidate_y):.4f}m '
                    f'from {candidate} in {nearest_zone}; insufficient '
                    'physical placement clearance.')
        if self._active_placement is not None:
            # Gravity release remains visible, but before the scoring joint is
            # attached, remove the small residual roll/pitch and contact
            # velocity.  Otherwise the zone joint permanently preserves a
            # tilted cube, and the last A-slot release can kick a neighbour.
            # Keep the released XY truth; this is a level/damping correction,
            # never a lateral snap to a nominal slot centre.
            slot_x, slot_y = self._active_placement['slot']
            slot_error = math.hypot(x - slot_x, y - slot_y)
            q = (
                zone_world.orientation.x, zone_world.orientation.y,
                zone_world.orientation.z, zone_world.orientation.w)
            _ox, _oy, oz = self._rotate_by_quaternion(
                q, (0.0, 0.0, self._placement_drop_height))
            correction = SetEntityState.Request()
            correction.state.name = object_id
            correction.state.reference_frame = 'world'
            correction.state.pose.position.x = released_world[0]
            correction.state.pose.position.y = released_world[1]
            correction.state.pose.position.z = zone_world.position.z + oz
            correction.state.pose.orientation = zone_world.orientation
            correction.state.twist = Twist()
            response = await self._set_state_client.call_async(correction)
            if not response.success:
                raise ManipulationError(
                    self.RELEASE_FAILED,
                    f'Final level correction failed for {object_id}: '
                    f'{response.status_message}')
            self.get_logger().info(
                f'{object_id} levelled at physical {nearest_zone} landing '
                f'({x:.3f},{y:.3f}); selected slot=({slot_x:.3f},'
                f'{slot_y:.3f}), offset={slot_error:.4f}m before securing.')
            time.sleep(self._settle_time)

        request = AttachLink.Request()
        request.model1_name = nearest_zone
        request.link1_name = self._placement_zone_link
        request.model2_name = object_id
        request.link2_name = self._object_link
        response = await self._attach_client.call_async(request)
        if not response.success:
            raise ManipulationError(
                self.RELEASE_FAILED,
                f'Failed to secure {object_id} in {nearest_zone}: {response.message}')
        if response.success:
            self._last_attach = (
                nearest_zone, self._placement_zone_link,
                object_id, self._object_link,
            )
            self._active_placement = None
            self.get_logger().info(
                f'{object_id} physically placed inside and secured to '
                f'{nearest_zone}; robot-zone distance={nearest_distance:.2f}m.')

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
