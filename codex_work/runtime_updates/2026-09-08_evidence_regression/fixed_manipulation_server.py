#!/usr/bin/env python3
"""Execute configuration-driven fixed pick/place sequences.

Phases 3-8: CARRY_HOLD state machine, low-position place, mission
executor closed-loop, dynamic IK, finite retry, per-segment speed.
"""

import json
import math
import re
import time

import numpy as np

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetEntityState, SetEntityState
from linkattacher_msgs.srv import AttachLink, DetachLink
from moon_warehouse_interfaces.action import ExecuteManipulation
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ManipulationError(RuntimeError):
    """Expected manipulation failure carrying a stable error code."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class ArmState:
    """Manipulation lifecycle states (phase-3)."""
    IDLE = 'IDLE'
    PICKING = 'PICKING'
    CARRY_HOLD = 'CARRY_HOLD'
    PLACING = 'PLACING'
    RETURNING_HOME = 'RETURNING_HOME'
    RECOVERING = 'RECOVERING'
    ERROR = 'ERROR'


class FixedManipulationServer(Node):
    """Action server with state machine, dynamic IK, retry, speed config."""

    SUCCESS = 0
    INVALID_GOAL = 1
    DEPENDENCY_UNAVAILABLE = 2
    ARM_FAILED = 3
    GRIPPER_FAILED = 4
    ATTACH_FAILED = 5
    CANCELLED = 6
    GRASP_WINDOW_FAILED = 7
    INTERNAL_ERROR = 99
    SUPPORTED_OPERATIONS = (
        'pick', 'place',
        'home', 'pick_pose', 'lift', 'place_pose', 'open', 'close', 'move_arm',
    )

    def __init__(self):
        super().__init__('fixed_manipulation_server')
        self._callback_group = ReentrantCallbackGroup()
        self._busy = False

        # Parameters must exist before any ROS interface uses their names.
        self._declare_parameters()
        self._load_parameters()

        # Phase-3: State machine
        self._state = ArmState.IDLE
        self._carried_object_id = None
        self._pending_pick = None
        self._last_attach = None
        self._carry_reference_offset = None
        self._active_grasp_solution = None
        self._active_placement_zone = None
        # Carry health monitoring (phase-3)
        self._carry_health_failure_count = 0
        self._stage_times = {}
        self._pick_start_time = 0.0
        self._place_start_time = 0.0

        self._carry_status_publisher = self.create_publisher(
            String, '/manipulation/carry_status', 10,
            callback_group=self._callback_group)
        self._stage_publisher = self.create_publisher(
            String, '/manipulation/stage', 10,
            callback_group=self._callback_group)
        self._carry_health_timer = self.create_timer(
            0.5, self._carry_health_check_callback,
            callback_group=self._callback_group)

        self._arm_client = ActionClient(
            self, FollowJointTrajectory, self._arm_action_name,
            callback_group=self._callback_group,
        )
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, self._gripper_action_name,
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
            SetEntityState, self._set_state_service_name,
            callback_group=self._callback_group)
        # Gazebo model states for dynamic IK input (phase-6)
        self._model_states_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self._model_states_callback, 10,
            callback_group=self._callback_group)
        self._latest_model_states = None

        self._server = ActionServer(
            self, ExecuteManipulation, self._action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f'Fixed manipulation server ready on {self._action_name}; '
            f'dry_run={self._dry_run}, dynamic_ik={self._dynamic_ik}.')

    def _model_states_callback(self, msg):
        self._latest_model_states = msg

    # --------------------------------------------------------
    # Parameter declaration (phases 6,7,8 add params)
    # --------------------------------------------------------

    def _declare_parameters(self):
        defaults = {
            'action_name': '/manipulation/execute',
            'arm_action_name': '/arm_controller/follow_joint_trajectory',
            'gripper_action_name': '/gripper_controller/follow_joint_trajectory',
            'attach_service_name': '/ATTACHLINK',
            'detach_service_name': '/DETACHLINK',
            'get_state_service_name': '/gazebo/get_entity_state',
            'set_state_service_name': '/gazebo/set_entity_state',
            'dependency_timeout_s': 10.0,
            'robot_model': 'six_arm',
            'tool_link': 'link6',
            'default_object': 'red_cube_1',
            'object_link': 'link',
            'arm_joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            'gripper_joints': ['finger_joint1', 'finger_joint2'],
            'arm_home': [0.0, 0.6102, 1.2593, 0.0, -1.4931, 0.0],
            'arm_pick': [0.0, 1.2, 1.17, 0.0, -0.3, 0.0],
            'pregrasp_offset_m': 0.10,
            'dynamic_grasp_ik': True,
            'dynamic_grasp_max_target_distance_m': 0.48,
            'ik_fk_error_tolerance_m': 0.005,
            'fixed_pose_fallback': False,
            'arm_ik_reference_link': 'base_footprint',
            'arm_ik_reference_to_base_z': 0.1634,
            # Keep the 80 mm vertical finger collision bodies above the floor
            # while closing, then lift the attached object before navigation.
            'grasp_vertical_clearance_m': 0.018,
            'object_half_height_m': 0.015,
            'carried_object_floor_clearance_m': 0.006,
            'post_pick_lift_enabled': True,
            'post_pick_lift_settle_time_s': 0.3,
            'arm_pregrasp': [0.0, 1.2, 1.27, 0.0, -0.3, 0.0],
            'arm_lift': [0.0, 0.6102, 1.2593, 0.0, -1.4931, 0.0],
            # Keep the attached cube about 0.10 m above the floor before the
            # one-way descent.  The former [0,1.2,1.27,0,-0.3,0] endpoint
            # put the gripper centre at only 0.017 m world height and drove
            # the cube into the floor, transmitting the impact to the base.
            'arm_preplace': [0.0, 0.8154, 1.3923, 0.0, -0.4636, 0.0],
            'arm_place': [0.0, 1.2, 1.17, 0.0, -0.3, 0.0],
            # Both finger joints start on opposite sides and use opposite
            # positive axes.  Increasing both positions moves both fingers
            # toward the centre, so zero is open and a positive pair closes.
            'gripper_open': [0.0, 0.0],
            # With 75 mm open centre separation and 10 mm finger thickness,
            # 18 mm travel per side gives a ~29 mm inner gap for a 30 mm cube.
            'gripper_closed': [0.018, 0.018],
            # A square cube presents a wider cross-section when its yaw is
            # not aligned with the fingers.  Compute the closing travel from
            # that live projection instead of always crushing toward 29 mm.
            'object_width_m': 0.030,
            'gripper_open_inner_gap_m': 0.065,
            'gripper_contact_compression_m': 0.001,
            # Phase-8: per-segment durations
            'arm_home_to_pregrasp_duration_s': 1.3,
            'arm_pregrasp_to_grasp_duration_s': 1.3,
            'arm_return_home_duration_s': 1.2,
            'arm_motion_duration_s': 1.5,
            'gripper_open_duration_s': 0.3,
            'gripper_close_duration_s': 0.35,
            'gripper_motion_duration_s': 1.5,
            'settle_time_s': 0.03,
            'pre_close_settle_time_s': 0.12,
            'validate_grasp_window': True,
            'grasp_target_z': 0.045,
            'grasp_xy_tolerance': 0.018,
            # The fingers are long in tool x but narrow in tool y.  Keep the
            # lateral jaw-centering check strict while allowing harmless
            # fore/aft seating produced by closing against floor friction.
            'grasp_forward_tolerance_m': 0.036,
            'grasp_z_min': 0.027,
            'grasp_z_max': 0.063,
            'attached_drift_tolerance': 0.01,
            'carry_relative_tolerance_m': 0.08,
            'attachment_validation_samples': 3,
            'attachment_validation_interval_s': 0.1,
            'attachment_settle_time_s': 0.1,
            # Phase-4: release order & placement zone
            'release_order': 'open_then_detach',
            'placement_zone_models': ['zone_a', 'zone_b', 'zone_c'],
            'placement_zone_half_width': 0.50,
            'placement_zone_half_depth': 0.25,
            'placement_boundary_margin': 0.025,
            'placement_slot_clearance': 0.055,
            'placement_settle_time_s': 0.2,
            'placement_validation_attempts': 3,
            'placement_speed_tolerance_mps': 0.12,
            # A successful x/y score is not sufficient: reject a release if
            # the cube is already penetrating the floor or the chassis has
            # been levered upward by the attached payload.
            'pre_release_object_z_min_m': 0.015,
            'pre_release_object_z_max_m': 0.035,
            'pre_release_max_tilt_rad': 0.12,
            'pre_release_robot_max_z_m': 0.003,
            'pre_release_robot_max_tilt_rad': 0.05,
            'placed_object_z_min_m': 0.012,
            'placed_object_z_max_m': 0.020,
            'placed_object_max_tilt_rad': 0.12,
            # Release a few millimetres above the 15 mm resting centre.  The
            # final arm joint target is solved once from the live attachment
            # offset, because individual grasps seat at different tool z.
            'place_target_object_z_m': 0.0215,
            'preplace_settle_time_s': 0.12,
            'place_pose_settle_time_s': 0.20,
            # Phase-7: retry limits
            'pick_max_attempts': 2,
            'ik_max_attempts': 1,
            'attach_max_attempts': 1,
            'retry_delay_s': 0.5,
            # Phase-8: velocity scaling
            'velocity_scaling': 1.0,
            'acceleration_scaling': 1.0,
            'dry_run': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    # --------------------------------------------------------
    # Parameter loading
    # --------------------------------------------------------

    def _load_parameters(self):
        value = lambda name: self.get_parameter(name).value
        self._action_name = str(value('action_name'))
        self._arm_action_name = str(value('arm_action_name'))
        self._gripper_action_name = str(value('gripper_action_name'))
        self._attach_service_name = str(value('attach_service_name'))
        self._detach_service_name = str(value('detach_service_name'))
        self._get_state_service_name = str(value('get_state_service_name'))
        self._set_state_service_name = str(value('set_state_service_name'))
        self._dependency_timeout = float(value('dependency_timeout_s'))
        self._robot_model = str(value('robot_model'))
        self._tool_link = str(value('tool_link'))
        self._default_object = str(value('default_object'))
        self._object_link = str(value('object_link'))
        self._arm_joints = list(value('arm_joints'))
        self._gripper_joints = list(value('gripper_joints'))
        self._arm_home = list(value('arm_home'))
        self._arm_pick = list(value('arm_pick'))
        self._arm_pregrasp = list(value('arm_pregrasp'))
        self._arm_lift = list(value('arm_lift'))
        self._arm_preplace = list(value('arm_preplace'))
        self._arm_place = list(value('arm_place'))
        self._gripper_open = list(value('gripper_open'))
        self._gripper_closed = list(value('gripper_closed'))
        self._object_width = float(value('object_width_m'))
        self._gripper_open_inner_gap = float(
            value('gripper_open_inner_gap_m'))
        self._gripper_contact_compression = float(
            value('gripper_contact_compression_m'))
        # Phase-8: per-segment durations
        self._dur_home_to_pregrasp = float(
            value('arm_home_to_pregrasp_duration_s'))
        self._dur_pregrasp_to_grasp = float(
            value('arm_pregrasp_to_grasp_duration_s'))
        self._dur_return_home = float(
            value('arm_return_home_duration_s'))
        self._arm_duration = float(value('arm_motion_duration_s'))
        self._gripper_open_dur = float(
            value('gripper_open_duration_s'))
        self._gripper_close_dur = float(
            value('gripper_close_duration_s'))
        self._gripper_duration = float(
            value('gripper_motion_duration_s'))
        self._settle_time = float(value('settle_time_s'))
        self._pre_close_settle_time = float(value('pre_close_settle_time_s'))
        self._validate_window = bool(value('validate_grasp_window'))
        self._grasp_target_z = float(value('grasp_target_z'))
        self._grasp_xy_tolerance = float(
            value('grasp_xy_tolerance'))
        self._grasp_forward_tolerance = float(
            value('grasp_forward_tolerance_m'))
        self._grasp_z_min = float(value('grasp_z_min'))
        self._grasp_z_max = float(value('grasp_z_max'))
        self._attached_drift_tolerance = float(
            value('attached_drift_tolerance'))
        self._carry_relative_tolerance = float(
            value('carry_relative_tolerance_m'))
        self._validation_samples = int(
            value('attachment_validation_samples'))
        self._validation_interval = float(
            value('attachment_validation_interval_s'))
        self._settle_time_attach = float(
            value('attachment_settle_time_s'))
        # Phase-4: release order & placement zone
        self._release_order = str(value('release_order'))
        self._placement_zone_models = list(
            value('placement_zone_models'))
        self._zone_hw = float(value('placement_zone_half_width'))
        self._zone_hd = float(value('placement_zone_half_depth'))
        self._zone_margin = float(value('placement_boundary_margin'))
        self._slot_clearance = float(value('placement_slot_clearance'))
        self._placement_settle = float(value('placement_settle_time_s'))
        self._placement_val_attempts = int(
            value('placement_validation_attempts'))
        self._placement_speed_tolerance = float(
            value('placement_speed_tolerance_mps'))
        self._pre_release_z_min = float(value('pre_release_object_z_min_m'))
        self._pre_release_z_max = float(value('pre_release_object_z_max_m'))
        self._pre_release_max_tilt = float(value('pre_release_max_tilt_rad'))
        self._pre_release_robot_max_z = float(
            value('pre_release_robot_max_z_m'))
        self._pre_release_robot_max_tilt = float(
            value('pre_release_robot_max_tilt_rad'))
        self._placed_z_min = float(value('placed_object_z_min_m'))
        self._placed_z_max = float(value('placed_object_z_max_m'))
        self._placed_max_tilt = float(value('placed_object_max_tilt_rad'))
        self._place_target_object_z = float(value('place_target_object_z_m'))
        self._preplace_settle_time = float(
            value('preplace_settle_time_s'))
        self._place_pose_settle_time = float(
            value('place_pose_settle_time_s'))
        # Phase-6: IK params
        self._pregrasp_offset = float(
            value('pregrasp_offset_m'))
        self._dynamic_ik = bool(value('dynamic_grasp_ik'))
        self._max_ik_distance = float(
            value('dynamic_grasp_max_target_distance_m'))
        self._ik_fk_tol = float(
            value('ik_fk_error_tolerance_m'))
        self._fallback_enabled = bool(
            value('fixed_pose_fallback'))
        self._arm_ik_reference_link = str(
            value('arm_ik_reference_link'))
        self._arm_ik_reference_to_base_z = float(
            value('arm_ik_reference_to_base_z'))
        self._grasp_vertical_clearance = float(
            value('grasp_vertical_clearance_m'))
        self._object_half_height = float(value('object_half_height_m'))
        self._carried_floor_clearance = float(
            value('carried_object_floor_clearance_m'))
        self._post_pick_lift_enabled = bool(
            value('post_pick_lift_enabled'))
        self._post_pick_lift_settle = float(
            value('post_pick_lift_settle_time_s'))
        # Phase-7: retry
        self._pick_max = int(value('pick_max_attempts'))
        self._ik_max = int(value('ik_max_attempts'))
        self._attach_max = int(value('attach_max_attempts'))
        self._retry_delay = float(value('retry_delay_s'))
        # Phase-8: velocity scaling
        self._vel_scale = max(0.01, float(
            value('velocity_scaling')))
        self._acc_scale = max(0.01, float(
            value('acceleration_scaling')))
        self._dry_run = bool(value('dry_run'))
        self._validate_configuration()

    # --------------------------------------------------------
    # Configuration validation
    # --------------------------------------------------------

    def _validate_configuration(self):
        vectors = (
            ('arm_home', self._arm_home, self._arm_joints),
            ('arm_pick', self._arm_pick, self._arm_joints),
            ('arm_pregrasp', self._arm_pregrasp, self._arm_joints),
            ('arm_lift', self._arm_lift, self._arm_joints),
            ('arm_preplace', self._arm_preplace, self._arm_joints),
            ('arm_place', self._arm_place, self._arm_joints),
            ('gripper_open', self._gripper_open, self._gripper_joints),
            ('gripper_closed', self._gripper_closed, self._gripper_joints),
        )
        for name, positions, joints in vectors:
            if len(positions) != len(joints):
                raise ValueError(
                    f'{name} has {len(positions)} values '
                    f'but requires {len(joints)}.')
        if self._arm_duration <= 0.0 or self._gripper_duration <= 0.0:
            raise ValueError(
                'Trajectory durations must be positive.')
        if self._grasp_xy_tolerance <= 0.0:
            raise ValueError('grasp_xy_tolerance must be positive.')
        if not self._grasp_xy_tolerance <= self._grasp_forward_tolerance <= 0.04:
            raise ValueError('grasp_forward_tolerance_m is outside safe range.')
        if self._grasp_z_min >= self._grasp_z_max:
            raise ValueError(
                'grasp_z_min must be less than grasp_z_max.')
        if self._object_width <= 0.0:
            raise ValueError('object_width_m must be positive.')
        if self._gripper_open_inner_gap <= self._object_width:
            raise ValueError(
                'gripper_open_inner_gap_m must exceed object_width_m.')
        if not 0.0 <= self._gripper_contact_compression < 0.005:
            raise ValueError(
                'gripper_contact_compression_m must be in [0, 0.005).')
        if self._validation_samples < 3:
            raise ValueError(
                'attachment_validation_samples must be at least 3.')
        if self._validation_interval <= 0.0:
            raise ValueError(
                'attachment_validation_interval_s must be positive.')
        if self._post_pick_lift_settle < 0.0:
            raise ValueError(
                'post_pick_lift_settle_time_s must not be negative.')
        if not 0.05 <= self._max_ik_distance <= 0.80:
            raise ValueError(
                'dynamic_grasp_max_target_distance_m must be 0.05-0.80.')
        if self._carry_relative_tolerance <= 0.0:
            raise ValueError('carry_relative_tolerance_m must be positive.')
        if self._release_order not in (
                'open_then_detach', 'detach_then_open'):
            raise ValueError(
                f'Invalid release_order: {self._release_order}')
        if not self._pre_release_z_min < self._pre_release_z_max:
            raise ValueError('pre-release z limits are invalid.')
        if not self._placed_z_min < self._placed_z_max:
            raise ValueError('placed-object z limits are invalid.')
        if not self._placed_z_min <= self._place_target_object_z <= 0.035:
            raise ValueError('place_target_object_z_m is outside safe range.')
        if self._pick_max < 1 or self._pick_max > 3:
            raise ValueError('pick_max_attempts must be 1-3')
        if self._ik_max < 0 or self._ik_max > 2:
            raise ValueError('ik_max_attempts must be 0-2')
        if self._attach_max < 0 or self._attach_max > 2:
            raise ValueError('attach_max_attempts must be 0-2')

    # --------------------------------------------------------
    # Goal / Cancel callbacks
    # --------------------------------------------------------

    def _goal_callback(self, goal):
        operation = goal.operation.strip().lower()
        object_id = goal.object_id.strip() or self._default_object
        if self._busy:
            self.get_logger().warning(
                'Rejecting manipulation goal: server is busy.')
            return GoalResponse.REJECT
        if operation not in self.SUPPORTED_OPERATIONS:
            self.get_logger().warning(
                f'Rejecting unsupported operation: {operation}.')
            return GoalResponse.REJECT
        if not re.fullmatch(r'(red|blue)_cube_[1-9][0-9]*', object_id):
            self.get_logger().warning(
                f'Rejecting invalid object id: {object_id}.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    # --------------------------------------------------------
    # Top-level execute wrapper
    # --------------------------------------------------------

    async def _execute(self, goal_handle):
        self._busy = True
        operation = goal_handle.request.operation.strip().lower()
        object_id = goal_handle.request.object_id.strip()             or self._default_object
        try:
            if self._dry_run:
                await self._execute_dry_run(
                    goal_handle, operation, object_id)
            else:
                self._wait_for_dependencies(operation)
                await self._execute_operation(
                    goal_handle, operation, object_id)
            goal_handle.succeed()
            return self._result(True, self.SUCCESS,
                f'{operation} completed for {object_id}.')
        except ManipulationError as error:
            if self._carried_object_id is None:
                self._state = ArmState.IDLE
            if error.code == self.CANCELLED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return self._result(False, error.code, str(error))
        except Exception as error:
            if self._carried_object_id is None:
                self._state = ArmState.ERROR
            self.get_logger().error(
                f'Unexpected manipulation failure: {error}')
            goal_handle.abort()
            return self._result(
                False, self.INTERNAL_ERROR, str(error))
        finally:
            self._busy = False

    # --------------------------------------------------------
    # Operation dispatcher
    # --------------------------------------------------------

    async def _execute_operation(self, handle, operation, object_id):
        if operation == 'pick':
            await self._pick(handle, object_id)
        elif operation == 'place':
            await self._place(handle, object_id)
        elif operation == 'home':
            await self._move_arm_once(
                handle, 'return_home', self._arm_home)
        elif operation == 'pick_pose':
            await self._move_arm_once(
                handle, 'move_to_pick_pose', self._arm_pick)
        elif operation == 'lift':
            await self._move_arm_once(
                handle, 'lift_object', self._arm_lift)
        elif operation == 'place_pose':
            await self._move_arm_once(
                handle, 'move_to_place_pose', self._arm_place)
        elif operation == 'open':
            await self._move_gripper_once(
                handle, 'open_gripper', self._gripper_open)
        elif operation == 'close':
            await self._move_gripper_once(
                handle, 'close_gripper', self._gripper_closed)

    # --------------------------------------------------------
    # Low-level trajectory helpers (phase-8 uses per-segment dur)
    # --------------------------------------------------------

    async def _move_arm_once(self, handle, stage, positions):
        await self._stage(handle, stage, 0.25)
        duration = (
            self._dur_return_home if stage == 'return_home'
            else self._arm_duration)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, positions,
            duration, self.ARM_FAILED,
            stage.replace('_', ' '))
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
        if operation in ('pick', 'place', 'home', 'pick_pose',
                         'lift', 'place_pose'):
            checks.append((self._arm_action_name,
                self._arm_client.wait_for_server(
                    timeout_sec=self._dependency_timeout)))
        if operation in ('pick', 'place', 'open', 'close'):
            checks.append((self._gripper_action_name,
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
        if operation == 'place':
            checks.append((self._set_state_service_name,
                self._set_state_client.wait_for_service(
                    timeout_sec=self._dependency_timeout)))
        if operation == 'pick' and self._validate_window:
            checks.append((self._get_state_service_name,
                self._state_client.wait_for_service(
                    timeout_sec=self._dependency_timeout)))
        unavailable = [name for name, ready in checks if not ready]
        if unavailable:
            raise ManipulationError(
                self.DEPENDENCY_UNAVAILABLE,
                f'Required interfaces are unavailable: {unavailable}.')

    # --------------------------------------------------------
    # Phase-3: Carry health check (background timer)
    # --------------------------------------------------------

    def _carry_health_check_callback(self):
        if (self._state != ArmState.CARRY_HOLD
                or self._carried_object_id is None):
            self._carry_health_failure_count = 0
            status_msg = String()
            status_msg.data = json.dumps({
                'state': self._state,
                'object_id': self._carried_object_id or '',
                'healthy': self._state != ArmState.ERROR,
                'failure_count': 0,
            }, ensure_ascii=False)
            self._carry_status_publisher.publish(status_msg)
            return
        try:
            healthy = self._check_carry_health()
        except Exception as exc:
            self.get_logger().debug(
                f'Carry health exception: {exc}')
            healthy = False
        if healthy:
            self._carry_health_failure_count = 0
        else:
            self._carry_health_failure_count += 1
        status_msg = String()
        status_msg.data = json.dumps({
            'state': self._state,
            'object_id': self._carried_object_id or '',
            'healthy': healthy
                and self._carry_health_failure_count < 3,
            'failure_count': self._carry_health_failure_count,
        }, ensure_ascii=False)
        self._carry_status_publisher.publish(status_msg)

    def _check_carry_health(self):
        msg = self._latest_model_states
        if msg is None or self._carry_reference_offset is None:
            return False
        try:
            obj_idx = msg.name.index(self._carried_object_id)
        except ValueError:
            return False
        try:
            base_idx = msg.name.index(self._robot_model)
        except ValueError:
            return False
        current = self._point_in_pose_frame(
            msg.pose[obj_idx].position, msg.pose[base_idx])
        drift = math.sqrt(sum(
            (current[i] - self._carry_reference_offset[i]) ** 2
            for i in range(3)))
        return drift <= self._carry_relative_tolerance

    def _capture_carry_reference(self, object_id):
        msg = self._latest_model_states
        if msg is None:
            return None
        try:
            obj_idx = msg.name.index(object_id)
            base_idx = msg.name.index(self._robot_model)
        except ValueError:
            return None
        return self._point_in_pose_frame(
            msg.pose[obj_idx].position, msg.pose[base_idx])

    @staticmethod
    def _point_in_pose_frame(point, frame_pose):
        """Express a world point in frame_pose, including frame rotation."""
        vx = float(point.x - frame_pose.position.x)
        vy = float(point.y - frame_pose.position.y)
        vz = float(point.z - frame_pose.position.z)
        q = frame_pose.orientation
        # Rotate with the inverse unit quaternion. Normalize defensively.
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-9:
            return (vx, vy, vz)
        x, y, z, w = -q.x/norm, -q.y/norm, -q.z/norm, q.w/norm
        tx = 2.0 * (y*vz - z*vy)
        ty = 2.0 * (z*vx - x*vz)
        tz = 2.0 * (x*vy - y*vx)
        return (
            vx + w*tx + (y*tz - z*ty),
            vy + w*ty + (z*tx - x*tz),
            vz + w*tz + (x*ty - y*tx),
        )

    async def _pick(self, handle, object_id):
        max_attempts = self._pick_max
        last_error = None
        for attempt in range(1, max_attempts + 1):
            was_detached = False
            try:
                if attempt > 1:
                    self.get_logger().info(
                        f'Retry #{attempt}/{max_attempts} '
                        f'for {object_id}.')
                    await self._stage(handle,
                        f'retry_attempt_{attempt}', 0.00)
                await self._do_pick_once(handle, object_id)
                return
            except ManipulationError as exc:
                last_error = exc
                if exc.code in (
                        self.ARM_FAILED, self.GRIPPER_FAILED,
                        self.DEPENDENCY_UNAVAILABLE,
                        self.CANCELLED, self.INVALID_GOAL):
                    raise
                if exc.code == self.ATTACH_FAILED and not was_detached:
                    await self._rollback_attach()
                    was_detached = True
                if exc.code not in (
                        self.GRASP_WINDOW_FAILED, self.ATTACH_FAILED):
                    raise
                if self._carried_object_id is not None:
                    raise
            except Exception:
                if self._carried_object_id is not None:
                    raise
            if self._pending_pick is not None:
                self._pending_pick = None
            if self._last_attach is not None:
                self._last_attach = None
            if attempt < max_attempts:
                self.get_logger().info(
                    f'Waiting {self._retry_delay:.1f}s before pick retry.')
                await self._sleep(self._retry_delay)
                try:
                    await self._send_trajectory_safe(
                        self._arm_client, self._arm_joints, self._arm_home,
                        self._dur_home_to_pregrasp)
                except Exception:
                    pass
        self.get_logger().error(
            f'Pick failed after {max_attempts} attempts: {last_error}')
        raise last_error or ManipulationError(
            self.ARM_FAILED, 'Pick exhausted all retries.')

    async def _do_pick_once(self, handle, object_id):
        # Step 0: check carried object
        if self._carried_object_id is not None:
            raise ManipulationError(
                self.ARM_FAILED,
                f'Already carrying {self._carried_object_id}; '
                f'cannot start new pick.')
        # Step 1: open gripper
        self._log_stage_start('open')
        await self._stage(handle, 'open_gripper', 0.00)
        await self._send_trajectory(
            self._gripper_client, self._gripper_joints,
            self._gripper_open, self._gripper_open_dur,
            self.GRIPPER_FAILED, 'open gripper')
        self._log_stage_end('open')
        # Step 2: move to pre-grasp (IK-driven or configured backup)
        self._log_stage_start('move_to_pregrasp')
        pregrasp_pose = await self._compute_pregrasp_for(
            handle, object_id)
        await self._stage(handle, 'move_to_pregrasp', 0.10)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, pregrasp_pose,
            self._dur_home_to_pregrasp, self.ARM_FAILED,
            'move arm to pre-grasp')
        self._log_stage_end('move_to_pregrasp')
        # Step 3: descend from pre-grasp to grasp
        self._log_stage_start('descend_to_pick_pose')
        await self._stage(handle, 'descend_to_pick_pose', 0.25)
        grasp_pose = await self._compute_grasp_for(object_id)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, grasp_pose,
            self._dur_pregrasp_to_grasp, self.ARM_FAILED,
            'descend to pick pose')
        self._log_stage_end('descend_to_pick_pose')
        await self._stage(handle, 'stabilize_before_close', 0.35, settle=False)
        await self._sleep(self._pre_close_settle_time)
        # Step 4: close gripper
        self._log_stage_start('close')
        await self._stage(handle, 'close_gripper', 0.40)
        close_positions = await self._close_positions_for(object_id)
        await self._send_trajectory(
            self._gripper_client, self._gripper_joints,
            close_positions, self._gripper_close_dur,
            self.GRIPPER_FAILED, 'close gripper')
        self._log_stage_end('close')
        # Step 5: validate grasp window
        await self._stage(
            handle, 'validate_grasp_window', 0.50, settle=False)
        await self._validate_grasp_pose(object_id)
        # Seat the physically closed gripper around the cube with a few
        # millimetres of clearance.  This avoids creating a rigid
        # LinkAttacher joint while the object is constrained by the ground.
        await self._stage(
            handle, 'establish_floor_clearance', 0.58, settle=False)
        # Step 6: attach with retry
        # These two operations must be consecutive.  The 5 g cube falls back
        # to the floor within a few control cycles if any stage delay is left
        # between SetEntityState and LinkAttacher.
        await self._establish_floor_clearance(object_id)
        # SetEntityState must not become a remote "magnet".  Refuse the rigid
        # safety attachment unless the raised cube is still between the jaws.
        await self._validate_grasp_pose(object_id)
        await self._stage(handle, 'attach_object', 0.65, settle=False)
        self._pending_pick = object_id
        self._last_attach = (self._robot_model, self._tool_link,
                             object_id, self._object_link)
        attach_errors = 0
        while True:
            try:
                await self._call_attach(object_id, attach=True)
                break
            except ManipulationError:
                attach_errors += 1
                if attach_errors <= self._attach_max:
                    self.get_logger().warning(
                        f'Attach attempt {attach_errors} failed '
                        f'for {object_id}, retrying...')
                    await self._sleep(self._retry_delay)
                    continue
                raise
        # Settle
        await self._stage(
            handle, 'settling_for_attachment', 0.72, settle=False)
        await self._sleep(self._settle_time_attach)
        # Step 7: static low-position attachment validation
        await self._stage(handle, 'validate_attachment', 0.85)
        try:
            await self._validate_attachment_static(object_id)
        except ManipulationError:
            await self._rollback_attach()
            raise
        # Step 8: lift clear of the floor before navigation.  Validate again
        # after motion and capture the carry reference only at the final pose.
        if self._post_pick_lift_enabled:
            await self._stage(handle, 'lift_attached_object', 0.90)
            try:
                await self._send_trajectory(
                    self._arm_client, self._arm_joints, self._arm_lift,
                    self._dur_return_home, self.ARM_FAILED,
                    'lift attached object')
                await self._sleep(self._post_pick_lift_settle)
                await self._stage(
                    handle, 'validate_lifted_attachment', 0.93)
                await self._validate_attachment_static(object_id)
            except Exception:
                await self._rollback_attach()
                raise
        # Step 9: commit state
        self._state = ArmState.CARRY_HOLD
        self._carried_object_id = object_id
        self._carry_reference_offset = self._capture_carry_reference(object_id)
        if self._carry_reference_offset is None:
            await self._rollback_attach()
            self._carried_object_id = None
            self._state = ArmState.IDLE
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Cannot capture carry reference for {object_id}.')
        self._pending_pick = None
        self._active_grasp_solution = None
        # Step 10: hold + complete
        await self._stage(handle, 'carry_hold', 0.97)
        await self._stage(handle, 'pick_complete', 1.0)

    # --------------------------------------------------------
    # Phase-6: Dynamic IK for pregrasp and grasp
    # --------------------------------------------------------

    async def _compute_pregrasp_for(self, handle, object_id):
        self._state = ArmState.PICKING
        if not self._dynamic_ik:
            return list(self._arm_pregrasp)
        try:
            pregrasp, grasp = await self._solve_dynamic_grasp(object_id)
            self._active_grasp_solution = (object_id, pregrasp, grasp)
            return pregrasp
        except ManipulationError:
            if not self._fallback_enabled:
                raise
            self.get_logger().warning(
                f'Dynamic IK failed for {object_id}; using enabled fixed fallback.')
            self._active_grasp_solution = (
                object_id, list(self._arm_pregrasp), list(self._arm_pick))
            return list(self._arm_pregrasp)

    async def _compute_grasp_for(self, object_id):
        cached = self._active_grasp_solution
        if cached is not None and cached[0] == object_id:
            return list(cached[2])
        if not self._dynamic_ik:
            return list(self._arm_pick)
        pregrasp, grasp = await self._solve_dynamic_grasp(object_id)
        self._active_grasp_solution = (object_id, pregrasp, grasp)
        return grasp

    @staticmethod
    def _arm_fk_point(joints, link6_point):
        """Transform a point fixed in link6 into arm_base_link."""
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
        return (matrix @ np.array((*link6_point, 1.0)))[:3]

    @classmethod
    def _grasp_fk_center(cls, joints):
        """Return link6 + 45 mm fingertip centre in arm_base_link."""
        return cls._arm_fk_point(joints, (0.0, 0.0, 0.045))

    def _solve_grasp_center_ik(self, target, seed):
        joints = np.asarray(seed, dtype=float).copy()
        lower = np.array((-2.30, -2.30, -2.57, -2.30, -2.30, -2.30))
        upper = np.array((2.30, 2.30, 2.57, 2.30, 2.30, 2.30))
        target = np.asarray(target, dtype=float)
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
            augmented_jacobian = np.vstack((jacobian, 0.22 * pitch_axis))
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

    def _solve_link_point_ik(self, target, link6_point, seed):
        """Solve IK for the actual carried-object centre fixed in link6."""
        joints = np.asarray(seed, dtype=float).copy()
        lower = np.array((-2.30, -2.30, -2.57, -2.30, -2.30, -2.30))
        upper = np.array((2.30, 2.30, 2.57, 2.30, 2.30, 2.30))
        target = np.asarray(target, dtype=float)
        point = np.asarray(link6_point, dtype=float)
        pitch_axis = np.array((0.0, 1.0, 1.0, 0.0, -1.0, 0.0))
        reference_pitch = float(pitch_axis @ np.asarray(seed, dtype=float))
        for _ in range(260):
            centre = self._arm_fk_point(joints, point)
            error = target - centre
            if float(np.linalg.norm(error)) <= 0.0015:
                break
            epsilon = 1e-5
            columns = []
            for index in range(6):
                perturbed = joints.copy()
                perturbed[index] += epsilon
                columns.append(
                    (self._arm_fk_point(perturbed, point) - centre) / epsilon)
            jacobian = np.column_stack(columns)
            pitch_error = reference_pitch - float(pitch_axis @ joints)
            augmented_jacobian = np.vstack((jacobian, 0.22 * pitch_axis))
            augmented_error = np.append(error, 0.22 * pitch_error)
            try:
                delta = np.linalg.solve(
                    augmented_jacobian.T @ augmented_jacobian
                    + 0.004 * np.eye(6),
                    augmented_jacobian.T @ augmented_error)
            except np.linalg.LinAlgError:
                break
            length = float(np.linalg.norm(delta))
            if length > 0.055:
                delta *= 0.055 / length
            joints = np.clip(joints + delta, lower, upper)
        residual = float(np.linalg.norm(
            self._arm_fk_point(joints, point) - target))
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
                f'{self._arm_ik_reference_link} for IK.')
        position = response.state.pose.position
        orientation = response.state.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z)
        yaw = math.atan2(sin_yaw, cos_yaw)
        projected_width = self._object_width * (
            abs(math.cos(yaw)) + abs(math.sin(yaw)))
        # A square cube presented near 45 degrees seats higher between the
        # fingers because its projected width is larger.  Raise link6 by that
        # measured excess so the finger tips retain the same floor clearance
        # as a face-on grasp.  This is a one-shot IK correction, not object
        # following or a post-grasp teleport.
        yaw_height_compensation = min(
            0.004, max(0.0, projected_width - self._object_width))
        cube = np.array((
            position.x,
            position.y,
            position.z - self._arm_ik_reference_to_base_z
            + self._grasp_vertical_clearance
            + yaw_height_compensation,
        ), dtype=float)
        distance = float(np.linalg.norm(cube))
        if distance > self._max_ik_distance:
            raise ManipulationError(
                self.GRASP_WINDOW_FAILED,
                f'{object_id} is {distance:.3f}m from arm base; '
                'outside safe dynamic-grasp reach.')
        pick, pick_error = self._solve_grasp_center_ik(cube, self._arm_pick)
        pre_target = cube + np.array((0.0, 0.0, self._pregrasp_offset))
        pregrasp, pre_error = self._solve_grasp_center_ik(pre_target, pick)
        if pick_error > self._ik_fk_tol or pre_error > max(
                self._ik_fk_tol, 0.010):
            raise ManipulationError(
                self.GRASP_WINDOW_FAILED,
                f'Dynamic IK residual too high for {object_id}: '
                f'pick={pick_error:.4f}m, pregrasp={pre_error:.4f}m.')
        self.get_logger().info(
            f'Dynamic IK {object_id}: target='
            f'({cube[0]:.3f},{cube[1]:.3f},{cube[2]:.3f}), '
            f'yaw_height_comp={yaw_height_compensation:.4f}m, '
            f'pick_error={pick_error:.4f}m, pre_error={pre_error:.4f}m.')
        return pregrasp, pick

    # --------------------------------------------------------
    # Rollback, attach/detach helpers, placement validation
    # --------------------------------------------------------

    async def _rollback_attach(self):
        if self._last_attach is not None:
            model1, link1, obj_id, link2 = self._last_attach
            try:
                await self._call_attach_raw(
                    model1, link1, obj_id, link2, attach=False)
                self.get_logger().info(f'Rolled back attachment of {obj_id}.')
            except ManipulationError as exc:
                self.get_logger().error(
                    f'Rollback detach failed for {obj_id}: {exc}')
        self._pending_pick = None
        self._last_attach = None

    async def _call_attach_raw(self, model1, link1, object_id,
                               object_link, attach=True):
        req_type = AttachLink.Request if attach else DetachLink.Request
        request = req_type()
        request.model1_name = model1
        request.link1_name = link1
        request.model2_name = object_id
        request.link2_name = object_link
        client = self._attach_client if attach else self._detach_client
        response = await client.call_async(request)
        if not response.success:
            verb = 'attach' if attach else 'detach'
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Failed to {verb} {object_id}: {response.message}')

    async def _validate_grasp_pose(self, object_id):
        pose = await self._get_relative_pose(object_id)
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z)
        in_window = (abs(x) <= self._grasp_forward_tolerance
                     and abs(y) <= self._grasp_xy_tolerance
                     and self._grasp_z_min <= z <= self._grasp_z_max)
        self.get_logger().info(
            f'{object_id} relative to {self._tool_link}: '
            f'x={x:.4f}, y={y:.4f}, z={z:.4f}, '
            f'in_grasp_window={in_window}')
        if not in_window:
            raise ManipulationError(
                self.GRASP_WINDOW_FAILED,
                f'{object_id} outside grasp window: '
                f'x={x:.4f}, y={y:.4f}, z={z:.4f}.')
        return pose

    async def _close_positions_for(self, object_id):
        """Return a contact-safe symmetric closing travel for a square cube."""
        pose = await self._get_relative_pose(object_id)
        orientation = pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z)
        yaw = math.atan2(sin_yaw, cos_yaw)
        projected_width = self._object_width * (
            abs(math.cos(yaw)) + abs(math.sin(yaw)))
        target_gap = max(
            self._object_width,
            projected_width - self._gripper_contact_compression)
        travel = 0.5 * (self._gripper_open_inner_gap - target_gap)
        max_travel = min(float(value) for value in self._gripper_closed)
        travel = max(0.0, min(max_travel, travel))
        self.get_logger().info(
            f'Yaw-aware gripper close for {object_id}: '
            f'yaw={yaw:.3f}rad, projected_width={projected_width:.3f}m, '
            f'target_gap={target_gap:.3f}m, travel={travel:.4f}m.')
        return [travel for _ in self._gripper_joints]

    async def _validate_attachment_static(self, object_id):
        n = max(3, self._validation_samples)
        interval = max(0.05, self._validation_interval)
        samples = []
        self.get_logger().info(
            f'Starting static attachment validation: '
            f'{n} samples @ {interval:.2f}s interval.')
        for i in range(n):
            await self._sleep(interval)
            pose = await self._get_relative_pose(object_id)
            samples.append((float(pose.position.x),
                            float(pose.position.y),
                            float(pose.position.z)))
            self.get_logger().info(
                f'  sample {i+1}/{n}: '
                f'x={samples[-1][0]:.6f}, '
                f'y={samples[-1][1]:.6f}, '
                f'z={samples[-1][2]:.6f}')
        reference = samples[0]
        max_drift = 0.0
        for s in samples[1:]:
            dx = s[0] - reference[0]
            dy = s[1] - reference[1]
            dz = s[2] - reference[2]
            drift = math.sqrt(dx * dx + dy * dy + dz * dz)
            if drift > max_drift:
                max_drift = drift
        self.get_logger().info(
            f'Static attachment validation: max drift={max_drift:.6f} m '
            f'(tolerance={self._attached_drift_tolerance:.6f} m)')
        if max_drift > self._attached_drift_tolerance:
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Object {object_id} drifted {max_drift:.6f} m during '
                f'static validation (tolerance '
                f'{self._attached_drift_tolerance:.6f} m).')

    async def _get_relative_pose(self, object_id):
        request = GetEntityState.Request()
        request.name = object_id
        request.reference_frame = f'{self._robot_model}::{self._tool_link}'
        response = await self._state_client.call_async(request)
        if not response.success:
            raise ManipulationError(
                self.GRASP_WINDOW_FAILED,
                f'Cannot query {object_id} relative to {request.reference_frame}.')
        return response.state.pose

    async def _call_attach(self, object_id, attach):
        verb = 'attach' if attach else 'detach'
        self._publish_manipulation_event(
            f'{verb}_request', object_id=object_id)
        req_type = AttachLink.Request if attach else DetachLink.Request
        request = req_type()
        request.model1_name = self._robot_model
        request.link1_name = self._tool_link
        request.model2_name = object_id
        request.link2_name = self._object_link
        client = self._attach_client if attach else self._detach_client
        response = await client.call_async(request)
        self._publish_manipulation_event(
            f'{verb}_result', object_id=object_id,
            success=bool(response.success),
            message=str(response.message))
        if not response.success:
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Failed to {verb} {object_id}: {response.message}')

    # --------------------------------------------------------
    # Phase-4: Low-position place flow
    # --------------------------------------------------------

    async def _place(self, handle, object_id):
        if self._carried_object_id is None:
            raise ManipulationError(
                self.INVALID_GOAL,
                'No object carried; cannot place.')
        if self._carried_object_id != object_id:
            raise ManipulationError(
                self.INVALID_GOAL,
                f'Carrying {self._carried_object_id} but requested '
                f'place for {object_id}.')
        # Verify carry health before release
        if not self._check_carry_health():
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Carry health invalid for {object_id}; refusing release.')
        self._log_stage_start('place')
        await self._stage(handle, 'validate_carried_object', 0.05)
        self._state = ArmState.PLACING

        # Mirror the proven grasp geometry: approach the destination from
        # above, then execute exactly one short descent.  The robot base is
        # already parked and facing the zone, so no diagonal arm sweep or
        # repeated correction is allowed here.
        await self._stage(handle, 'move_to_preplace_pose', 0.15)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, self._arm_preplace,
            self._dur_home_to_pregrasp, self.ARM_FAILED,
            'move arm to pre-place pose')
        await self._sleep(self._preplace_settle_time)
        # The navigation carry check is expressed in the robot base frame.
        # It must not be reused after an intentional arm motion because the
        # attached cube is then expected to move relative to the base.  Once
        # parked, verify the physical attachment in the tool frame instead.
        await self._validate_attachment_static(object_id)

        place_pose = await self._compute_place_pose_for(object_id)
        await self._stage(handle, 'descend_to_place_pose', 0.35)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, place_pose,
            self._dur_pregrasp_to_grasp, self.ARM_FAILED,
            'descend once to place pose')
        await self._sleep(self._place_pose_settle_time)
        await self._validate_attachment_static(object_id)

        # Validate only after the physical cube has reached the final pose.
        # A failure leaves the attachment and closed gripper intact.
        await self._stage(handle, 'validate_placement_zone', 0.45)
        zone_ok = self._is_in_valid_placement_zone(object_id)
        if not zone_ok:
            self.get_logger().error('Target zone invalid; keeping object.')
            raise ManipulationError(
                self.INVALID_GOAL,
                f'Target zone invalid for {object_id}; holding object.')
        if not self._release_geometry_safe(object_id):
            raise ManipulationError(
                self.INVALID_GOAL,
                f'Unsafe release geometry for {object_id}; holding object.')
        # Execute release sequence based on config
        await self._stage(handle, 'prepare_release', 0.48)
        if self._release_order == 'open_then_detach':
            await self._place_open_then_detach(handle, object_id)
        else:
            await self._place_detach_then_open(handle, object_id)
        # Physical ownership ended after DETACH. Never pretend the object is
        # still carried if subsequent placement validation fails.
        self._carried_object_id = None
        self._carry_reference_offset = None
        self._last_attach = None
        # Settle and verify placement
        await self._stage(handle, 'settle_object', 0.90)
        await self._sleep(self._placement_settle)
        await self._stage(handle, 'validate_placement', 0.95)
        placed_ok = await self._verify_placement_validated(object_id)
        if not placed_ok:
            self._state = ArmState.ERROR
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Placement validation failed for {object_id}.')
        await self._stage(handle, 'return_home', 0.98)
        await self._move_arm_once(handle, 'return_home', self._arm_home)
        self._log_stage_end('place')
        self._state = ArmState.IDLE
        await self._stage(handle, 'place_complete', 1.0)

    async def _place_open_then_detach(self, handle, object_id):
        self._log_stage_start('open_gripper')
        await self._stage(handle, 'open_gripper', 0.50)
        await self._send_trajectory_safe(
            self._gripper_client, self._gripper_joints, self._gripper_open,
            self._gripper_open_dur)
        self._log_stage_end('open_gripper')
        self._log_stage_start('detach_object')
        await self._stage(handle, 'detach_object', 0.70)
        try:
            await self._call_attach(object_id, attach=False)
        except ManipulationError as exc:
            self.get_logger().error(f'DETACH failed: {exc}')
            raise
        await self._stabilize_released_object(object_id)
        self._log_stage_end('detach_object')

    async def _place_detach_then_open(self, handle, object_id):
        self._log_stage_start('detach_object')
        await self._stage(handle, 'detach_object', 0.50)
        try:
            await self._call_attach(object_id, attach=False)
        except ManipulationError as exc:
            self.get_logger().error(f'DETACH failed: {exc}')
            raise
        await self._stabilize_released_object(object_id)
        self._log_stage_end('detach_object')
        self._log_stage_start('open_gripper')
        await self._stage(handle, 'open_gripper', 0.70)
        await self._send_trajectory_safe(
            self._gripper_client, self._gripper_joints, self._gripper_open,
            self._gripper_open_dur)
        self._log_stage_end('open_gripper')

    def _is_in_valid_placement_zone(self, object_id):
        msg = self._latest_model_states
        if msg is None:
            return False
        try:
            obj_idx = msg.name.index(object_id)
        except ValueError:
            return False
        best = None
        for zone in self._placement_zone_models:
            try:
                zone_idx = msg.name.index(zone)
            except ValueError:
                continue
            local = self._point_in_pose_frame(
                msg.pose[obj_idx].position, msg.pose[zone_idx])
            distance = math.hypot(local[0], local[1])
            if best is None or distance < best[0]:
                best = (distance, zone, local)
        if best is None:
            return False
        _, zone, local = best
        x_limit = self._zone_hw - self._zone_margin
        y_limit = self._zone_hd - self._zone_margin
        valid = abs(local[0]) <= x_limit and abs(local[1]) <= y_limit
        if valid:
            self._active_placement_zone = zone
        else:
            self.get_logger().warning(
                f'{object_id} outside zones; nearest={zone}, '
                f'local=({local[0]:.3f},{local[1]:.3f}).')
        return valid

    async def _compute_place_pose_for(self, object_id):
        """Solve one descent for the actual object centre fixed in link6."""
        msg = self._latest_model_states
        if msg is None:
            raise ManipulationError(
                self.ARM_FAILED, 'No Gazebo state for dynamic placement IK.')
        try:
            object_pose = msg.pose[msg.name.index(object_id)]
            _robot_pose = msg.pose[msg.name.index(self._robot_model)]
        except ValueError as exc:
            raise ManipulationError(
                self.ARM_FAILED,
                f'Missing model for dynamic placement IK: {exc}.')

        relative = await self._get_relative_pose(object_id)
        link6_point = (
            float(relative.position.x),
            float(relative.position.y),
            float(relative.position.z))
        current_object = self._arm_fk_point(
            self._arm_preplace, link6_point)
        height_delta = (
            self._place_target_object_z - float(object_pose.position.z))
        target = (
            float(current_object[0]),
            float(current_object[1]),
            float(current_object[2]) + height_delta)
        solution, residual = self._solve_link_point_ik(
            target, link6_point, self._arm_preplace)
        if residual > self._ik_fk_tol:
            raise ManipulationError(
                self.ARM_FAILED,
                f'Dynamic placement IK residual too high: {residual:.4f}m.')
        self.get_logger().info(
            f'Dynamic placement IK {object_id}: '
            f'link6_object=({link6_point[0]:.4f},'
            f'{link6_point[1]:.4f},{link6_point[2]:.4f})m, '
            f'object_target_z={self._place_target_object_z:.4f}m, '
            f'height_delta={height_delta:.4f}m, '
            f'residual={residual:.4f}m.')
        return solution

    @staticmethod
    def _pose_roll_pitch(pose):
        q = pose.orientation
        roll = math.atan2(
            2.0 * (q.w*q.x + q.y*q.z),
            1.0 - 2.0 * (q.x*q.x + q.y*q.y))
        pitch = math.asin(max(-1.0, min(1.0,
            2.0 * (q.w*q.y - q.z*q.x))))
        return roll, pitch

    def _release_geometry_safe(self, object_id):
        """Verify payload and chassis geometry before physical ownership ends."""
        msg = self._latest_model_states
        if msg is None:
            return False
        try:
            object_index = msg.name.index(object_id)
            robot_index = msg.name.index(self._robot_model)
        except ValueError:
            return False
        object_pose = msg.pose[object_index]
        robot_pose = msg.pose[robot_index]
        object_roll, object_pitch = self._pose_roll_pitch(object_pose)
        robot_roll, robot_pitch = self._pose_roll_pitch(robot_pose)
        object_tilt = max(abs(object_roll), abs(object_pitch))
        robot_tilt = max(abs(robot_roll), abs(robot_pitch))
        safe = (
            self._pre_release_z_min <= object_pose.position.z
            <= self._pre_release_z_max
            and object_tilt <= self._pre_release_max_tilt
            and robot_pose.position.z <= self._pre_release_robot_max_z
            and robot_tilt <= self._pre_release_robot_max_tilt)
        self.get_logger().info(
            f'Pre-release 3D geometry for {object_id}: '
            f'object_z={object_pose.position.z:.4f}m, '
            f'object_tilt={object_tilt:.4f}rad, '
            f'robot_z={robot_pose.position.z:.4f}m, '
            f'robot_tilt={robot_tilt:.4f}rad, safe={safe}')
        return safe

    async def _verify_placement_validated(self, object_id):
        required = max(1, self._placement_val_attempts)
        for sample in range(required):
            in_zone = self._is_in_valid_placement_zone(object_id)
            speed = self._object_speed(object_id)
            surface_ok, z, tilt = self._placed_surface_geometry(object_id)
            if (not in_zone or not surface_ok
                    or speed > self._placement_speed_tolerance):
                self.get_logger().warning(
                    f'Placement sample {sample + 1}/{required} invalid: '
                    f'in_zone={in_zone}, surface_ok={surface_ok}, '
                    f'z={z:.4f}m, tilt={tilt:.4f}rad, '
                    f'speed={speed:.3f}m/s.')
                return False
            await self._sleep(0.10)
        return True

    def _placed_surface_geometry(self, object_id):
        msg = self._latest_model_states
        if msg is None:
            return False, float('nan'), float('nan')
        try:
            pose = msg.pose[msg.name.index(object_id)]
        except ValueError:
            return False, float('nan'), float('nan')
        roll, pitch = self._pose_roll_pitch(pose)
        tilt = max(abs(roll), abs(pitch))
        z = float(pose.position.z)
        return (
            self._placed_z_min <= z <= self._placed_z_max
            and tilt <= self._placed_max_tilt,
            z, tilt)

    def _object_speed(self, object_id):
        msg = self._latest_model_states
        if msg is None:
            return float('inf')
        try:
            index = msg.name.index(object_id)
        except ValueError:
            return float('inf')
        linear = msg.twist[index].linear
        return math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)

    async def _stabilize_released_object(self, object_id):
        """Remove LinkAttacher release impulse while preserving release pose."""
        query = GetEntityState.Request()
        query.name = object_id
        query.reference_frame = 'world'
        current = await self._state_client.call_async(query)
        if not current.success:
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Cannot query released object {object_id}.')
        request = SetEntityState.Request()
        request.state.name = object_id
        request.state.pose = current.state.pose
        request.state.reference_frame = 'world'
        response = await self._set_state_client.call_async(request)
        if not response.success:
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Cannot stabilize released object {object_id}.')

    async def _establish_floor_clearance(self, object_id):
        """Give a closed-gripper object minimal clearance before attachment."""
        query = GetEntityState.Request()
        query.name = object_id
        query.reference_frame = 'world'
        current = await self._state_client.call_async(query)
        if not current.success:
            raise ManipulationError(
                self.GRASP_WINDOW_FAILED,
                f'Cannot query {object_id} before attachment.')
        target_z = self._object_half_height + self._carried_floor_clearance
        if current.state.pose.position.z >= target_z - 0.0005:
            return
        request = SetEntityState.Request()
        request.state.name = object_id
        request.state.pose = current.state.pose
        request.state.pose.position.z = target_z
        request.state.reference_frame = 'world'
        response = await self._set_state_client.call_async(request)
        if not response.success:
            raise ManipulationError(
                self.GRASP_WINDOW_FAILED,
                f'Cannot establish floor clearance for {object_id}.')
        self._publish_manipulation_event(
            'floor_clearance', object_id=object_id,
            target_z=float(target_z),
            clearance_m=float(self._carried_floor_clearance))

    # --------------------------------------------------------
    # Stage feedback, trajectory sending, helpers (phase-8 speed)
    # --------------------------------------------------------

    async def _send_trajectory(self, client, joints, positions,
                               duration_s, error_code, description):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = positions
        effective_duration = duration_s / min(1.0, self._vel_scale)
        if client is self._gripper_client:
            self._publish_manipulation_event(
                'gripper_command', stage=description,
                positions=[float(value) for value in positions],
                duration_s=float(effective_duration))
        point.time_from_start = self._duration(effective_duration)
        goal.trajectory.points = [point]
        goal_handle = await client.send_goal_async(goal)
        if not goal_handle.accepted:
            raise ManipulationError(error_code,
                f'Controller rejected: {description}.')
        wrapped_result = await goal_handle.get_result_async()
        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise ManipulationError(
                error_code,
                f'Controller failed: {description}; '
                f'error_code={result.error_code}.')

    async def _send_trajectory_safe(
            self, client, joints, positions, duration_s):
        # Best-effort trajectory for retry resets.
        try:
            await self._send_trajectory(
                client, joints, positions, duration_s,
                self.GRIPPER_FAILED if client is self._gripper_client
                else self.ARM_FAILED,
                'safe reset move')
        except Exception:
            pass

    async def _stage(self, handle, name, progress, settle=True):
        if handle.is_cancel_requested:
            raise ManipulationError(self.CANCELLED,
                f'Cancelled during {name}.')
        feedback = ExecuteManipulation.Feedback()
        feedback.stage = name
        feedback.progress = float(progress)
        handle.publish_feedback(feedback)
        self._publish_manipulation_event(
            'stage', stage=name, progress=float(progress))
        self.get_logger().info(f'Stage: {name} ({progress:.0%})')
        if settle:
            await self._sleep(0.05 if self._dry_run else self._settle_time)

    def _publish_manipulation_event(self, event, **values):
        """Publish low-rate structured telemetry without changing control."""
        message = String()
        payload = {
            'event': event,
            'state': self._state,
            'object_id': values.pop(
                'object_id', self._carried_object_id or ''),
            'monotonic_time_s': time.monotonic(),
        }
        payload.update(values)
        message.data = json.dumps(payload, ensure_ascii=False)
        self._stage_publisher.publish(message)

    async def _sleep(self, seconds):
        """Wait deterministically without creating transient ROS timers.

        The server uses a four-thread executor, so a bounded wait in this
        action callback leaves subscriptions, services and controller action
        results free to progress on the remaining threads.  Transient timers
        intermittently stopped firing after an attach response, leaving the
        second item stuck forever at ``settling_for_attachment``.
        """
        time.sleep(max(0.0, float(seconds)))

    def _log_stage_start(self, stage_name):
        t = time.monotonic()
        self._stage_times[stage_name] = [t, None]

    def _log_stage_end(self, stage_name):
        if stage_name in self._stage_times:
            end_t = time.monotonic()
            self._stage_times[stage_name][1] = end_t
            elapsed = end_t - self._stage_times[stage_name][0]
            self.get_logger().info(
                f'  [{stage_name}] {elapsed * 1000:.0f}ms')
        if stage_name == 'open':
            self._pick_start_time = time.monotonic()
        elif stage_name == 'place_complete':
            self._place_start_time = 0.0

    @staticmethod
    def _duration(seconds):
        whole = int(seconds)
        return Duration(sec=whole,
            nanosec=int((seconds - whole) * 1_000_000_000))

    @staticmethod
    def _result(success, error_code, message):
        result = ExecuteManipulation.Result()
        result.success = success
        result.error_code = error_code
        result.message = message
        return result

    # --------------------------------------------------------
    # Dry-run fallback
    # --------------------------------------------------------

    async def _execute_dry_run(self, handle, operation, object_id):
        stages = (('validate_goal', 0.10),
                  (f'{operation}_{object_id}', 0.50),
                  (f'{operation}_complete', 1.0))
        for stage, progress in stages:
            await self._stage(handle, stage, progress)


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
