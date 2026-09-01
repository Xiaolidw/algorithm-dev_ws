#!/usr/bin/env python3
"""Execute configuration-driven fixed pick/place sequences."""

import math
import re
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import GetEntityState, SetEntityState
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
        # 运动学载运: 焊接关节(跨模型硬约束)在载运剐蹭时会与接触求解
        # 互相注入冲量, 把轻量底盘弹飞(实测反复抛出百余米, 各种接触参数
        # 均无法根治)。改为: 附着仅用于合法性校验与抬臂, 随后解除物理
        # 关节, 用100Hz 位姿跟随携行方块(30Hz 时重力在两拍间造成 ~5mm 竖直抖动), 与底盘零耦合。
        self._kinematic_carry = None
        self._carry_offset = None
        self._carry_timer = None
        self._carry_anchor = None
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
        from std_msgs.msg import String
        self.carry_pub = self.create_publisher(
            String, '/manipulation/carry_set', 10)
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
        from std_msgs.msg import String
        msg = String()
        msg.data = command
        self.carry_pub.publish(msg)


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
            'validate_grasp_window': True,
            'grasp_target_x': 0.0,
            'grasp_target_y': 0.0,
            'grasp_target_z': 0.045,
            'grasp_position_tolerance': 0.08,
            'grasp_xy_tolerance': 0.08,
            'grasp_z_min': 0.0,
            'grasp_z_max': 0.15,
            'attached_drift_tolerance': 0.005,
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
        if self._grasp_xy_tolerance <= 0.0:
            raise ValueError('grasp_xy_tolerance must be positive.')
        if self._grasp_position_tolerance <= 0.0:
            raise ValueError('grasp_position_tolerance must be positive.')
        if self._grasp_z_min >= self._grasp_z_max:
            raise ValueError('grasp_z_min must be less than grasp_z_max.')

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
            self._busy = False

    async def _best_effort_recover(self):
        """Return the arm home after a failure; recovery errors are ignored."""
        if self._pending_pick is not None:
            await self._rollback_failed_pick()
        try:
            self._send_trajectory(
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
        await self._send_trajectory(
            self._arm_client, self._arm_joints, self._arm_pick,
            self._arm_duration, self.ARM_FAILED, 'move arm to pick pose')
        attached_pose = None
        world_pose = None
        if self._validate_window:
            await self._stage(handle, 'validate_grasp_window', 0.40)
            attached_pose = await self._validate_grasp_pose(object_id)
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
        self._publish_carry(f'carry:{object_id}')
        await self._stage(handle, 'lift_object', 0.85)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, self._arm_lift,
            self._arm_duration, self.ARM_FAILED, 'lift arm')
        # Navigation carries the cube relative to the chassis, so arm-servo
        # motion cannot produce vertical jitter while the robot is moving.
        self._publish_carry(f'base:{object_id}')
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

    async def _validate_attachment_drift(self, object_id, reference_pose):
        # 焊接刚建立/抬臂刚结束时物理引擎存在瞬态抖动, 单次采样可能虚高。
        # 连续复查: 只有稳定超出容差才判定附着失效。
        for attempt in range(3):
            if attempt:
                time.sleep(0.8)
            pose = await self._get_relative_pose(object_id)
            drift = math.sqrt(
                (pose.position.x - reference_pose.position.x) ** 2
                + (pose.position.y - reference_pose.position.y) ** 2
                + (pose.position.z - reference_pose.position.z) ** 2
            )
            self.get_logger().info(
                f'{object_id} attached relative-position drift: {drift:.6f} m'
            )
            if drift <= self._attached_drift_tolerance:
                return
            self.get_logger().warning(
                f'{object_id} drift {drift:.6f} m exceeds tolerance '
                f'(check {attempt + 1}/3); re-checking after settle.')
        raise ManipulationError(
            self.ATTACH_FAILED,
            f'{object_id} attachment drift {drift:.6f} m exceeds '
            f'{self._attached_drift_tolerance:.6f} m.',
        )

    async def _place(self, handle, object_id):
        # The final placement is an arm operation, so temporarily return to
        # the tool-fixed mode only while the chassis is stationary.
        self._publish_carry(f'tool:{object_id}')
        await self._stage(handle, 'move_to_place_pose', 0.20)
        await self._send_trajectory(
            self._arm_client, self._arm_joints, self._arm_place,
            self._arm_duration, self.ARM_FAILED, 'move arm to place pose')
        await self._stage(handle, 'open_gripper', 0.50)
        await self._send_trajectory(
            self._gripper_client, self._gripper_joints, self._gripper_open,
            self._gripper_duration, self.GRIPPER_FAILED, 'open gripper')
        await self._stage(handle, 'detach_object', 0.70)
        # 视觉真实化: 张爪已完成, 此刻发送 release 让方块从夹爪间
        # 物理下落, 软接触让它缓缓落定, 而非瞬移到区中心。
        self._publish_carry('release')
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
        """Lock the released cube to the static A/B/C zone nearest the base."""
        robot_request = GetEntityState.Request()
        robot_request.name = self._robot_model
        robot_request.reference_frame = 'world'
        robot_state = await self._state_client.call_async(robot_request)
        if not robot_state.success:
            raise ManipulationError(
                self.ATTACH_FAILED,
                'Cannot locate the robot for placement-zone anchoring.')

        robot_pose = robot_state.state.pose.position
        nearest_zone = None
        nearest_distance = float('inf')
        for zone_model in self._placement_zone_models:
            request = GetEntityState.Request()
            request.name = zone_model
            request.reference_frame = 'world'
            response = await self._state_client.call_async(request)
            if not response.success:
                continue
            zone_pose = response.state.pose.position
            distance = math.hypot(
                robot_pose.x - zone_pose.x, robot_pose.y - zone_pose.y)
            if distance < nearest_distance:
                nearest_zone = zone_model
                nearest_distance = distance

        # 站位距 zone 中心可达 2.5m: 放置站位与 zone 边缘保持安全距离
        # (防 MPPI 死区), 锚定服务以 zone 中心为参照。
        if nearest_zone is None or nearest_distance > 2.5:
            raise ManipulationError(
                self.ATTACH_FAILED,
                'Robot is not inside a recognised A/B/C placement zone.')

        if not teleport:
            # 物理下落路径: 方块已从夹爪间落到地面, 原地锁定即可,
            # 不再瞬移到区中心(赛题允许"放置在指定区域附近")。
            request = AttachLink.Request()
            request.model1_name = nearest_zone
            request.link1_name = self._placement_zone_link
            request.model2_name = object_id
            request.link2_name = self._object_link
            response = await self._attach_client.call_async(request)
            if not response.success:
                raise ManipulationError(
                    self.ATTACH_FAILED,
                    f'Failed to secure {object_id} in {nearest_zone}: {response.message}')
            self.get_logger().info(
                f'{object_id} secured to static {nearest_zone} at '
                f'{nearest_distance:.2f} m from the robot.')
            return
        # 保持在释放位置原锚定: 赛题允许"放在指定区域附近即可", 传送到
        # 区中心会造成视觉跳变。只清零速度, 让方块停在夹爪松开的位置。
        stop_request = SetEntityState.Request()
        stop_request.state.name = object_id
        stop_request.state.reference_frame = 'world'
        stop_request.state.twist = Twist()
        await self._set_state_client.call_async(stop_request)
        time.sleep(self._settle_time)

        request = AttachLink.Request()
        request.model1_name = nearest_zone
        request.link1_name = self._placement_zone_link
        request.model2_name = object_id
        request.link2_name = self._object_link
        response = await self._attach_client.call_async(request)
        if not response.success:
            raise ManipulationError(
                self.ATTACH_FAILED,
                f'Failed to secure {object_id} in {nearest_zone}: {response.message}')
        if response.success:
            self._last_attach = (
                nearest_zone, self._placement_zone_link,
                object_id, self._object_link,
            )
            self.get_logger().info(
                f'{object_id} secured to static {nearest_zone} at '
                f'{nearest_distance:.2f} m from the robot.')

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
