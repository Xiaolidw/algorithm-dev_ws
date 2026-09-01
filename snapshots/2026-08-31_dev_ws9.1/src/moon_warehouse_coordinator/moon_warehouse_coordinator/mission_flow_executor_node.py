#!/usr/bin/env python3
"""Execute the semantic task queue as a navigation/manipulation flow."""

import json
import math
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.action import DriveOnHeading, Spin
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetEntityState
from moon_warehouse_interfaces.action import ExecuteManipulation
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


class MissionFlowExecutorNode(Node):
    """Drive every planned pickup/drop-off leg through the coordinator."""

    NAVIGATION_STATES = {
        'NAVIGATING_PICKUP',
        'NAVIGATING_DROPOFF',
    }

    MANIPULATION_STATES = {
        'GRASPING',
        'PLACING',
    }

    FAILURE_RESULTS = {
        'ABORTED',
        'FAILED',
        'REJECTED',
        'CANCELED',
    }

    def __init__(self):
        super().__init__('mission_flow_executor_node')
        self.callback_group = ReentrantCallbackGroup()

        self.config = self.load_config()
        self.object_approaches = self.config['object_approaches']
        self.object_preapproaches = self.config.get(
            'object_preapproaches', {}
        )
        self.object_dock_stages = self.config.get(
            'object_dock_stages', {}
        )
        self.destinations = self.config['destinations']
        self.dropoff_transits = self.config.get('dropoff_transits', {})
        self.plan_timeout_sec = float(
            self.config.get('plan_timeout_sec', 45.0)
        )
        self.navigation_timeout_sec = float(
            self.config.get('navigation_timeout_sec', 180.0)
        )
        self.navigation_stall_timeout_sec = float(
            self.config.get('navigation_stall_timeout_sec', 45.0)
        )
        self.navigation_motion_progress_m = float(
            self.config.get('navigation_motion_progress_m', 0.15)
        )
        self.max_navigation_retries = int(
            self.config.get('max_navigation_retries', 1)
        )
        self.max_manipulation_retries = int(
            self.config.get('max_manipulation_retries', 2)
        )
        self.retry_delay_sec = float(
            self.config.get('retry_delay_sec', 2.0)
        )
        # A pre-approach is only a staging waypoint; it must not consume the
        # whole mission when a differential-drive base settles a few cm to
        # the side while already facing the straight docking corridor.  The
        # real PICKUP leg and grasp-window check remain strict.
        self.preapproach_acceptance_m = float(
            self.config.get('preapproach_acceptance_m', 0.15)
        )
        self.pickup_acceptance_m = float(
            self.config.get('pickup_acceptance_m', 0.15)
        )
        # Placement zones are areas, not point poses.  Once the chassis is
        # safely inside the surveyed zone-centre tolerance, waiting for Nav2
        # to perfect the terminal yaw only creates endpoint oscillation and
        # can trip the progress checker while carrying a cube.
        self.dropoff_acceptance_m = float(
            self.config.get('dropoff_acceptance_m', 0.20)
        )
        raw_dropoff_overrides = self.config.get(
            'dropoff_acceptance_by_destination', {})
        self.dropoff_acceptance_by_destination = {
            str(destination).upper(): float(tolerance)
            for destination, tolerance in raw_dropoff_overrides.items()
        } if isinstance(raw_dropoff_overrides, dict) else {}
        self.post_place_departure_m = float(
            self.config.get('post_place_departure_m', 0.50)
        )
        self.post_place_reverse_speed_mps = float(
            self.config.get('post_place_reverse_speed_mps', 0.12)
        )
        self.post_place_min_cube_clearance_m = float(
            self.config.get('post_place_min_cube_clearance_m', 0.48)
        )
        self.post_place_clearance_settle_sec = float(
            self.config.get('post_place_clearance_settle_sec', 0.60)
        )
        self.pickup_yaw_tolerance_rad = float(
            self.config.get('pickup_yaw_tolerance_rad', 0.08)
        )
        self.dropoff_yaw_tolerance_rad = float(
            self.config.get('dropoff_yaw_tolerance_rad', 0.10)
        )
        self.precision_dock_timeout_sec = float(
            self.config.get('precision_dock_timeout_sec', 18.0)
        )
        self.precision_dock_max_speed_mps = float(
            self.config.get('precision_dock_max_speed_mps', 0.14)
        )
        self.latest_base_x = None
        self.latest_base_y = None
        self.latest_base_yaw = None
        self.manipulation_timeout_sec = float(
            self.config.get('manipulation_timeout_sec', 45.0)
        )
        self.max_flow_duration_sec = float(
            self.config.get('max_flow_duration_sec', 295.0)
        )

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.execution_status_publisher = self.create_publisher(
            String,
            '/mission/execution_status',
            status_qos,
        )
        self.navigation_goal_publisher = self.create_publisher(
            PoseStamped,
            '/mission/navigation_goal',
            10,
        )

        self.plan_subscription = self.create_subscription(
            String,
            '/mission/plan',
            self.plan_callback,
            status_qos,
            callback_group=self.callback_group,
        )
        self.mission_status_subscription = self.create_subscription(
            String,
            '/mission/status',
            self.mission_status_callback,
            status_qos,
            callback_group=self.callback_group,
        )
        self.navigation_status_subscription = self.create_subscription(
            String,
            '/mission/navigation_status',
            self.navigation_status_callback,
            status_qos,
            callback_group=self.callback_group,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            20,
            callback_group=self.callback_group,
        )

        self.mission_start_client = self.create_client(
            Trigger,
            '/mission/start',
            callback_group=self.callback_group,
        )
        self.navigation_cancel_client = self.create_client(
            Trigger,
            '/mission/cancel_navigation',
            callback_group=self.callback_group,
        )
        self.local_costmap_clear_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
            callback_group=self.callback_group,
        )
        self.global_costmap_clear_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
            callback_group=self.callback_group,
        )
        self.manipulation_client = ActionClient(
            self,
            ExecuteManipulation,
            '/manipulation/execute',
            callback_group=self.callback_group,
        )
        self.precision_drive_client = ActionClient(
            self, DriveOnHeading, '/drive_on_heading',
            callback_group=self.callback_group)
        self.precision_spin_client = ActionClient(
            self, Spin, '/spin', callback_group=self.callback_group)
        # 流程终止时释放运动学载运的方块(防止孤儿跟随让方块跟着
        # 机器人到处打转)
        self.manipulation_release_client = self.create_client(
            Trigger, '/manipulation/release',
            callback_group=self.callback_group)
        # 控制器就绪探测客户端: 常驻创建, 绝不在回调里销毁(在回调中
        # destroy ActionClient 会与 executor wait set 竞态, 触发
        # "rcl_action_client_t invalid" 使进程崩溃, 实测每次必现)。
        from control_msgs.action import FollowJointTrajectory
        self.arm_probe_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group)
        self.gripper_probe_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            callback_group=self.callback_group)
        # The Nav2 action server exists before bt_navigator becomes ACTIVE.
        # Sending during that lifecycle window is accepted by DDS but rejected
        # by Nav2.  Keep one lifecycle client/timer for the process lifetime and
        # queue a leg until the navigator is genuinely ready.
        self.nav2_state_client = self.create_client(
            GetState, '/bt_navigator/get_state',
            callback_group=self.callback_group)
        self.nav2_state_future = None
        self.nav2_probe_started_mono = None
        self.nav2_ready = False
        self.pending_navigation_leg = None
        self.run_service = self.create_service(
            Trigger,
            '/mission/run_flow',
            self.run_callback,
            callback_group=self.callback_group,
        )
        self.stop_service = self.create_service(
            Trigger,
            '/mission/stop_flow',
            self.stop_callback,
            callback_group=self.callback_group,
        )

        self.watchdog_timer = self.create_timer(
            0.5,
            self.watchdog_callback,
            callback_group=self.callback_group,
        )
        self.nav2_poll_timer = self.create_timer(
            0.5,
            self.nav2_readiness_tick,
            callback_group=self.callback_group,
        )
        self.emergency_stop_until_mono = 0.0
        self.emergency_stop_timer = self.create_timer(
            0.05,
            self.emergency_stop_tick,
            callback_group=self.callback_group,
        )
        self.retry_timer = None
        self.precision_dock_timer = None
        self.post_place_departure_timer = None
        self.post_place_departure_start = None
        self.post_place_departure_started_mono = None
        # ReentrantCallbackGroup can dispatch more than one 20 Hz departure
        # callback before a timer cancellation becomes visible.  Commit the
        # task transition exactly once so a stale callback cannot advance the
        # task index a second time and dispatch two navigation goals.
        self.post_place_completion_committed = False
        self.precision_dock_started_monotonic = None
        self.precision_dock_goal_handle = None
        self.precision_dock_generation = 0
        self.precision_heading_corrections = 0
        self.precision_control_stage = None
        self.precision_settle_count = 0
        self.precision_best_distance = math.inf
        self.precision_last_progress_mono = None
        self.manipulation_generation = 0
        # /mission/start 重发节拍器: 常驻创建一次, 回调里绝不 create/
        # destroy 实体(与 executor wait set 竞态会让进程崩溃, 实测
        # rcl_action_client_t invalid 直接致死)。
        self.mission_start_retry_timer = self.create_timer(
            1.0,
            self.mission_start_retry_tick,
            callback_group=self.callback_group,
        )
        self.mission_start_last_request = 0.0
        # 抓取/放置结果看门狗: 本机 DDS 偶发丢失动作结果回执, 服务器侧
        # 明明已完成(实测 pick_complete 100%)流程却永远卡在 GRASPING,
        # 机器人原地不动。超时后主动查 Gazebo 真值判定成败并续跑。
        from gazebo_msgs.msg import ModelStates
        self.cube_world_z = {}
        self.cube_world_xy = {}
        self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self.model_states_callback, 10)
        self.manipulation_watch_timer = self.create_timer(
            1.0, self.manipulation_watch_tick,
            callback_group=self.callback_group)
        # model_states 订阅在本机 DDS 下偶发收不到数据, 探测时直接查
        # Gazebo 服务兜底; 零速发布用于失败重试前的清场(P4 恢复策略)
        self.gazebo_state_client = self.create_client(
            GetEntityState, '/gazebo/get_entity_state',
            callback_group=self.callback_group)
        self.zero_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        # 最终取件进给不能绕过速度安全链。/cmd_vel 与 collision_monitor
        # 的输出会竞争；由此入口进入会依次经过航向锁、SIPP 限速、平滑器
        # 和碰撞监视器，墙体、方块和移动障碍都保留硬制动。
        self.precision_velocity_publisher = self.create_publisher(
            Twist, '/cmd_vel_nav_raw', 20)
        # Non-blocking runtime cap for the MPPI transport segment.  The
        # request is deliberately best-effort: a controller that rejects a
        # live parameter update must never terminate the mission executor.
        self.mppi_parameter_client = self.create_client(
            SetParameters, '/controller_server/set_parameters',
            callback_group=self.callback_group)
        self.manipulation_started_mono = None
        self.manipulation_probe_pending = False
        self.manipulation_probe_since = None

        self.reset_runtime()
        self.publish_execution_status()
        self.get_logger().info(
            'Mission flow executor ready. Call /mission/run_flow to run '
            'question -> pickup pose -> drop-off pose for every task.'
        )

    def reset_runtime(self):
        self.active = False
        self.state = 'IDLE'
        self.detail = 'Waiting for /mission/run_flow'
        self.latest_plan = None
        self.coordinator_plan_ready = False
        self.current_plan_id = None
        self.tasks = []
        self.current_task_index = 0
        self.completed_task_count = 0
        self.deadline_ns = 0
        self.expected_goal = None
        self.current_phase = None
        self.current_attempt = 0
        self.leg_sent_monotonic = 0.0
        self.seen_current_navigation = False
        self.last_navigation_status = {}
        self.navigation_best_remaining = math.inf
        self.navigation_progress_mono = None
        self.navigation_progress_anchor_xy = None
        self.leg_history = []
        self.flow_started_monotonic = None
        self.manipulation_started_mono = None
        self.manipulation_probe_pending = False
        self.manipulation_probe_since = None
        self.current_manipulation = None
        self.manipulation_goal_handle = None
        self.pending_navigation_leg = None
        self.nav2_ready = False
        self.precision_dock_timer = None
        self.precision_dock_started_monotonic = None
        self.precision_dock_goal_handle = None
        self.precision_dock_generation += 1
        self.precision_heading_corrections = 0
        self.precision_control_stage = None
        self.precision_settle_count = 0
        self.precision_best_distance = math.inf
        self.precision_last_progress_mono = None

    def load_config(self):
        package_share = get_package_share_directory(
            'moon_warehouse_coordinator'
        )
        path = package_share + '/config/task_execution.yaml'
        try:
            with open(path, 'r', encoding='utf-8') as config_file:
                data = yaml.safe_load(config_file)
        except (OSError, yaml.YAMLError) as error:
            raise RuntimeError(
                f'Cannot load task execution config: {error}'
            ) from error

        if not isinstance(data, dict):
            raise RuntimeError('Task execution config must be an object')
        flow = data.get('flow_execution')
        if not isinstance(flow, dict):
            raise RuntimeError('flow_execution must be an object')

        for section in ('object_approaches', 'destinations'):
            values = flow.get(section)
            if not isinstance(values, dict) or not values:
                raise RuntimeError(f'{section} must be a non-empty object')
            for name, pose in values.items():
                self.validate_pose(name, pose)
        dock_stages = flow.get('object_dock_stages', {})
        if dock_stages:
            if not isinstance(dock_stages, dict):
                raise RuntimeError('object_dock_stages must be an object')
            for name, pose in dock_stages.items():
                self.validate_pose(name, pose)
        return flow

    @staticmethod
    def validate_pose(name, pose):
        if not isinstance(pose, dict):
            raise RuntimeError(f'Pose {name} must be an object')
        values = [pose.get(key) for key in ('x', 'y', 'yaw')]
        if not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in values
        ):
            raise RuntimeError(f'Pose {name} must contain finite x/y/yaw')

    def run_callback(self, request, response):
        del request
        if self.active:
            response.success = False
            response.message = f'Flow is already active: {self.state}'
            return response
        if not self.mission_start_client.wait_for_service(timeout_sec=3.0):
            response.success = False
            response.message = 'Service /mission/start is unavailable'
            return response
        if not self.manipulation_client.wait_for_server(timeout_sec=3.0):
            response.success = False
            response.message = 'Action /manipulation/execute is unavailable'
            return response
        # 控制器守卫: 机械臂/夹爪控制器在 spawner 链上偶尔晚于 Nav2 就绪,
        # 依赖检查若只看到 manipulation server 会漏判(实测抓取阶段才发现
        # 接口缺失而整轮失败)。探测客户端在 __init__ 常驻。
        for probe, label in (
            (self.arm_probe_client, 'arm'),
            (self.gripper_probe_client, 'gripper'),
        ):
            if not probe.wait_for_server(timeout_sec=5.0):
                response.success = False
                response.message = f'{label} controller is not active yet'
                return response
        # The plan may be generated before Nav2 activation.  Individual legs
        # are held by nav2_readiness_tick until bt_navigator reports ACTIVE.
        self.reset_runtime()
        self.active = True
        self.flow_started_monotonic = time.monotonic()
        self.set_state(
            'REQUESTING_PLAN',
            'Calling /mission/start for a new official question',
            timeout_sec=self.plan_timeout_sec,
        )
        self.request_mission_start()
        response.success = True
        response.message = 'Question-driven avoidance flow accepted'
        return response

    def request_mission_start(self):
        """发起 /mission/start; 10s 无应答由常驻节拍器重发。"""
        future = self.mission_start_client.call_async(Trigger.Request())
        future.add_done_callback(self.mission_start_response_callback)
        self.mission_start_last_request = time.monotonic()

    def model_states_callback(self, message):
        for name, pose in zip(message.name, message.pose):
            if name.startswith(('red_cube_', 'blue_cube_')):
                self.cube_world_z[name] = float(pose.position.z)
                self.cube_world_xy[name] = (
                    float(pose.position.x), float(pose.position.y))

    def manipulation_watch_tick(self):
        """抓取/放置结果超时探测: 主动查真值恢复流程推进。"""
        if self.manipulation_probe_pending:
            # DDS 应答丢失会让探测服务的 future 永远挂起; 6s 无结果就
            # 解除挂起, 下个周期重试探测。
            if (self.manipulation_probe_since is not None
                    and time.monotonic() - self.manipulation_probe_since > 6.0):
                self.get_logger().warning(
                    'truth probe response lost; retrying probe')
                self.manipulation_probe_pending = False
                self.manipulation_probe_since = None
            return
        if (not self.active
                or self.state not in self.MANIPULATION_STATES
                or self.manipulation_started_mono is None):
            return
        if (time.monotonic() - self.manipulation_started_mono
                < self.manipulation_timeout_sec):
            return
        task = (self.tasks[self.current_task_index]
                if 0 <= self.current_task_index < len(self.tasks) else {})
        object_id = task.get('object_id', '')
        cube_z = self.cube_world_z.get(object_id)
        if cube_z is None and self.gazebo_state_client.service_is_ready():
            request = GetEntityState.Request()
            request.name = object_id
            request.reference_frame = 'world'
            future = self.gazebo_state_client.call_async(request)
            future.add_done_callback(
                lambda fut: self._resolve_manipulation_by_service(
                    object_id, fut))
            self.manipulation_probe_pending = True
            self.manipulation_probe_since = time.monotonic()
            return
        if cube_z is None:
            self.get_logger().warning(
                f'{object_id} z unknown (no model_states, no service); '
                'will retry probe')
            return
        self.get_logger().warning(
            f'{self.state} result not received in '
            f'{self.manipulation_timeout_sec:.0f}s; probing Gazebo truth')
        self.manipulation_probe_pending = True
        self.manipulation_probe_since = time.monotonic()
        self._resolve_manipulation_timeout(cube_z)

    def _resolve_manipulation_by_service(self, object_id, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(f'probe service failed: {error}')
            self.manipulation_probe_pending = False
            return
        if not response.success:
            self.get_logger().warning(f'probe: {object_id} not found')
            self.manipulation_probe_pending = False
            self.manipulation_probe_since = None
            return
        cube_z = float(response.state.pose.position.z)
        self.cube_world_z[object_id] = cube_z
        if not self.active or self.state not in self.MANIPULATION_STATES:
            self.manipulation_probe_pending = False
            self.manipulation_probe_since = None
            return
        task = self.tasks[self.current_task_index]
        self.get_logger().warning(
            f'{self.state} result not received in '
            f'{self.manipulation_timeout_sec:.0f}s; probing Gazebo truth')
        self._resolve_manipulation_timeout(cube_z)

    def _resolve_manipulation_timeout(self, cube_z):
        if not self.active or self.state not in self.MANIPULATION_STATES:
            return
        task = self.tasks[self.current_task_index]
        record = {
            'operation': ('pick' if self.state == 'GRASPING' else 'place'),
            'object_id': task['object_id'],
            'result': 'SUCCEEDED',
            'error_code': 0,
            'message': 'recovered via truth probe after lost result',
            'stage': 'probe',
        }
        if self.state == 'GRASPING':
            if cube_z > 0.12:
                self.get_logger().warning(
                    f'{task["object_id"]} lifted (z={cube_z:.2f}); '
                    'treating pick as succeeded')
                self._apply_pick_success(task, record)
            else:
                self.fail(
                    f'pick probe: {task["object_id"]} not lifted '
                    f'(z={cube_z:.2f})')
            return
        # place: 方块已落地(非携带高度)即视为成功
        if cube_z < 0.12:
            self.get_logger().warning(
                f'{task["object_id"]} grounded (z={cube_z:.2f}); '
                'treating place as succeeded')
            self._apply_place_success(task, record)
        else:
            self.fail(
                f'place probe: {task["object_id"]} still carried '
                f'(z={cube_z:.2f})')

    def set_mppi_speed(self, speed_limit):
        """Apply a safe transport speed without coupling flow progress to DDS."""
        speed_limit = max(0.10, float(speed_limit))
        if not self.mppi_parameter_client.service_is_ready():
            self.get_logger().warning(
                'MPPI parameter service unavailable; keeping current speed cap')
            return
        request = SetParameters.Request()
        request.parameters = [Parameter(
            'FollowPath.vx_max', value=speed_limit
        ).to_parameter_msg()]
        future = self.mppi_parameter_client.call_async(request)

        def report_result(completed):
            try:
                result = completed.result()
                if not result.results or not result.results[0].successful:
                    reason = (result.results[0].reason if result.results else
                              'empty parameter response')
                    self.get_logger().warning(
                        f'MPPI speed cap was not applied: {reason}')
            except Exception as error:
                self.get_logger().warning(
                    f'MPPI speed update failed non-fatally: {error}')

        future.add_done_callback(report_result)

    def _apply_pick_success(self, task, record):
        self.manipulation_started_mono = None
        task['pickup_manipulation'] = dict(record)
        task['execution_status'] = 'OBJECT_GRASPED'
        # 载物降速, 放置后恢复
        self.set_mppi_speed(0.45)
        self.set_state(
            'OBJECT_GRASPED',
            f'Grasped {task["object_id"]}; navigating to '
            f'zone {task["destination"]}',
        )
        self.start_current_dropoff()

    def _apply_place_success(self, task, record):
        self.manipulation_started_mono = None
        task['dropoff_manipulation'] = dict(record)
        task['execution_status'] = 'COMPLETED'
        self.set_mppi_speed(0.60)
        self.completed_task_count += 1
        self.set_state(
            'TASK_COMPLETED',
            f'Task {self.current_task_index + 1}/{len(self.tasks)} '
            'pick/place completed',
        )
        self.current_task_index += 1
        if self.current_task_index >= len(self.tasks):
            self.complete_flow(
                f'Completed {self.completed_task_count} pick/place task(s)')
        else:
            self.start_current_pickup()

    def mission_start_retry_tick(self):
        if not self.active or self.state != 'REQUESTING_PLAN':
            return
        if time.monotonic() - self.mission_start_last_request < 10.0:
            return
        self.get_logger().warning(
            '/mission/start response lost; re-issuing request')
        self.request_mission_start()

    def cancel_nav2_wait(self):
        # Client and timer are process-lifetime entities.  Destroying either
        # from a callback can invalidate the executor wait set.  Clearing the
        # queued leg is enough to prevent a late lifecycle reply from moving
        # the robot after STOP/ERROR.
        self.pending_navigation_leg = None

    def nav2_readiness_tick(self):
        """Poll bt_navigator without blocking and dispatch one queued leg."""
        now = time.monotonic()
        future = self.nav2_state_future
        if future is not None:
            if not future.done():
                # A lost service reply must not permanently disable polling.
                if (self.nav2_probe_started_mono is not None
                        and now - self.nav2_probe_started_mono > 2.0):
                    self.get_logger().warning(
                        'bt_navigator lifecycle reply timed out; retrying',
                        throttle_duration_sec=5.0)
                    self.nav2_state_future = None
                    self.nav2_probe_started_mono = None
                return
            try:
                response = future.result()
                self.nav2_ready = (
                    int(response.current_state.id)
                    == int(State.PRIMARY_STATE_ACTIVE)
                )
            except Exception as error:
                self.nav2_ready = False
                self.get_logger().warning(
                    f'bt_navigator lifecycle probe failed: {error}',
                    throttle_duration_sec=5.0)
            self.nav2_state_future = None
            self.nav2_probe_started_mono = None

            if (self.nav2_ready and self.active
                    and self.pending_navigation_leg is not None):
                phase, target, attempt = self.pending_navigation_leg
                self.pending_navigation_leg = None
                self.get_logger().info(
                    'bt_navigator is ACTIVE; dispatching queued '
                    f'{phase} leg attempt {attempt}')
                self.publish_leg(phase, target, attempt)

        if (self.nav2_state_future is None
                and self.nav2_state_client.service_is_ready()):
            self.nav2_state_future = self.nav2_state_client.call_async(
                GetState.Request())
            self.nav2_probe_started_mono = now

    def stop_callback(self, request, response):
        del request
        if not self.active:
            response.success = False
            response.message = 'Flow is not active'
            return response

        self.active = False
        self.deadline_ns = 0
        self.cancel_retry_timer()
        self.cancel_precision_dock()
        self.stop_base()
        self.cancel_nav2_wait()
        if self.navigation_cancel_client.service_is_ready():
            self.navigation_cancel_client.call_async(Trigger.Request())
        if self.manipulation_goal_handle is not None:
            self.manipulation_goal_handle.cancel_goal_async()
        self.release_carried_cube()
        self.set_state('STOPPED', 'Flow stopped by /mission/stop_flow')
        response.success = True
        response.message = 'Flow stop requested'
        return response

    def mission_start_response_callback(self, future):
        if not self.active:
            return
        try:
            result = future.result()
        except Exception as error:
            self.fail(f'/mission/start call failed: {error}')
            return
        if not result.success:
            # 重发场景下, 第一次请求其实已送达时协调器会返回"already
            # running"。这不是失败: 语义流程在跑, 等它的 PLAN 即可。
            if 'already' in result.message.lower():
                self.get_logger().info(
                    '/mission/start already running; waiting for plan')
                return
            self.fail(f'/mission/start rejected: {result.message}')
            return
        self.set_state(
            'WAITING_PLAN',
            'Waiting for PLAN_READY and /mission/plan',
            timeout_sec=self.plan_timeout_sec,
        )
        self.try_begin_plan()

    def plan_callback(self, message):
        try:
            plan = json.loads(message.data)
            tasks = plan.get('tasks')
            if not isinstance(tasks, list):
                raise ValueError('tasks must be a list')
            self.latest_plan = plan
        except (json.JSONDecodeError, TypeError, ValueError) as error:
            if self.active:
                self.fail(f'Invalid /mission/plan: {error}')
            return
        self.try_begin_plan()

    def mission_status_callback(self, message):
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        mission_state = str(status.get('state', ''))
        if self.active and mission_state == 'ERROR':
            self.fail(
                'Coordinator error while preparing flow: '
                + str(status.get('detail', 'unknown error'))
            )
            return
        if mission_state == 'PLAN_READY':
            self.coordinator_plan_ready = True
            # 记录 PLAN_READY 携带的 plan_id：只有同一轮请求生成的
            # plan 才允许启动执行，避免多线程下旧 plan 抢先。
            plan_id = status.get('plan_id')
            if plan_id is not None:
                self.current_plan_id = plan_id
            self.try_begin_plan()

    def try_begin_plan(self):
        if not self.active or self.state not in {
            'REQUESTING_PLAN',
            'WAITING_PLAN',
        }:
            return
        if not self.coordinator_plan_ready or self.latest_plan is None:
            return
        # plan_id 匹配检查：PLAN_READY 先于真正的 plan 到达时，
        # latest_plan 仍是旧消息，必须等到本轮 plan_id 对上才启动。
        if self.current_plan_id is not None:
            plan_id = self.latest_plan.get('plan_id')
            if plan_id is not None and plan_id != self.current_plan_id:
                return

        tasks = self.latest_plan.get('tasks', [])
        expected_count = self.latest_plan.get('task_count')
        if expected_count != len(tasks):
            self.fail('task_count does not match /mission/plan tasks')
            return

        prepared = []
        for task in tasks:
            if not isinstance(task, dict):
                self.fail('Every task in /mission/plan must be an object')
                return
            object_id = str(task.get('object_id', ''))
            destination = str(task.get('destination', '')).upper()
            if object_id not in self.object_approaches:
                self.fail(f'No approach pose configured for {object_id}')
                return
            if destination not in self.destinations:
                self.fail(f'No drop-off pose configured for {destination}')
                return
            prepared_task = dict(task)
            prepared_task['execution_status'] = 'PENDING'
            prepared_task['pickup_manipulation'] = {'result': 'PENDING'}
            prepared_task['dropoff_manipulation'] = {'result': 'PENDING'}
            prepared_task['pick_attempts'] = 0
            prepared_task['place_attempts'] = 0
            prepared.append(prepared_task)

        self.tasks = prepared
        self.current_task_index = 0
        self.completed_task_count = 0
        if not self.tasks:
            self.complete_flow('Question produced an empty task queue')
            return
        self.start_current_pickup()

    def start_current_pickup(self):
        task = self.tasks[self.current_task_index]
        task['execution_status'] = 'NAVIGATING_TO_PICKUP'
        pre_approach = self.object_preapproaches.get(task['object_id'])
        if pre_approach is not None:
            # 两段式取件：先到同航向的预接近点，最后一段直行对准驶入，
            # 避免差速底盘在斜向接近时于目标附近振荡。
            self.send_leg('PICKUP_PRE', pre_approach, attempt=1)
            return
        stage = self.object_dock_stages.get(task['object_id'])
        if stage is not None:
            self.send_leg('PICKUP_DOCK_STAGE', stage, attempt=1)
            return
        target = self.object_approaches[task['object_id']]
        self.send_leg('PICKUP', target, attempt=1)

    def start_current_dropoff(self):
        task = self.tasks[self.current_task_index]
        task['execution_status'] = 'NAVIGATING_TO_DROPOFF'
        transit = self.dropoff_transits.get(task['object_id'])
        if transit is not None and not task.get('dropoff_transit_done', False):
            self.send_leg('DROPOFF_TRANSIT', transit, attempt=1)
            return
        target = self.destinations[str(task['destination']).upper()]
        self.send_leg('DROPOFF', target, attempt=1)

    def send_leg(self, phase, target, attempt):
        if not self.active:
            return
        target_copy = {
            'x': float(target['x']),
            'y': float(target['y']),
            'yaw': float(target['yaw']),
        }
        if not self.nav2_ready:
            self.pending_navigation_leg = (phase, target_copy, attempt)
            self.current_phase = phase
            self.current_attempt = attempt
            self.expected_goal = dict(target_copy)
            self.seen_current_navigation = False
            self.last_navigation_status = {}
            self.stop_base()
            self.set_state(
                'WAITING_NAVIGATION_READY',
                f'Waiting for active bt_navigator before {phase} '
                f'leg attempt {attempt}',
                timeout_sec=30.0,
            )
            return
        self.publish_leg(phase, target_copy, attempt)

    def publish_leg(self, phase, target, attempt):
        """Publish a navigation leg only after the lifecycle gate opens."""
        if not self.active:
            return
        self.current_phase = phase
        self.current_attempt = attempt
        self.leg_sent_monotonic = time.monotonic()
        self.expected_goal = {
            'x': float(target['x']),
            'y': float(target['y']),
            'yaw': float(target['yaw']),
        }
        self.seen_current_navigation = False
        self.last_navigation_status = {}
        self.navigation_best_remaining = math.inf
        self.navigation_progress_mono = None
        if self.latest_base_x is not None and self.latest_base_y is not None:
            self.navigation_progress_anchor_xy = (
                self.latest_base_x, self.latest_base_y)
        else:
            self.navigation_progress_anchor_xy = None

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        # 不打时间戳(stamp=0): 实测带仿真时间戳的导航目标会让整段导航
        # 零速冻结(bt_navigator已接受但controller不输出), stamp=0 让
        # Nav2 使用最新状态, 导航立即正常。
        pose.pose.position.x = self.expected_goal['x']
        pose.pose.position.y = self.expected_goal['y']
        pose.pose.orientation.z = math.sin(
            self.expected_goal['yaw'] / 2.0
        )
        pose.pose.orientation.w = math.cos(
            self.expected_goal['yaw'] / 2.0
        )

        state = (
            'NAVIGATING_PICKUP'
            if phase in ('PICKUP', 'PICKUP_PRE', 'PICKUP_DOCK_STAGE')
            else 'NAVIGATING_DROPOFF'
        )
        task = self.tasks[self.current_task_index]
        self.set_state(
            state,
            f'Task {self.current_task_index + 1}/{len(self.tasks)} '
            f'{phase.lower()} leg attempt {attempt}: '
            f'x={self.expected_goal["x"]:.2f}, '
            f'y={self.expected_goal["y"]:.2f}',
            timeout_sec=self.navigation_timeout_sec,
        )
        self.navigation_goal_publisher.publish(pose)

    def navigation_status_callback(self, message):
        if not self.active or self.state not in self.NAVIGATION_STATES:
            return
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        goal = status.get('goal')
        if not self.goal_matches(goal):
            return
        # Once proximity acceptance has requested cancellation, every later
        # ACCEPTED/CANCELING/CANCELED sample belongs to the retired staging
        # goal.  Ignoring the whole tail prevents CANCELING from re-arming
        # seen_current_navigation and turning an intentional handoff into a
        # recovery retry.
        if self.current_phase in ('PICKUP_PRE_HANDOFF',
                                  'PICKUP_DOCK_STAGE_HANDOFF'):
            return

        result = str(status.get('result', 'NONE')).upper()
        active = bool(status.get('active', False))
        if active or result in {'PENDING', 'ACCEPTED', 'CANCELING'}:
            self.seen_current_navigation = True
            self.last_navigation_status = status
            feedback = status.get('feedback', {})
            try:
                remaining_for_watchdog = float(
                    feedback.get('distance_remaining_m', math.inf))
            except (TypeError, ValueError):
                remaining_for_watchdog = math.inf
            if math.isfinite(remaining_for_watchdog):
                now_mono = time.monotonic()
                if (not math.isfinite(self.navigation_best_remaining)
                        or remaining_for_watchdog
                        <= self.navigation_best_remaining - 0.20):
                    self.navigation_best_remaining = remaining_for_watchdog
                    self.navigation_progress_mono = now_mono
                    if (self.latest_base_x is not None
                            and self.latest_base_y is not None):
                        self.navigation_progress_anchor_xy = (
                            self.latest_base_x, self.latest_base_y)
            goal_active_age = time.monotonic() - self.leg_sent_monotonic
            if (active
                    and goal_active_age >= 1.5
                    and self.current_phase in (
                        'PICKUP_PRE', 'PICKUP', 'DROPOFF')):
                feedback = status.get('feedback', {})
                try:
                    remaining = float(
                        feedback.get('distance_remaining_m', math.inf))
                except (TypeError, ValueError):
                    remaining = math.inf
                position_error = self.expected_position_error()
                yaw_aligned = self.is_expected_yaw_aligned()
                dropoff_yaw_aligned = self.is_expected_yaw_aligned(
                    self.dropoff_yaw_tolerance_rad)
                if (self.current_phase == 'PICKUP_PRE'
                        and position_error <= self.preapproach_acceptance_m):
                    self.get_logger().info(
                        f'Pre-approach accepted at {position_error:.3f} m '
                        'remaining; switching to strict final docking leg.')
                    if self.navigation_cancel_client.service_is_ready():
                        self.navigation_cancel_client.call_async(
                            Trigger.Request())
                    # Do not publish the final goal in the same callback as
                    # the cancel request.  The coordinator still owns the
                    # old action handle for a short interval and would
                    # reject the new goal as "already active".  Record this
                    # staging leg now, ignore its late CANCELED terminal
                    # status, then hand off after the action state settles.
                    record = self.make_leg_record('SUCCEEDED_PROXIMITY')
                    self.leg_history.append(record)
                    task = self.tasks[self.current_task_index]
                    task['prepickup_navigation'] = record
                    self.seen_current_navigation = False
                    self.current_phase = 'PICKUP_PRE_HANDOFF'
                    self.set_state(
                        'NAVIGATING_PICKUP',
                        f'Pre-approach reached for {task["object_id"]}; '
                        'waiting for navigation cancel handoff',
                        timeout_sec=4.0,
                    )
                    self.cancel_retry_timer()

                    def start_final_docking():
                        self.cancel_retry_timer()
                        if self.active:
                            # A second Nav2 goal only 0.15--0.25 m away can
                            # generate a large endpoint loop while trying to
                            # satisfy yaw.  The pre-approach already opens onto
                            # a surveyed clear corridor; hand it directly to
                            # Spin + DriveOnHeading, still downstream of the
                            # collision monitor.
                            self.start_precision_dock()

                    self.retry_timer = self.create_timer(
                        1.0,
                        start_final_docking,
                        callback_group=self.callback_group,
                    )
                    return
                if (self.current_phase == 'PICKUP_DOCK_STAGE'
                        and position_error <= self.pickup_acceptance_m):
                    self.handoff_dock_stage_proximity(position_error)
                    return
                if (self.current_phase == 'PICKUP'
                        and position_error <= self.pickup_acceptance_m
                        and yaw_aligned):
                    self.get_logger().info(
                        f'Final docking accepted at {position_error:.3f} m '
                        'navigation residual; handing off to strict '
                        'grasp-window validation.')
                    if self.navigation_cancel_client.service_is_ready():
                        self.navigation_cancel_client.call_async(
                            Trigger.Request())
                    record = self.make_leg_record('SUCCEEDED_PROXIMITY')
                    self.leg_history.append(record)
                    task = self.tasks[self.current_task_index]
                    task['pickup_navigation'] = record
                    task['execution_status'] = 'PICKUP_REACHED'
                    self.seen_current_navigation = False
                    self.set_state(
                        'PICKUP_REACHED',
                        f'Reached {task["object_id"]}; waiting for '
                        'navigation cancel handoff before grasp',
                        timeout_sec=4.0,
                    )
                    self.cancel_retry_timer()

                    def start_strict_grasp():
                        self.cancel_retry_timer()
                        if self.active:
                            self.start_manipulation('pick')

                    self.retry_timer = self.create_timer(
                        1.0,
                        start_strict_grasp,
                        callback_group=self.callback_group,
                    )
                    return
                if (self.current_phase == 'DROPOFF'
                        and position_error
                        <= self.current_dropoff_acceptance_m()
                        and dropoff_yaw_aligned):
                    self.get_logger().info(
                        f'Drop-off zone accepted at {position_error:.3f} m '
                        'navigation residual with arm-aligned terminal yaw.')
                    if self.navigation_cancel_client.service_is_ready():
                        self.navigation_cancel_client.call_async(
                            Trigger.Request())
                    record = self.make_leg_record('SUCCEEDED_ZONE_PROXIMITY')
                    self.leg_history.append(record)
                    task = self.tasks[self.current_task_index]
                    task['dropoff_navigation'] = record
                    task['execution_status'] = 'DROPOFF_REACHED'
                    self.seen_current_navigation = False
                    self.set_state(
                        'DROPOFF_REACHED',
                        f'Reached zone {task["destination"]}; waiting for '
                        'navigation cancel handoff before place',
                        timeout_sec=4.0,
                    )
                    self.cancel_retry_timer()

                    def start_strict_place():
                        self.cancel_retry_timer()
                        if self.active:
                            self.start_manipulation('place')

                    self.retry_timer = self.create_timer(
                        1.0,
                        start_strict_place,
                        callback_group=self.callback_group,
                    )
                    return
            self.publish_execution_status()
            return
        if not self.seen_current_navigation and result == 'REJECTED':
            # Nav2 在生命周期恢复窗口内可能直接拒绝目标，此时不会先发布
            # ACCEPTED/active。旧逻辑永远等不到 seen_current_navigation，
            # 流程会静默卡住直到整腿超时。REJECTED 不会是一个已接受旧
            # 目标的迟到结果，可以立即走清图、保进度、重试恢复链。
            self.last_navigation_status = status
            self.finish_leg_failure(result)
            return
        if not self.seen_current_navigation:
            return

        # /mission/navigation_status is transient.  A terminal status for a
        # just-cancelled goal may have identical coordinates to a retry and
        # arrive immediately after the new publication.  It is not the new
        # goal's result, so wait for its action state to become observable.
        if (result in self.FAILURE_RESULTS and
                time.monotonic() - self.leg_sent_monotonic < 1.0):
            return

        self.last_navigation_status = status
        if result == 'SUCCEEDED':
            self.finish_leg_success()
        elif result in self.FAILURE_RESULTS:
            self.finish_leg_failure(result)

    def handoff_dock_stage_proximity(self, position_error):
        """Promote a safe staging arrival without waiting for Nav2's result.

        Collision Monitor intentionally leaves the action ACTIVE when it
        holds a chassis near a cube.  The base pose, unlike the action result,
        is still continuously available from odometry, so use the measured
        12 cm staging gate and keep the normal guarded final-dock sequence.
        """
        if (not self.active or self.current_phase != 'PICKUP_DOCK_STAGE'
                or self.state not in self.NAVIGATION_STATES):
            return False
        self.get_logger().info(
            f'Dock stage accepted at {position_error:.3f} m; '
            'handing off to collision-guarded final approach.')
        if self.navigation_cancel_client.service_is_ready():
            self.navigation_cancel_client.call_async(Trigger.Request())
        record = self.make_leg_record('SUCCEEDED_PROXIMITY')
        self.leg_history.append(record)
        task = self.tasks[self.current_task_index]
        task['dock_stage_navigation'] = record
        self.seen_current_navigation = False
        self.current_phase = 'PICKUP_DOCK_STAGE_HANDOFF'
        self.set_state(
            'NAVIGATING_PICKUP',
            f'Dock stage reached for {task["object_id"]}; '
            'waiting for navigation cancel handoff',
            timeout_sec=4.0,
        )
        self.cancel_retry_timer()

        def start_guarded_final_dock():
            self.cancel_retry_timer()
            if self.active:
                self.start_precision_dock()

        self.retry_timer = self.create_timer(
            1.0,
            start_guarded_final_dock,
            callback_group=self.callback_group,
        )
        return True

    def odom_callback(self, message):
        self.latest_base_x = message.pose.pose.position.x
        self.latest_base_y = message.pose.pose.position.y
        orientation = message.pose.pose.orientation
        self.latest_base_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                         + orientation.z * orientation.z),
        )
        # Do not refresh the navigation progress watchdog from raw chassis
        # displacement.  A robot circling near a conflict zone can travel a
        # long distance while making no progress towards the current goal.
        # navigation_status_callback owns the progress clock and only refreshes
        # it after Nav2's best distance_remaining improves by 0.20 m.

    @staticmethod
    def angle_error(a, b):
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    def is_expected_yaw_aligned(self, tolerance=None):
        if self.latest_base_yaw is None or self.expected_goal is None:
            return False
        if tolerance is None:
            tolerance = self.pickup_yaw_tolerance_rad
        return self.angle_error(
            self.latest_base_yaw,
            self.expected_goal['yaw'],
        ) <= float(tolerance)

    def expected_position_error(self):
        if (self.latest_base_x is None or self.latest_base_y is None
                or self.expected_goal is None):
            return math.inf
        return math.hypot(
            self.latest_base_x - self.expected_goal['x'],
            self.latest_base_y - self.expected_goal['y'],
        )

    def current_dropoff_acceptance_m(self):
        """Return the surveyed reachability gate for the active zone.

        The gate controls only when Nav2 hands over to manipulation.  The
        dynamic placement IK and Gazebo zone-boundary check remain the final
        authorities for release, so a zone-specific differential-drive
        stopping residual cannot reduce physical placement precision.
        """
        if 0 <= self.current_task_index < len(self.tasks):
            destination = str(
                self.tasks[self.current_task_index].get(
                    'destination', '')).upper()
            if destination in self.dropoff_acceptance_by_destination:
                return max(
                    self.dropoff_acceptance_m,
                    float(self.dropoff_acceptance_by_destination[destination]),
                )
        return self.dropoff_acceptance_m

    def goal_matches(self, goal):
        if not isinstance(goal, dict) or self.expected_goal is None:
            return False
        try:
            return (
                abs(float(goal.get('x')) - self.expected_goal['x']) < 0.02
                and abs(float(goal.get('y')) - self.expected_goal['y']) < 0.02
            )
        except (TypeError, ValueError):
            return False

    def finish_leg_success(self):
        record = self.make_leg_record('SUCCEEDED')
        self.leg_history.append(record)
        task = self.tasks[self.current_task_index]
        if self.current_phase == 'DROPOFF_TRANSIT':
            task['dropoff_transit_navigation'] = record
            task['dropoff_transit_done'] = True
            self.set_state(
                'NAVIGATING_DROPOFF',
                f'Diagonal transit cleared for {task["object_id"]}; '
                f'continuing to zone {task["destination"]}',
            )
            self.start_current_dropoff()
            return
        if self.current_phase == 'PICKUP_PRE':
            task['prepickup_navigation'] = record
            self.set_state(
                'NAVIGATING_PICKUP',
                f'Pre-approach reached for {task["object_id"]}; '
                'driving the straight final approach',
            )
            self.start_precision_dock()
            return
        if self.current_phase == 'PICKUP_DOCK_STAGE':
            task['dock_stage_navigation'] = record
            self.start_precision_dock()
            return
        if self.current_phase == 'PICKUP':
            # Nav2 的终态只说明它满足自身 goal checker；抓取需要更严格
            # 的底盘真值。不得因为一条过时/宽松的 Nav2 成功回执就伸臂，
            # 否则方块会被预抓取姿态从侧面推走。把该情况纳入已有的
            # 清图、等待动态障碍、保留任务进度的恢复链。
            position_error = self.expected_position_error()
            yaw_aligned = self.is_expected_yaw_aligned()
            if (position_error > self.pickup_acceptance_m
                    or not yaw_aligned):
                self.get_logger().warning(
                    'Nav2 reported pickup success outside the physical '
                    f'docking gate: position_error={position_error:.3f}m, '
                    f'yaw_aligned={yaw_aligned}; retrying without grasp')
                self.finish_leg_failure('DOCKING_RESIDUAL')
                return
            task['pickup_navigation'] = record
            task['execution_status'] = 'PICKUP_REACHED'
            self.set_state(
                'PICKUP_REACHED',
                f'Reached {task["object_id"]}; starting grasp action',
            )
            self.start_manipulation('pick')
            return

        position_error = self.expected_position_error()
        yaw_aligned = self.is_expected_yaw_aligned(
            self.dropoff_yaw_tolerance_rad)
        if (position_error > self.current_dropoff_acceptance_m()
                or not yaw_aligned):
            self.get_logger().warning(
                'Nav2 reported drop-off success outside the physical place '
                f'gate: position_error={position_error:.3f}m, '
                f'yaw_aligned={yaw_aligned}; retrying without releasing cube')
            self.finish_leg_failure('PLACEMENT_DOCKING_RESIDUAL')
            return
        task['dropoff_navigation'] = record
        task['execution_status'] = 'DROPOFF_REACHED'
        self.set_state(
            'DROPOFF_REACHED',
            f'Reached zone {task["destination"]}; starting place action',
        )
        self.start_manipulation('place')

    @staticmethod
    def signed_angle_error(target, current):
        return math.atan2(
            math.sin(target - current), math.cos(target - current)
        )

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def stop_base(self):
        self.emergency_stop_until_mono = max(
            self.emergency_stop_until_mono,
            time.monotonic() + 1.5,
        )
        self.publish_zero_velocity()

    def publish_zero_velocity(self):
        zero = Twist()
        self.zero_vel_publisher.publish(zero)
        self.precision_velocity_publisher.publish(zero)

    def emergency_stop_tick(self):
        if time.monotonic() < self.emergency_stop_until_mono:
            self.publish_zero_velocity()

    def cancel_precision_dock(self):
        if getattr(self, 'precision_dock_timer', None) is not None:
            self.precision_dock_timer.cancel()
            self.destroy_timer(self.precision_dock_timer)
            self.precision_dock_timer = None
        if self.precision_dock_goal_handle is not None:
            self.precision_dock_goal_handle.cancel_goal_async()
            self.precision_dock_goal_handle = None
        self.precision_dock_started_monotonic = None
        self.precision_control_stage = None
        self.publish_zero_velocity()

    def start_precision_dock(self):
        """Safely drive the final known-clear docking corridor by odometry.

        Nav2 handles the long path, unknown scan data and moving obstacles.
        The last half metre is deliberately separated because the target cube
        must be excluded from the global costmap to remain reachable; letting
        a global planner optimise through that exclusion can otherwise push
        the cube before the arm is in range.
        """
        if not self.active:
            return
        self.cancel_precision_dock()
        self.precision_dock_generation += 1
        generation = self.precision_dock_generation
        task = self.tasks[self.current_task_index]
        target = self.object_approaches[task['object_id']]
        self.current_phase = 'PICKUP_DOCK'
        self.expected_goal = {
            'x': float(target['x']),
            'y': float(target['y']),
            'yaw': float(target['yaw']),
        }
        self.leg_sent_monotonic = time.monotonic()
        self.precision_dock_started_monotonic = self.leg_sent_monotonic
        task['execution_status'] = 'PRECISION_DOCKING'
        self.set_state(
            'PRECISION_DOCKING',
            f'Final Nav2 behavior dock for {task["object_id"]}; '
            'collision monitor remains active',
            timeout_sec=self.precision_dock_timeout_sec,
        )
        if (self.latest_base_x is None or self.latest_base_y is None
                or self.latest_base_yaw is None):
            self.precision_dock_failed('odometry unavailable for precision dock')
            return
        dx = self.expected_goal['x'] - self.latest_base_x
        dy = self.expected_goal['y'] - self.latest_base_y
        distance = math.hypot(dx, dy)
        if distance > 1.10:
            self.precision_dock_failed(
                f'precision dock started {distance:.2f}m from its corridor')
            return
        self.precision_control_stage = 'ALIGN_PATH'
        self.precision_settle_count = 0
        self.precision_best_distance = distance
        self.precision_last_progress_mono = time.monotonic()
        self.precision_dock_timer = self.create_timer(
            0.05,
            lambda: self.precision_control_tick(generation),
            callback_group=self.callback_group,
        )

    def precision_control_tick(self, generation):
        """Closed-loop final dock through the complete velocity safety chain."""
        if (not self.active or generation != self.precision_dock_generation
                or self.state != 'PRECISION_DOCKING'):
            self.publish_zero_velocity()
            return
        if (self.latest_base_x is None or self.latest_base_y is None
                or self.latest_base_yaw is None):
            self.precision_dock_failed('odometry lost during precision dock')
            return
        now = time.monotonic()
        if (self.precision_dock_started_monotonic is not None
                and now - self.precision_dock_started_monotonic
                > self.precision_dock_timeout_sec):
            self.precision_dock_failed('closed-loop precision dock timed out')
            return

        dx = self.expected_goal['x'] - self.latest_base_x
        dy = self.expected_goal['y'] - self.latest_base_y
        distance = math.hypot(dx, dy)
        path_yaw = (
            math.atan2(dy, dx)
            if distance > 0.01 else self.expected_goal['yaw']
        )
        command = Twist()

        if self.precision_control_stage == 'ALIGN_PATH':
            error = self.signed_angle_error(path_yaw, self.latest_base_yaw)
            if abs(error) <= 0.045:
                self.precision_settle_count += 1
                self.publish_zero_velocity()
                if self.precision_settle_count >= 4:
                    self.precision_control_stage = 'DRIVE'
                    self.precision_settle_count = 0
                    self.precision_best_distance = distance
                    self.precision_last_progress_mono = now
                return
            self.precision_settle_count = 0
            command.angular.z = self._clamp(1.3 * error, -0.42, 0.42)
            if abs(command.angular.z) < 0.14:
                command.angular.z = math.copysign(0.14, error)
            self.precision_velocity_publisher.publish(command)
            return

        if self.precision_control_stage == 'DRIVE':
            object_id = self.tasks[self.current_task_index]['object_id']
            cube_xy = self.cube_world_xy.get(object_id)
            cube_distance = math.inf
            if cube_xy is not None:
                cube_distance = math.hypot(
                    cube_xy[0] - self.latest_base_x,
                    cube_xy[1] - self.latest_base_y)
            if cube_distance <= 0.25:
                self.publish_zero_velocity()
                if distance <= self.pickup_acceptance_m:
                    self.precision_control_stage = 'ALIGN_GRASP'
                    self.precision_settle_count = 0
                    return
                self.precision_dock_failed(
                    f'cube hard-stop at {cube_distance:.3f}m before dock gate')
                return
            if distance <= 0.045:
                self.publish_zero_velocity()
                self.precision_settle_count += 1
                if self.precision_settle_count >= 5:
                    self.precision_control_stage = 'ALIGN_GRASP'
                    self.precision_settle_count = 0
                return
            self.precision_settle_count = 0
            if distance + 0.006 < self.precision_best_distance:
                self.precision_best_distance = distance
                self.precision_last_progress_mono = now
            elif (self.precision_last_progress_mono is not None
                    and now - self.precision_last_progress_mono > 3.0):
                self.precision_dock_failed(
                    f'collision-held precision drive at residual={distance:.3f}m')
                return
            heading_error = self.signed_angle_error(
                path_yaw, self.latest_base_yaw)
            command.linear.x = self._clamp(0.65 * distance, 0.025, 0.11)
            command.angular.z = self._clamp(
                1.0 * heading_error, -0.18, 0.18)
            self.precision_velocity_publisher.publish(command)
            return

        if self.precision_control_stage == 'ALIGN_GRASP':
            error = self.signed_angle_error(
                self.expected_goal['yaw'], self.latest_base_yaw)
            if abs(error) <= self.pickup_yaw_tolerance_rad:
                self.publish_zero_velocity()
                self.precision_settle_count += 1
                if self.precision_settle_count >= 5:
                    self.cancel_precision_dock()
                    self.complete_precision_dock(generation)
                return
            self.precision_settle_count = 0
            command.angular.z = self._clamp(1.2 * error, -0.32, 0.32)
            if abs(command.angular.z) < 0.14:
                command.angular.z = math.copysign(0.14, error)
            self.precision_velocity_publisher.publish(command)
            return

        self.precision_dock_failed(
            f'unknown precision control stage {self.precision_control_stage}')

    def start_precision_spin(self, generation, relative_yaw, phase):
        if (not self.active or generation != self.precision_dock_generation
                or self.state != 'PRECISION_DOCKING'):
            return
        if not self.precision_spin_client.wait_for_server(timeout_sec=2.0):
            self.precision_dock_failed('Nav2 spin behavior is unavailable')
            return
        goal = Spin.Goal()
        goal.target_yaw = float(relative_yaw)
        goal.time_allowance.sec = 8
        future = self.precision_spin_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self.precision_spin_goal_response(
                generation, phase, completed))

    def precision_spin_goal_response(self, generation, phase, future):
        if (not self.active or generation != self.precision_dock_generation
                or self.state != 'PRECISION_DOCKING'):
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self.precision_dock_failed(f'precision spin request failed: {error}')
            return
        if goal_handle is None or not goal_handle.accepted:
            self.precision_dock_failed('precision spin goal was rejected')
            return
        self.precision_dock_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda completed: self.precision_spin_result(
                generation, phase, completed))

    def precision_spin_result(self, generation, phase, future):
        if (not self.active or generation != self.precision_dock_generation
                or self.state != 'PRECISION_DOCKING'):
            return
        self.precision_dock_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as error:
            self.precision_dock_failed(f'precision spin failed: {error}')
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.precision_dock_failed(
                f'precision spin ended with status={wrapped.status}')
            return
        if phase == 'ALIGN_PATH':
            self.start_precision_drive(generation)
        else:
            # The behavior result can arrive one odometry cycle before the
            # final wheel/TF update.  Verify after a short physical settle;
            # otherwise a successful spin is falsely classified as yaw error.
            self.cancel_retry_timer()

            def verify_settled_heading():
                self.cancel_retry_timer()
                self.complete_precision_dock(generation)

            self.retry_timer = self.create_timer(
                0.35,
                verify_settled_heading,
                callback_group=self.callback_group,
            )

    def start_precision_drive(self, generation):
        if (not self.active or generation != self.precision_dock_generation
                or self.state != 'PRECISION_DOCKING'):
            return
        if (self.latest_base_x is None or self.latest_base_y is None):
            self.precision_dock_failed('odometry lost before straight dock')
            return
        if not self.precision_drive_client.wait_for_server(timeout_sec=2.0):
            self.precision_dock_failed('Nav2 drive-on-heading behavior unavailable')
            return
        dx = self.expected_goal['x'] - self.latest_base_x
        dy = self.expected_goal['y'] - self.latest_base_y
        goal = DriveOnHeading.Goal()
        goal.target.x = float(math.hypot(dx, dy))
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = float(self.precision_dock_max_speed_mps)
        goal.time_allowance.sec = 16
        future = self.precision_drive_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self.precision_drive_goal_response(
                generation, completed))

    def precision_drive_goal_response(self, generation, future):
        if (not self.active or generation != self.precision_dock_generation
                or self.state != 'PRECISION_DOCKING'):
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self.precision_dock_failed(f'straight dock request failed: {error}')
            return
        if goal_handle is None or not goal_handle.accepted:
            self.precision_dock_failed('straight dock goal was rejected')
            return
        self.precision_dock_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda completed: self.precision_drive_result(generation, completed))

    def precision_drive_result(self, generation, future):
        if (not self.active or generation != self.precision_dock_generation
                or self.state != 'PRECISION_DOCKING'):
            return
        self.precision_dock_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as error:
            self.precision_dock_failed(f'straight dock failed: {error}')
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            # 局部碰撞监视器会在把待抓方块视作实体时提前数厘米制动。
            # 绝不强行继续前进；只有底盘真实残差仍落入经验证的夹爪
            # 窗口时，才保持当前无接触位置并仅原地转到抓取航向。
            residual = self.expected_position_error()
            if residual > self.pickup_acceptance_m:
                self.precision_dock_failed(
                    f'straight dock ended status={wrapped.status}, '
                    f'residual={residual:.3f}m')
                return
            self.get_logger().info(
                'Straight dock stopped by collision monitor inside the '
                f'physical grasp gate ({residual:.3f}m); holding position')
        if self.latest_base_yaw is None:
            self.precision_dock_failed('odometry lost after straight dock')
            return
        self.start_precision_spin(
            generation,
            self.signed_angle_error(
                self.expected_goal['yaw'], self.latest_base_yaw),
            'ALIGN_GRASP',
        )

    def complete_precision_dock(self, generation):
        if (not self.active or generation != self.precision_dock_generation
                or self.expected_goal is None):
            return
        position_error = self.expected_position_error()
        yaw_aligned = self.is_expected_yaw_aligned()
        if position_error > self.pickup_acceptance_m:
            self.precision_dock_failed(
                f'Nav2 behavior residual {position_error:.3f}m, '
                f'yaw_aligned={yaw_aligned}')
            return
        if not yaw_aligned:
            yaw_error = self.signed_angle_error(
                self.expected_goal['yaw'], self.latest_base_yaw)
            if (self.precision_heading_corrections < 2
                    and abs(yaw_error) <= 0.35):
                self.precision_heading_corrections += 1
                self.get_logger().info(
                    'Dock position is valid; applying local heading '
                    f'correction {self.precision_heading_corrections}/2 '
                    f'({yaw_error:+.3f} rad) without replanning the route')
                self.start_precision_spin(
                    generation, yaw_error, 'ALIGN_GRASP_CORRECTION')
                return
            self.precision_dock_failed(
                f'dock heading remains outside grasp gate: '
                f'error={yaw_error:+.3f}rad')
            return
        task = self.tasks[self.current_task_index]
        record = {
            'task_index': self.current_task_index,
            'phase': 'PICKUP_PRECISION_DOCK',
            'attempt': self.current_attempt,
            'target': dict(self.expected_goal),
            'result': 'SUCCEEDED',
            'navigation_time_s': round(
                time.monotonic() - self.leg_sent_monotonic, 2),
            'number_of_recoveries': 0,
        }
        self.leg_history.append(record)
        task['pickup_navigation'] = record
        task['execution_status'] = 'PICKUP_REACHED'
        self.current_phase = 'PICKUP'
        self.set_state(
            'PICKUP_REACHED',
            f'Physical docking gate passed for {task["object_id"]}; '
            'starting close-range grasp',
        )
        self.start_manipulation('pick')

    def precision_dock_failed(self, reason):
        self.stop_base()
        self.cancel_precision_dock()
        if not self.active:
            return
        task = self.tasks[self.current_task_index]
        attempt = int(task.get('precision_dock_attempts', 0)) + 1
        task['precision_dock_attempts'] = attempt
        task['execution_status'] = 'PRECISION_DOCK_FAILED'
        for client in (
            self.local_costmap_clear_client,
            self.global_costmap_clear_client,
        ):
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
        if attempt > self.max_navigation_retries:
            self.fail(
                f'Physical docking failed for {task["object_id"]} after '
                f'{attempt} attempt(s): {reason}')
            return
        self.set_state(
            'RETRY_WAIT',
            f'{reason}; restarting safe approach {attempt}/'
            f'{self.max_navigation_retries}',
            timeout_sec=self.retry_delay_sec + 2.0,
        )
        self.cancel_retry_timer()

        def retry():
            self.cancel_retry_timer()
            if self.active:
                self.start_current_pickup()

        self.retry_timer = self.create_timer(
            self.retry_delay_sec, retry,
            callback_group=self.callback_group,
        )

    def start_manipulation(self, operation):
        if not self.active:
            return
        task = self.tasks[self.current_task_index]
        object_id = task['object_id']
        self.current_manipulation = {
            'operation': operation,
            'object_id': object_id,
            'stage': 'SENDING_GOAL',
            'progress': 0.0,
            'result': 'PENDING',
        }
        self.manipulation_goal_handle = None
        self.manipulation_generation += 1
        generation = self.manipulation_generation
        self.manipulation_started_mono = time.monotonic()
        self.manipulation_probe_pending = False

        state = 'GRASPING' if operation == 'pick' else 'PLACING'
        task['execution_status'] = state
        self.set_state(
            state,
            f'Task {self.current_task_index + 1}/{len(self.tasks)} '
            f'{operation} {object_id}',
            timeout_sec=self.manipulation_timeout_sec,
        )

        goal = ExecuteManipulation.Goal()
        goal.operation = operation
        goal.object_id = object_id
        future = self.manipulation_client.send_goal_async(
            goal,
            feedback_callback=lambda message: self.manipulation_feedback_callback(
                generation, message
            ),
        )
        future.add_done_callback(
            lambda completed: self.manipulation_goal_response_callback(
                generation, completed
            )
        )

    def manipulation_feedback_callback(self, generation, message):
        if not self.active or generation != self.manipulation_generation:
            return
        if self.current_manipulation is None:
            return
        feedback = message.feedback
        self.current_manipulation['stage'] = feedback.stage
        self.current_manipulation['progress'] = round(
            float(feedback.progress), 3
        )
        self.publish_execution_status()

    def manipulation_goal_response_callback(self, generation, future):
        if not self.active or generation != self.manipulation_generation:
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self.fail(f'Manipulation goal request failed: {error}')
            return
        if goal_handle is None or not goal_handle.accepted:
            self.fail('Manipulation goal was rejected')
            return
        self.manipulation_goal_handle = goal_handle
        self.current_manipulation['result'] = 'ACCEPTED'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self.manipulation_result_callback(
                generation, completed
            )
        )

    def manipulation_result_callback(self, generation, future):
        if not self.active or generation != self.manipulation_generation:
            return
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as error:
            self.fail(f'Manipulation result failed: {error}')
            return

        self.manipulation_started_mono = None
        operation = self.current_manipulation['operation']
        record = {
            'operation': operation,
            'object_id': self.current_manipulation['object_id'],
            'result': 'SUCCEEDED' if result.success else 'FAILED',
            'error_code': int(result.error_code),
            'message': result.message,
            'stage': self.current_manipulation.get('stage'),
        }
        self.current_manipulation = record
        self.manipulation_goal_handle = None
        task = self.tasks[self.current_task_index]

        field = (
            'pickup_manipulation'
            if operation == 'pick'
            else 'dropoff_manipulation'
        )
        task[field] = dict(record)
        if not result.success:
            task['execution_status'] = f'{operation.upper()}_FAILED'
            attempts_field = f'{operation}_attempts'
            attempt = int(task.get(attempts_field, 0)) + 1
            task[attempts_field] = attempt
            if attempt <= self.max_manipulation_retries:
                self.set_state(
                    'MANIPULATION_RETRY_WAIT',
                    f'{operation} alignment/action failed for '
                    f'{task["object_id"]}; retrying {attempt}/'
                    f'{self.max_manipulation_retries} after '
                    f'{self.retry_delay_sec:.1f}s',
                    timeout_sec=self.retry_delay_sec + 2.0,
                )
                self.schedule_manipulation_retry(operation)
                return
            self.fail(
                f'{operation} failed for {task["object_id"]}: '
                f'code={result.error_code}, {result.message}'
            )
            return

        if operation == 'pick':
            task['execution_status'] = 'OBJECT_GRASPED'
            # 载物状态下重心升高, 高速碰撞会把轻量底盘连同臂尖物块一起
            # 掀飞(实测被弹出场外)。运送段动态降速, 放置后恢复。
            self.set_mppi_speed(0.45)
            self.set_state(
                'OBJECT_GRASPED',
                f'Grasped {task["object_id"]}; navigating to '
                f'zone {task["destination"]}',
            )
            self.start_current_dropoff()
            return

        # Keep the just-placed cube as the current task (and therefore
        # excluded from the cube costmap) until the chassis has backed out of
        # its inflation radius.  Switching to the next task immediately puts
        # a cube only ~0.20 m from the base back into the costmap, so MPPI
        # correctly reports that every candidate starts in collision and the
        # robot can never leave the zone.
        self.start_post_place_departure()

    def start_post_place_departure(self):
        task = self.tasks[self.current_task_index]
        if self.post_place_departure_timer is not None:
            self.destroy_timer(self.post_place_departure_timer)
            self.post_place_departure_timer = None
        if self.latest_base_x is None or self.latest_base_y is None:
            self.fail('Cannot leave placement zone: odometry unavailable')
            return
        self.post_place_departure_start = (
            self.latest_base_x, self.latest_base_y)
        self.post_place_departure_started_mono = time.monotonic()
        self.post_place_completion_committed = False
        task['execution_status'] = 'DEPARTING_DROPOFF'
        self.set_state(
            'DEPARTING_DROPOFF',
            f'Placed {task["object_id"]}; verifying physical clearance '
            'before restoring it to the obstacle map',
            timeout_sec=8.0 + self.post_place_clearance_settle_sec,
        )
        self.post_place_departure_timer = self.create_timer(
            0.05,
            self.post_place_departure_tick,
            callback_group=self.callback_group,
        )

    def post_place_departure_tick(self):
        if not self.active or self.state != 'DEPARTING_DROPOFF':
            self.publish_zero_velocity()
            if self.post_place_departure_timer is not None:
                self.destroy_timer(self.post_place_departure_timer)
                self.post_place_departure_timer = None
            return
        if (self.latest_base_x is None or self.latest_base_y is None
                or self.post_place_departure_start is None):
            self.publish_zero_velocity()
            return
        travelled = math.hypot(
            self.latest_base_x - self.post_place_departure_start[0],
            self.latest_base_y - self.post_place_departure_start[1],
        )
        elapsed = time.monotonic() - self.post_place_departure_started_mono
        task = self.tasks[self.current_task_index]
        cube_xy = self.cube_world_xy.get(task['object_id'])
        if cube_xy is not None:
            cube_clearance = math.hypot(
                self.latest_base_x - cube_xy[0],
                self.latest_base_y - cube_xy[1],
            )
            if cube_clearance >= self.post_place_min_cube_clearance_m:
                self.publish_zero_velocity()
                task['post_place_cube_clearance_m'] = round(
                    float(cube_clearance), 3)
                self.get_logger().info(
                    f'Placed cube has {cube_clearance:.2f} m base clearance '
                    'after Gazebo truth settled; skipping blind reverse')
                self.finish_post_place_departure(travelled)
                return
        # The carry plugin secures the cube during the manipulation result
        # callback.  Give /gazebo/model_states time to publish the secured
        # pose before deciding that the old pickup coordinate is still real.
        if elapsed < self.post_place_clearance_settle_sec:
            self.publish_zero_velocity()
            return
        # The last task has no following navigation leg, so re-inserting its
        # cube into the obstacle map while the stationary base is still close
        # provides no safety benefit.  In a tight terminal zone (notably B),
        # Collision Monitor may correctly prevent the blind reverse forever.
        # The manipulation result has already physically secured the cube in
        # the zone; stop the base and commit the final task after Gazebo truth
        # has settled instead of turning that safe hold into mission failure.
        if self.current_task_index == len(self.tasks) - 1:
            self.publish_zero_velocity()
            self.get_logger().info(
                'Final task placement settled; no subsequent route requires '
                'post-place obstacle-map clearance')
            self.finish_post_place_departure(travelled)
            return
        if travelled >= self.post_place_departure_m:
            self.publish_zero_velocity()
            if self.post_place_departure_timer is not None:
                self.destroy_timer(self.post_place_departure_timer)
                self.post_place_departure_timer = None
            self.finish_post_place_departure(travelled)
            return
        if elapsed > 7.0 + self.post_place_clearance_settle_sec:
            self.publish_zero_velocity()
            self.fail(
                'Post-place departure was collision-held; refusing to '
                'restore the nearby cube to the obstacle map')
            return
        command = Twist()
        command.linear.x = -abs(self.post_place_reverse_speed_mps)
        self.precision_velocity_publisher.publish(command)

    def finish_post_place_departure(self, travelled):
        # This callback runs in a reentrant callback group.  Destroying the
        # timer prevents future invocations but cannot cancel one that has
        # already been queued.  The latch must be set before changing state or
        # task index; otherwise the queued callback completes the next task
        # without ever navigating to or manipulating it.
        if (self.post_place_completion_committed
                or not self.active
                or self.state != 'DEPARTING_DROPOFF'):
            return
        self.post_place_completion_committed = True
        if self.post_place_departure_timer is not None:
            self.destroy_timer(self.post_place_departure_timer)
            self.post_place_departure_timer = None
        task = self.tasks[self.current_task_index]
        task['execution_status'] = 'COMPLETED'
        task['post_place_departure_m'] = round(float(travelled), 3)
        cube_xy = self.cube_world_xy.get(task['object_id'])
        if (cube_xy is not None and self.latest_base_x is not None
                and self.latest_base_y is not None):
            task['post_place_cube_clearance_m'] = round(math.hypot(
                self.latest_base_x - cube_xy[0],
                self.latest_base_y - cube_xy[1]), 3)
        self.set_mppi_speed(0.60)
        self.completed_task_count += 1
        clearance = task.get('post_place_cube_clearance_m')
        if travelled < 0.02 and clearance is not None:
            completion_detail = (
                f'completed; cube clearance {clearance:.2f} m made reverse '
                'departure unnecessary')
        else:
            completion_detail = f'completed; departed {travelled:.2f} m'
        self.set_state(
            'TASK_COMPLETED',
            f'Task {self.current_task_index + 1}/{len(self.tasks)} '
            f'{completion_detail}',
        )
        self.current_task_index += 1
        if self.current_task_index >= len(self.tasks):
            self.complete_flow(
                f'Completed {self.completed_task_count} pick/place task(s)'
            )
        else:
            self.start_current_pickup()

    def finish_leg_failure(self, result):
        self.leg_history.append(self.make_leg_record(result))
        # P4 失败恢复: 先零速清场并取消残余目标, 重试时规划器基于
        # 最新状态重新出路径, 而不是顶着旧动量继续撞
        self.stop_base()
        if self.navigation_cancel_client.service_is_ready():
            self.navigation_cancel_client.call_async(Trigger.Request())
        cleared = []
        for label, client in (
            ('local', self.local_costmap_clear_client),
            ('global', self.global_costmap_clear_client),
        ):
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
                cleared.append(label)
        if cleared:
            self.get_logger().warning(
                'Navigation recovery cleared ' + '/'.join(cleared)
                + ' costmap(s); current task progress is preserved')
        if self.current_attempt <= self.max_navigation_retries:
            next_attempt = self.current_attempt + 1
            # Repeating the same goal immediately is ineffective when a
            # moving obstacle still occupies the conflict corridor.  Use a
            # bounded progressive wait after the first failure so SIPP/MPPI
            # receive a genuinely different obstacle-time state, while the
            # costmaps and task progress remain current.
            wait_delay = min(
                12.0,
                self.retry_delay_sec * max(1, self.current_attempt),
            )
            wait_state = (
                'WAIT_DYNAMIC_CLEAR'
                if self.current_attempt >= 2
                else 'RETRY_WAIT'
            )
            self.set_state(
                wait_state,
                f'{self.current_phase} leg {result}; retrying attempt '
                f'{next_attempt} after {wait_delay:.1f}s with fresh '
                'obstacle predictions',
                timeout_sec=wait_delay + 2.0,
            )
            self.schedule_retry(next_attempt, wait_delay)
            return
        task = self.tasks[self.current_task_index]
        task['execution_status'] = f'{self.current_phase}_{result}'
        self.fail(
            f'Task {self.current_task_index + 1} {self.current_phase} '
            f'failed after {self.current_attempt} attempt(s): {result}'
        )

    def schedule_retry(self, attempt, wait_delay=None):
        self.cancel_retry_timer()
        delay = (
            self.retry_delay_sec
            if wait_delay is None
            else max(0.1, float(wait_delay))
        )

        def retry():
            self.cancel_retry_timer()
            if self.active and self.expected_goal is not None:
                self.send_leg(
                    self.current_phase,
                    self.expected_goal,
                    attempt,
                )

        self.retry_timer = self.create_timer(
            delay,
            retry,
            callback_group=self.callback_group,
        )

    def schedule_manipulation_retry(self, operation):
        """Retry without driving a long navigation leg through the target.

        A failed pick leaves the chassis already inside the close-range work
        envelope.  Re-running pickup_pre from there can make Nav2 rotate over
        the cube and push it.  Keep task progress, stop all residual velocity,
        and let the collision-monitored precision dock re-establish the final
        centimetres instead.
        """
        self.cancel_retry_timer()
        self.stop_base()

        def retry():
            self.cancel_retry_timer()
            if not self.active:
                return
            if operation == 'pick':
                self.get_logger().info(
                    'Pick retry remains at the locked, grasp-window-validated '
                    'workstation; retrying manipulation without moving base.')
                self.start_manipulation('pick')
            else:
                self.start_current_dropoff()

        self.retry_timer = self.create_timer(
            self.retry_delay_sec,
            retry,
            callback_group=self.callback_group,
        )

    def cancel_retry_timer(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None

    def make_leg_record(self, result):
        feedback = self.last_navigation_status.get('feedback', {})
        return {
            'task_index': self.current_task_index,
            'phase': self.current_phase,
            'attempt': self.current_attempt,
            'target': self.expected_goal,
            'result': result,
            'navigation_time_s': feedback.get('navigation_time_s'),
            'number_of_recoveries': feedback.get(
                'number_of_recoveries', 0
            ),
        }

    def watchdog_callback(self):
        # A collision-held Nav2 action may stop publishing fresh feedback.
        # Keep the physical staging gate independent of that DDS stream.
        if (self.active and self.state == 'NAVIGATING_PICKUP'
                and self.current_phase == 'PICKUP_DOCK_STAGE'):
            position_error = self.expected_position_error()
            if position_error <= self.pickup_acceptance_m:
                self.handoff_dock_stage_proximity(position_error)
                return
        if self.active:
            self.get_logger().info(
                f'heartbeat: state={self.state}, '
                f'elapsed={time.monotonic() - self.flow_started_monotonic:.0f}s',
                throttle_duration_sec=30.0)
        if (self.active and self.state in self.NAVIGATION_STATES
                and self.seen_current_navigation
                and self.navigation_progress_mono is not None
                and time.monotonic() - self.navigation_progress_mono
                > self.navigation_stall_timeout_sec):
            stalled_for = time.monotonic() - self.navigation_progress_mono
            self.get_logger().warning(
                f'Navigation goal-progress watchdog: best remaining distance '
                f'did not improve by 0.20 m for {stalled_for:.1f}s; '
                'canceling and replanning '
                'without discarding task progress.')
            self.navigation_progress_mono = time.monotonic()
            self.stop_base()
            if self.navigation_cancel_client.service_is_ready():
                self.navigation_cancel_client.call_async(Trigger.Request())
            self.finish_leg_failure('STALLED_NO_PROGRESS')
            return
        if self.active and self.flow_started_monotonic is not None:
            elapsed = time.monotonic() - self.flow_started_monotonic
            if elapsed > self.max_flow_duration_sec:
                if self.state in self.NAVIGATION_STATES:
                    if self.navigation_cancel_client.service_is_ready():
                        self.navigation_cancel_client.call_async(
                            Trigger.Request()
                        )
                elif self.state in self.MANIPULATION_STATES:
                    if self.manipulation_goal_handle is not None:
                        self.manipulation_goal_handle.cancel_goal_async()
                self.fail(
                    f'Flow exceeded {self.max_flow_duration_sec:.0f}s limit'
                )
                return
        if not self.active or self.deadline_ns <= 0:
            return
        if self.get_clock().now().nanoseconds <= self.deadline_ns:
            return
        if self.state == 'PRECISION_DOCKING':
            self.precision_dock_failed('precision docking timed out')
            return
        if self.state in self.NAVIGATION_STATES:
            if self.navigation_cancel_client.service_is_ready():
                self.navigation_cancel_client.call_async(Trigger.Request())
            if self.current_phase in (
                    'PICKUP_PRE', 'PICKUP_DOCK_STAGE',
                    'PICKUP', 'DROPOFF'):
                self.finish_leg_failure('NAVIGATION_TIMEOUT')
                return
        elif self.state in self.MANIPULATION_STATES:
            if self.manipulation_goal_handle is not None:
                self.manipulation_goal_handle.cancel_goal_async()
        self.fail(f'Timeout while state={self.state}')

    def release_carried_cube(self):
        """通知机械臂服务器停止运动学载运(孤儿跟随防护)。"""
        if (not self.manipulation_release_client.service_is_ready()
                or getattr(self, '_release_pending', False)):
            return
        self._release_pending = True

        def on_done(_fut):
            self._release_pending = False
        self.manipulation_release_client.call_async(
            Trigger.Request()).add_done_callback(on_done)

    def complete_flow(self, detail):
        self.active = False
        self.deadline_ns = 0
        self.cancel_retry_timer()
        self.cancel_precision_dock()
        self.stop_base()
        self.cancel_nav2_wait()
        self.release_carried_cube()
        self.set_state('FLOW_COMPLETED', detail)

    def fail(self, detail):
        if not self.active and self.state == 'ERROR':
            return
        self.active = False
        self.deadline_ns = 0
        self.cancel_retry_timer()
        self.cancel_precision_dock()
        self.stop_base()
        self.cancel_nav2_wait()
        self.release_carried_cube()
        if self.manipulation_goal_handle is not None:
            self.manipulation_goal_handle.cancel_goal_async()
            self.manipulation_goal_handle = None
        self.set_state('ERROR', detail)
        self.get_logger().error(detail)

    def set_state(self, state, detail, timeout_sec=None):
        self.state = state
        self.detail = detail
        if timeout_sec is None:
            self.deadline_ns = 0
        else:
            self.deadline_ns = (
                self.get_clock().now().nanoseconds
                + int(float(timeout_sec) * 1_000_000_000)
            )
        self.get_logger().info(f'{state}: {detail}')
        self.publish_execution_status()

    def publish_execution_status(self):
        elapsed = 0.0
        if self.flow_started_monotonic is not None:
            elapsed = time.monotonic() - self.flow_started_monotonic
        current_task = None
        if 0 <= self.current_task_index < len(self.tasks):
            current_task = self.tasks[self.current_task_index]
        payload = {
            'state': self.state,
            'detail': self.detail,
            'active': self.active,
            'task_count': len(self.tasks),
            'completed_task_count': self.completed_task_count,
            'elapsed_time_s': round(elapsed, 1),
            'time_remaining_s': round(
                max(0.0, self.max_flow_duration_sec - elapsed), 1
            ),
            'current_task_index': self.current_task_index,
            'current_task': current_task,
            'current_phase': self.current_phase,
            'current_attempt': self.current_attempt,
            'current_goal': self.expected_goal,
            'navigation': self.last_navigation_status,
            'manipulation': self.current_manipulation,
            'leg_history': self.leg_history,
            'tasks': self.tasks,
            'manipulation_mode': 'EXECUTE_ACTION',
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.execution_status_publisher.publish(message)


def main(args=None):
    import sys
    import threading
    import traceback as _tb

    def _thread_excepthook(args):
        # MultiThreadedExecutor 工作线程里的回调异常会被吞掉, 导致所有
        # 定时器静默失灵(实测)。在这里显式曝光。
        import io
        buf = io.StringIO()
        _tb.print_exception(
            args.exc_type, args.exc_value, args.exc_traceback, file=buf)
        msg = f'EXECUTOR THREAD EXCEPTION:\n{buf.getvalue()}'
        print(msg, file=sys.stderr, flush=True)

    threading.excepthook = _thread_excepthook

    rclpy.init(args=args)
    node = MissionFlowExecutorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        import io
        buf = io.StringIO()
        _tb.print_exception(
            type(error), error, error.__traceback__, file=buf)
        print(f'EXECUTOR SPIN DIED:\n{buf.getvalue()}',
              file=sys.stderr, flush=True)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
