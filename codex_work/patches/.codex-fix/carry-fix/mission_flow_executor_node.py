#!/usr/bin/env python3
"""Execute the semantic task queue as a navigation/manipulation flow."""

import json
import math
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
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
        self.destinations = self.config['destinations']
        self.plan_timeout_sec = float(
            self.config.get('plan_timeout_sec', 45.0)
        )
        self.navigation_timeout_sec = float(
            self.config.get('navigation_timeout_sec', 180.0)
        )
        self.max_navigation_retries = int(
            self.config.get('max_navigation_retries', 1)
        )
        self.retry_delay_sec = float(
            self.config.get('retry_delay_sec', 2.0)
        )
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
        self.manipulation_client = ActionClient(
            self,
            ExecuteManipulation,
            '/manipulation/execute',
            callback_group=self.callback_group,
        )
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
        self.retry_timer = None
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
        self.leg_history = []
        self.flow_started_monotonic = None
        self.manipulation_started_mono = None
        self.manipulation_probe_pending = False
        self.manipulation_probe_since = None
        self.current_manipulation = None
        self.manipulation_goal_handle = None
        self.nav2_state_client = None
        self.nav2_state_future = None
        self.nav2_poll_timer = None

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
        # 直接进入流程。曾用 bt_navigator 生命周期轮询做启动门控, 但本机
        # DDS 偶发应答丢失会让轮询/定时器整体失灵(仿真时钟停摆), 流程卡死。
        # Nav2 未就绪时目标会被拒绝, 由 max_navigation_retries 重试兜底。
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
        self.set_mppi_speed(0.25)
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
        self.set_mppi_speed(0.45)
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
        if getattr(self, 'nav2_poll_timer', None) is not None:
            self.nav2_poll_timer.cancel()
            self.destroy_timer(self.nav2_poll_timer)
            self.nav2_poll_timer = None
        if getattr(self, 'nav2_state_client', None) is not None:
            self.destroy_client(self.nav2_state_client)
            self.nav2_state_client = None
        self.nav2_state_future = None

    def stop_callback(self, request, response):
        del request
        if not self.active:
            response.success = False
            response.message = 'Flow is not active'
            return response

        self.active = False
        self.deadline_ns = 0
        self.cancel_retry_timer()
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
        target = self.object_approaches[task['object_id']]
        self.send_leg('PICKUP', target, attempt=1)

    def start_current_dropoff(self):
        task = self.tasks[self.current_task_index]
        task['execution_status'] = 'NAVIGATING_TO_DROPOFF'
        target = self.destinations[str(task['destination']).upper()]
        self.send_leg('DROPOFF', target, attempt=1)

    def send_leg(self, phase, target, attempt):
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
            if phase in ('PICKUP', 'PICKUP_PRE')
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

        result = str(status.get('result', 'NONE')).upper()
        active = bool(status.get('active', False))
        if active or result in {'PENDING', 'ACCEPTED', 'CANCELING'}:
            self.seen_current_navigation = True
            self.last_navigation_status = status
            self.publish_execution_status()
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
        if self.current_phase == 'PICKUP_PRE':
            task['prepickup_navigation'] = record
            self.set_state(
                'NAVIGATING_PICKUP',
                f'Pre-approach reached for {task["object_id"]}; '
                'driving the straight final approach',
            )
            target = self.object_approaches[task['object_id']]
            self.send_leg('PICKUP', target, attempt=1)
            return
        if self.current_phase == 'PICKUP':
            task['pickup_navigation'] = record
            task['execution_status'] = 'PICKUP_REACHED'
            self.set_state(
                'PICKUP_REACHED',
                f'Reached {task["object_id"]}; starting grasp action',
            )
            self.start_manipulation('pick')
            return

        task['dropoff_navigation'] = record
        task['execution_status'] = 'DROPOFF_REACHED'
        self.set_state(
            'DROPOFF_REACHED',
            f'Reached zone {task["destination"]}; starting place action',
        )
        self.start_manipulation('place')

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
            self.fail(
                f'{operation} failed for {task["object_id"]}: '
                f'code={result.error_code}, {result.message}'
            )
            return

        if operation == 'pick':
            task['execution_status'] = 'OBJECT_GRASPED'
            # 载物状态下重心升高, 高速碰撞会把轻量底盘连同臂尖物块一起
            # 掀飞(实测被弹出场外)。运送段动态降速, 放置后恢复。
            self.set_mppi_speed(0.25)
            self.set_state(
                'OBJECT_GRASPED',
                f'Grasped {task["object_id"]}; navigating to '
                f'zone {task["destination"]}',
            )
            self.start_current_dropoff()
            return

        task['execution_status'] = 'COMPLETED'
        self.set_mppi_speed(0.45)
        self.completed_task_count += 1
        self.set_state(
            'TASK_COMPLETED',
            f'Task {self.current_task_index + 1}/{len(self.tasks)} '
            'pick/place completed',
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
        zero = Twist()
        self.zero_vel_publisher.publish(zero)
        if self.navigation_cancel_client.service_is_ready():
            self.navigation_cancel_client.call_async(Trigger.Request())
        if self.current_attempt <= self.max_navigation_retries:
            next_attempt = self.current_attempt + 1
            self.set_state(
                'RETRY_WAIT',
                f'{self.current_phase} leg {result}; retrying attempt '
                f'{next_attempt} after {self.retry_delay_sec:.1f}s',
                timeout_sec=self.retry_delay_sec + 2.0,
            )
            self.schedule_retry(next_attempt)
            return
        task = self.tasks[self.current_task_index]
        task['execution_status'] = f'{self.current_phase}_{result}'
        self.fail(
            f'Task {self.current_task_index + 1} {self.current_phase} '
            f'failed after {self.current_attempt} attempt(s): {result}'
        )

    def schedule_retry(self, attempt):
        self.cancel_retry_timer()

        def retry():
            self.cancel_retry_timer()
            if self.active and self.expected_goal is not None:
                self.send_leg(
                    self.current_phase,
                    self.expected_goal,
                    attempt,
                )

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
        if self.active:
            self.get_logger().info(
                f'heartbeat: state={self.state}, '
                f'elapsed={time.monotonic() - self.flow_started_monotonic:.0f}s',
                throttle_duration_sec=30.0)
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
        if self.state in self.NAVIGATION_STATES:
            if self.navigation_cancel_client.service_is_ready():
                self.navigation_cancel_client.call_async(Trigger.Request())
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
        self.cancel_nav2_wait()
        self.release_carried_cube()
        self.set_state('FLOW_COMPLETED', detail)

    def fail(self, detail):
        if not self.active and self.state == 'ERROR':
            return
        self.active = False
        self.deadline_ns = 0
        self.cancel_retry_timer()
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
