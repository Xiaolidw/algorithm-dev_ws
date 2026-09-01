#!/usr/bin/env python3
"""Connect the mission coordinator to the semantic module."""

import json
import math

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moon_warehouse_interfaces.msg import Detection2DArray
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


class MissionCoordinatorNode(Node):
    """Control the semantic stage of one mission."""

    ACTIVE_STATES = {
        'REQUESTING_QUESTION',
        'WAITING_SEMANTIC',
    }

    def __init__(self):
        super().__init__('mission_coordinator_node')

        self.callback_group = ReentrantCallbackGroup()

        self.state = 'IDLE'
        self.detail = 'Waiting for /mission/start'
        self.question_source_status = 'UNKNOWN'
        self.solver_status = 'UNKNOWN'
        self.variables = {}
        self.task_queue = []
        self.current_task_index = 0
        self.visual_stable_count = 0
        self.request_token = 0
        self.task_mapping = self.load_task_mapping()

        # Navigation is owned by the coordinator. External tools publish a
        # map-frame PoseStamped to /mission/navigation_goal; the coordinator
        # validates it, sends the Nav2 action and exposes feedback/results.
        self.navigation_goal_handle = None
        self.navigation_goal_token = 0
        self.navigation_active = False
        self.navigation_cancel_requested = False
        self.navigation_cancel_silent = False
        self.navigation_result = 'NONE'
        self.navigation_goal = {}
        self.navigation_feedback = {}
        self.last_navigation_feedback_publish_ns = 0

        self.declare_parameter(
            'visual_confidence_threshold',
            0.60,
        )
        self.declare_parameter(
            'visual_min_bbox_area_px',
            500,
        )
        self.declare_parameter(
            'visual_stable_frames',
            3,
        )
        self.declare_parameter(
            'navigation_action_name',
            '/navigate_to_pose',
        )

        self.visual_confidence_threshold = float(
            self.get_parameter(
                'visual_confidence_threshold'
            ).value
        )
        self.visual_min_bbox_area_px = int(
            self.get_parameter(
                'visual_min_bbox_area_px'
            ).value
        )
        self.visual_stable_frames = max(
            1,
            int(
                self.get_parameter(
                    'visual_stable_frames'
                ).value
            ),
        )
        self.navigation_action_name = str(
            self.get_parameter('navigation_action_name').value
        )

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 主控对外发布的统一状态
        self.status_publisher = self.create_publisher(
            String,
            '/mission/status',
            status_qos,
        )

        # 将经过检查的语义变量提供给后续任务规划
        self.variables_publisher = self.create_publisher(
            String,
            '/mission/variables',
            status_qos,
        )

        # 发布已排序的任务队列，供 Foxglove 和后续执行模块查看
        self.plan_publisher = self.create_publisher(
            String,
            '/mission/plan',
            status_qos,
        )

        # 单独发布当前任务，避免其他模块解析整份任务队列
        self.current_task_publisher = self.create_publisher(
            String,
            '/mission/current_task',
            status_qos,
        )

        # Dedicated, compact navigation state for Foxglove dashboards.
        self.navigation_status_publisher = self.create_publisher(
            String,
            '/mission/navigation_status',
            status_qos,
        )

        # Formal coordinator navigation input. Mission execution, Foxglove
        # and the single CLI test tool all use this same interface.
        self.navigation_goal_subscription = self.create_subscription(
            PoseStamped,
            '/mission/navigation_goal',
            self.navigation_goal_callback,
            10,
            callback_group=self.callback_group,
        )

        # 监听题目生成节点状态
        self.question_status_subscription = self.create_subscription(
            String,
            '/semantic/status',
            self.question_status_callback,
            10,
            callback_group=self.callback_group,
        )

        # 监听语义求解节点状态
        self.solver_status_subscription = self.create_subscription(
            String,
            '/semantic/solver_status',
            self.solver_status_callback,
            10,
            callback_group=self.callback_group,
        )

        # 接收大模型计算结果
        self.variables_subscription = self.create_subscription(
            String,
            '/semantic/variables',
            self.variables_callback,
            10,
            callback_group=self.callback_group,
        )

        # 主控只接收结构化二维检测，不接收高带宽相机图像
        self.visual_subscription = self.create_subscription(
            Detection2DArray,
            '/perception/detections_2d',
            self.visual_detections_callback,
            10,
            callback_group=self.callback_group,
        )

        # 调用已有的题目生成服务
        self.generate_client = self.create_client(
            Trigger,
            '/semantic/generate',
            callback_group=self.callback_group,
        )

        # Nav2's standard asynchronous navigation interface.  The
        # coordinator sends goals and receives feedback/results, while Nav2
        # remains solely responsible for planning, MPPI and obstacle avoidance.
        self.navigation_action_client = ActionClient(
            self,
            NavigateToPose,
            self.navigation_action_name,
            callback_group=self.callback_group,
        )

        # 启动一次语义任务
        self.start_service = self.create_service(
            Trigger,
            '/mission/start',
            self.start_callback,
            callback_group=self.callback_group,
        )

        # 清空当前主控状态
        self.reset_service = self.create_service(
            Trigger,
            '/mission/reset',
            self.reset_callback,
            callback_group=self.callback_group,
        )

        # 当前阶段用于独立测试视觉；以后由导航到达回调调用同一逻辑
        self.verify_visual_service = self.create_service(
            Trigger,
            '/mission/verify_visual',
            self.verify_visual_callback,
            callback_group=self.callback_group,
        )

        self.cancel_navigation_service = self.create_service(
            Trigger,
            '/mission/cancel_navigation',
            self.cancel_navigation_callback,
            callback_group=self.callback_group,
        )

        self.publish_status()
        self.publish_plan()
        self.publish_current_task()

        self.get_logger().info(
            'Mission coordinator started. '
            'Call /mission/start to begin semantic processing. '
            f'Navigation action: {self.navigation_action_name}'
        )

    def navigation_goal_callback(self, pose):
        """Validate a formal mission navigation request and forward it to Nav2."""
        if self.navigation_active:
            self.get_logger().warning(
                'Ignoring /mission/navigation_goal: a goal is already active'
            )
            return

        if self.state in self.ACTIVE_STATES:
            self.get_logger().warning(
                'Ignoring /mission/navigation_goal while semantic processing '
                f'is active: state={self.state}'
            )
            return

        if pose.header.frame_id != 'map':
            self.set_state(
                'NAVIGATION_FAILED',
                'Navigation goal frame must be map; '
                f'got {pose.header.frame_id!r}',
            )
            return

        values = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.set_state(
                'NAVIGATION_FAILED',
                'Navigation goal contains a non-finite pose value',
            )
            return

        quaternion_norm = math.sqrt(sum(
            float(value) * float(value)
            for value in (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
        ))
        if quaternion_norm < 1.0e-6:
            self.set_state(
                'NAVIGATION_FAILED',
                'Navigation goal orientation quaternion is invalid',
            )
            return

        if not self.navigation_action_client.wait_for_server(
            timeout_sec=3.0
        ):
            message = (
                f'Navigation action {self.navigation_action_name} '
                'is unavailable'
            )
            self.set_state('NAVIGATION_FAILED', message)
            return

        # 保留传入 stamp(默认0): 带仿真时间戳的目标会触发导航零速冻结
        self.send_navigation_goal(pose, purpose='MISSION_GOAL')

    def send_navigation_goal(self, pose, purpose):
        """Dispatch one NavigateToPose goal without blocking the executor."""
        self.navigation_goal_token += 1
        goal_token = self.navigation_goal_token

        self.navigation_active = True
        self.navigation_cancel_requested = False
        self.navigation_cancel_silent = False
        self.navigation_result = 'PENDING'
        self.navigation_feedback = {}
        self.last_navigation_feedback_publish_ns = 0
        self.navigation_goal = {
            'purpose': purpose,
            'frame_id': pose.header.frame_id,
            'x': round(float(pose.pose.position.x), 4),
            'y': round(float(pose.pose.position.y), 4),
            'yaw': round(
                2.0 * math.atan2(
                    float(pose.pose.orientation.z),
                    float(pose.pose.orientation.w),
                ),
                4,
            ),
        }

        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self.navigation_action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: (
                self.navigation_feedback_callback(feedback, goal_token)
            ),
        )
        future.add_done_callback(
            lambda completed: self.navigation_goal_response_callback(
                completed,
                goal_token,
            )
        )

        self.set_state(
            'NAVIGATING',
            f'Sending mission goal to {self.navigation_action_name}: '
            f'x={self.navigation_goal["x"]:.3f}, '
            f'y={self.navigation_goal["y"]:.3f}',
        )

    def navigation_goal_response_callback(self, future, goal_token):
        if goal_token != self.navigation_goal_token:
            return

        try:
            goal_handle = future.result()
        except Exception as error:
            self.finish_navigation(
                'FAILED',
                f'Navigation goal request failed: {error}',
            )
            return

        if not goal_handle.accepted:
            self.finish_navigation(
                'REJECTED',
                'Nav2 rejected the navigation goal',
            )
            return

        self.navigation_goal_handle = goal_handle
        self.navigation_result = 'ACCEPTED'

        if self.navigation_cancel_requested:
            goal_handle.cancel_goal_async()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self.navigation_result_callback(
                completed,
                goal_token,
            )
        )
        self.publish_status()

    def navigation_feedback_callback(self, feedback_message, goal_token):
        if goal_token != self.navigation_goal_token:
            return

        feedback = feedback_message.feedback
        navigation_time = self.duration_to_seconds(
            feedback.navigation_time
        )
        estimated_time = self.duration_to_seconds(
            feedback.estimated_time_remaining
        )
        distance_remaining = float(feedback.distance_remaining)

        self.navigation_feedback = {
            'distance_remaining_m': round(distance_remaining, 3),
            'navigation_time_s': round(navigation_time, 2),
            'estimated_time_remaining_s': round(estimated_time, 2),
            'number_of_recoveries': int(feedback.number_of_recoveries),
        }

        # Feedback can arrive at controller frequency.  Publish mission state
        # at most twice per second to avoid flooding Foxglove and rosbridge.
        now_ns = self.get_clock().now().nanoseconds
        if (
            self.last_navigation_feedback_publish_ns == 0
            or now_ns - self.last_navigation_feedback_publish_ns
            >= 500_000_000
        ):
            self.last_navigation_feedback_publish_ns = now_ns
            self.detail = (
                'Navigating: '
                f'{distance_remaining:.2f} m remaining, '
                f'{navigation_time:.1f} s elapsed'
            )
            self.publish_status()

    def navigation_result_callback(self, future, goal_token):
        if goal_token != self.navigation_goal_token:
            return

        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as error:
            self.finish_navigation(
                'FAILED',
                f'Navigation result failed: {error}',
            )
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            result_name = 'SUCCEEDED'
            state = 'NAVIGATION_SUCCEEDED'
            detail = 'Nav2 reached the requested mission goal'
        elif status == GoalStatus.STATUS_CANCELED:
            result_name = 'CANCELED'
            state = 'NAVIGATION_CANCELED'
            detail = 'Navigation goal was canceled'
        elif status == GoalStatus.STATUS_ABORTED:
            result_name = 'ABORTED'
            state = 'NAVIGATION_FAILED'
            detail = 'Nav2 aborted the navigation goal'
        else:
            result_name = f'STATUS_{status}'
            state = 'NAVIGATION_FAILED'
            detail = f'Navigation finished with status={status}'

        silent = self.navigation_cancel_silent
        self.navigation_result = result_name
        self.navigation_active = False
        self.navigation_goal_handle = None
        self.navigation_cancel_requested = False
        self.navigation_cancel_silent = False

        if silent:
            self.publish_status()
            return

        self.set_state(state, detail)

    def finish_navigation(self, result_name, detail):
        """Finish a goal that failed before producing a Nav2 result."""
        silent = self.navigation_cancel_silent
        self.navigation_result = result_name
        self.navigation_active = False
        self.navigation_goal_handle = None
        self.navigation_cancel_requested = False
        self.navigation_cancel_silent = False

        if silent:
            self.publish_status()
        else:
            self.set_state('NAVIGATION_FAILED', detail)

    def cancel_navigation_callback(self, request, response):
        del request

        if not self.request_navigation_cancel(silent=False):
            response.success = False
            response.message = 'No navigation goal is active'
            return response

        response.success = True
        response.message = 'Navigation cancellation requested'
        return response

    def request_navigation_cancel(self, silent):
        """Cancel an accepted goal, or remember cancellation while pending."""
        if not self.navigation_active:
            return False

        self.navigation_cancel_requested = True
        self.navigation_cancel_silent = (
            self.navigation_cancel_silent or silent
        )
        self.navigation_result = 'CANCELING'

        if self.navigation_goal_handle is not None:
            self.navigation_goal_handle.cancel_goal_async()

        self.publish_status()
        return True

    @staticmethod
    def duration_to_seconds(duration):
        return float(duration.sec) + float(duration.nanosec) * 1.0e-9

    def start_callback(self, request, response):
        del request

        if self.state in self.ACTIVE_STATES:
            response.success = False
            response.message = (
                f'Semantic task is already running: {self.state}'
            )
            return response

        if not self.generate_client.wait_for_service(
            timeout_sec=3.0
        ):
            self.set_state(
                'ERROR',
                'Service /semantic/generate is unavailable',
            )

            response.success = False
            response.message = self.detail
            return response

        self.request_token += 1
        current_token = self.request_token

        self.variables = {}
        self.task_queue = []
        self.current_task_index = 0
        self.visual_stable_count = 0
        # 不在这里发布空 plan 清除旧队列：空 plan 会与本轮真正的 plan 竞态，
        # 执行器可能先收到 PLAN_READY 再收到这份空 plan，导致提前判定
        # “空任务队列”。仪表盘通过 /mission/status 的 task_count=0 清空显示。
        self.question_source_status = 'UNKNOWN'
        self.solver_status = 'UNKNOWN'

        self.set_state(
            'REQUESTING_QUESTION',
            'Calling the official question generator',
        )

        future = self.generate_client.call_async(
            Trigger.Request()
        )

        future.add_done_callback(
            lambda completed:
            self.generate_response_callback(
                completed,
                current_token,
            )
        )

        response.success = True
        response.message = (
            'Semantic generation request accepted'
        )

        return response

    def generate_response_callback(
        self,
        future,
        request_token,
    ):
        # reset 或新任务之后，忽略旧任务回调
        if request_token != self.request_token:
            return

        try:
            result = future.result()
        except Exception as error:
            self.set_state(
                'ERROR',
                f'Generate service call failed: {error}',
            )
            return

        if not result.success:
            self.set_state(
                'ERROR',
                f'Question generation failed: '
                f'{result.message}',
            )
            return

        # 求解速度很快时，variables 可能先于服务响应到达
        if self.state not in {
            'SEMANTIC_READY',
            'PLAN_READY',
        }:
            self.set_state(
                'WAITING_SEMANTIC',
                'Question published; waiting for variables',
            )

    def question_status_callback(self, message):
        self.question_source_status = (
            message.data.strip() or 'UNKNOWN'
        )

        if (
            self.question_source_status.upper() == 'ERROR'
            and self.state in self.ACTIVE_STATES
        ):
            self.set_state(
                'ERROR',
                'Question source reported ERROR',
            )
            return

        self.publish_status()

    def solver_status_callback(self, message):
        self.solver_status = (
            message.data.strip() or 'UNKNOWN'
        )

        if (
            self.solver_status.upper() == 'ERROR'
            and self.state in self.ACTIVE_STATES
        ):
            self.set_state(
                'ERROR',
                'Semantic solver reported ERROR',
            )
            return

        self.publish_status()

    def variables_callback(self, message):
        if self.state not in self.ACTIVE_STATES:
            self.get_logger().warning(
                f'Ignoring semantic variables while '
                f'state={self.state}'
            )
            return

        try:
            semantic_result = json.loads(message.data)
            variables = semantic_result.get('variables')

            if not isinstance(variables, dict):
                raise ValueError(
                    'variables must be a JSON object'
                )

            if not variables:
                raise ValueError(
                    'variables must not be empty'
                )

            normalized_variables = {}

            for name, value in variables.items():
                if not isinstance(name, str) or not name:
                    raise ValueError(
                        'variable name is invalid'
                    )

                if (
                    isinstance(value, bool)
                    or not isinstance(value, int)
                    or value < 0
                ):
                    raise ValueError(
                        f'variable {name} must be '
                        f'a non-negative integer'
                    )

                normalized_variables[name] = value

        except (
            json.JSONDecodeError,
            TypeError,
            ValueError,
        ) as error:
            self.set_state(
                'ERROR',
                f'Invalid semantic variables: {error}',
            )
            return

        self.variables = normalized_variables

        mission_variables = {
            'variables': self.variables,
            'source': '/semantic/variables',
        }

        output_message = String()
        output_message.data = json.dumps(
            mission_variables,
            ensure_ascii=False,
        )

        self.variables_publisher.publish(
            output_message
        )

        self.set_state(
            'SEMANTIC_READY',
            f'Received {len(self.variables)} '
            f'solved variable(s)',
        )

        try:
            self.task_queue = self.build_task_queue(
                self.variables
            )
        except ValueError as error:
            self.set_state(
                'ERROR',
                f'Cannot build mission plan: {error}',
            )
            return

        self.publish_plan()

        self.current_task_index = 0
        self.visual_stable_count = 0
        self.publish_current_task()

        self.set_state(
            'PLAN_READY',
            f'Generated {len(self.task_queue)} task(s)',
        )

    def load_task_mapping(self):
        package_share = get_package_share_directory(
            'moon_warehouse_coordinator'
        )

        mapping_path = (
            package_share
            + '/config/task_mapping.yaml'
        )

        try:
            with open(
                mapping_path,
                'r',
                encoding='utf-8',
            ) as mapping_file:
                mapping_data = yaml.safe_load(
                    mapping_file
                )
        except (OSError, yaml.YAMLError) as error:
            raise RuntimeError(
                f'Cannot load task mapping: {error}'
            ) from error

        if not isinstance(mapping_data, dict):
            raise RuntimeError(
                'Task mapping root must be an object'
            )

        mission_mapping = mapping_data.get(
            'mission_mapping'
        )

        if not isinstance(mission_mapping, dict):
            raise RuntimeError(
                'mission_mapping must be an object'
            )

        return mission_mapping

    def build_task_queue(self, variables):
        expected_variables = set(
            self.task_mapping.keys()
        )
        received_variables = set(
            variables.keys()
        )

        if received_variables != expected_variables:
            raise ValueError(
                'semantic variables do not match mapping: '
                f'expected={sorted(expected_variables)}, '
                f'received={sorted(received_variables)}'
            )

        task_queue = []
        sequence = 1

        # YAML中的顺序就是任务执行顺序
        for variable_name, rule in self.task_mapping.items():
            if not isinstance(rule, dict):
                raise ValueError(
                    f'mapping for {variable_name} is invalid'
                )

            color = str(
                rule.get('color', '')
            ).strip().lower()
            destination = str(
                rule.get('destination', '')
            ).strip().upper()
            object_ids = rule.get('objects')

            if color not in {'red', 'blue'}:
                raise ValueError(
                    f'unsupported color for '
                    f'{variable_name}: {color}'
                )

            if destination not in {'A', 'B', 'C'}:
                raise ValueError(
                    f'unsupported destination for '
                    f'{variable_name}: {destination}'
                )

            if not isinstance(object_ids, list):
                raise ValueError(
                    f'objects for {variable_name} '
                    f'must be a list'
                )

            requested_count = variables[variable_name]

            if requested_count > len(object_ids):
                raise ValueError(
                    f'{variable_name} requests '
                    f'{requested_count} object(s), but only '
                    f'{len(object_ids)} are configured'
                )

            for object_id in object_ids[:requested_count]:
                task_queue.append({
                    'sequence': sequence,
                    'variable': variable_name,
                    'object_id': str(object_id),
                    'color': color,
                    'destination': destination,
                    'status': 'PENDING',
                })
                sequence += 1

        return task_queue

    def publish_plan(self):
        # plan_id 用于执行器匹配“PLAN_READY 状态 ↔ 对应 plan 消息”，
        # 防止多线程执行器乱序处理两个话题时用到陈旧的 plan。
        plan = {
            'plan_id': self.request_token,
            'task_count': len(self.task_queue),
            'tasks': self.task_queue,
        }

        message = String()
        message.data = json.dumps(
            plan,
            ensure_ascii=False,
        )

        self.plan_publisher.publish(message)

    def verify_visual_callback(self, request, response):
        del request

        if self.state != 'PLAN_READY':
            response.success = False
            response.message = (
                'Visual verification requires PLAN_READY; '
                f'current state={self.state}'
            )
            return response

        if not self.task_queue:
            response.success = False
            response.message = 'Task queue is empty'
            return response

        if self.current_task_index >= len(self.task_queue):
            response.success = False
            response.message = 'No current task is available'
            return response

        self.visual_stable_count = 0
        current_task = self.task_queue[
            self.current_task_index
        ]
        current_task['status'] = 'VERIFYING'
        self.publish_plan()
        self.publish_current_task()

        self.set_state(
            'VERIFYING_CARGO',
            f'Waiting for stable {current_task["color"]} '
            f'detection for {current_task["object_id"]}',
        )

        response.success = True
        response.message = (
            f'Visual verification started for '
            f'{current_task["object_id"]}'
        )
        return response

    def visual_detections_callback(self, message):
        if self.state != 'VERIFYING_CARGO':
            return

        if self.current_task_index >= len(self.task_queue):
            self.set_state(
                'ERROR',
                'Visual verification has no current task',
            )
            return

        current_task = self.task_queue[
            self.current_task_index
        ]
        expected_color = current_task['color']
        best_detection = None

        for detection in message.detections:
            bbox_area = int(
                detection.width
            ) * int(detection.height)

            if (
                detection.class_name.strip().lower()
                == expected_color
                and float(detection.confidence)
                >= self.visual_confidence_threshold
                and bbox_area
                >= self.visual_min_bbox_area_px
            ):
                if (
                    best_detection is None
                    or detection.confidence
                    > best_detection.confidence
                ):
                    best_detection = detection

        previous_count = self.visual_stable_count

        if best_detection is None:
            self.visual_stable_count = 0
        else:
            self.visual_stable_count += 1

        if self.visual_stable_count != previous_count:
            self.detail = (
                f'Visual {expected_color}: '
                f'{self.visual_stable_count}/'
                f'{self.visual_stable_frames} stable frame(s)'
            )
            self.publish_status()

        if (
            best_detection is None
            or self.visual_stable_count
            < self.visual_stable_frames
        ):
            return

        current_task['status'] = 'VISUAL_CONFIRMED'
        current_task['detection'] = {
            'confidence': round(
                float(best_detection.confidence),
                4,
            ),
            'x': int(best_detection.x),
            'y': int(best_detection.y),
            'width': int(best_detection.width),
            'height': int(best_detection.height),
        }

        self.publish_plan()
        self.publish_current_task()
        self.set_state(
            'VISUAL_CONFIRMED',
            f'Confirmed {expected_color} for '
            f'{current_task["object_id"]}',
        )

    def publish_current_task(self):
        current_task = None

        if (
            self.task_queue
            and self.current_task_index
            < len(self.task_queue)
        ):
            current_task = self.task_queue[
                self.current_task_index
            ]

        payload = {
            'current_task_index': self.current_task_index,
            'current_task': current_task,
        }

        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
        )
        self.current_task_publisher.publish(message)

    def reset_callback(self, request, response):
        del request

        # 使之前尚未返回的异步回调失效
        self.request_token += 1

        # Nav2 goals have their own asynchronous token.  Request cancellation
        # without allowing its late result to overwrite the reset IDLE state.
        self.request_navigation_cancel(silent=True)

        self.variables = {}
        self.task_queue = []
        self.current_task_index = 0
        self.visual_stable_count = 0
        self.publish_plan()
        self.publish_current_task()
        self.question_source_status = 'UNKNOWN'
        self.solver_status = 'UNKNOWN'

        self.set_state(
            'IDLE',
            'Mission semantic state reset',
        )

        response.success = True
        response.message = (
            'Mission semantic state reset'
        )

        return response

    def set_state(self, state, detail):
        self.state = state
        self.detail = detail

        self.publish_status()

        if state == 'ERROR':
            self.get_logger().error(
                f'{state}: {detail}'
            )
        else:
            self.get_logger().info(
                f'{state}: {detail}'
            )

    def publish_status(self):
        status = {
            'state': self.state,
            'detail': self.detail,
            'plan_id': self.request_token,
            'question_source_status':
                self.question_source_status,
            'solver_status':
                self.solver_status,
            'variables':
                self.variables,
            'task_count':
                len(self.task_queue),
            'current_task_index':
                self.current_task_index,
            'visual_stable_count':
                self.visual_stable_count,
            'navigation':
                {
                    'action_name': self.navigation_action_name,
                    'active': self.navigation_active,
                    'result': self.navigation_result,
                    'goal': self.navigation_goal,
                    'feedback': self.navigation_feedback,
                },
        }

        message = String()
        message.data = json.dumps(
            status,
            ensure_ascii=False,
        )

        self.status_publisher.publish(message)

        navigation_message = String()
        navigation_message.data = json.dumps(
            status['navigation'],
            ensure_ascii=False,
        )
        self.navigation_status_publisher.publish(navigation_message)


def main(args=None):
    rclpy.init(args=args)

    node = MissionCoordinatorNode()

    executor = MultiThreadedExecutor(
        num_threads=2
    )
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
