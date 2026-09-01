#!/usr/bin/env python3
"""Connect the mission coordinator to the semantic module."""

import json

from ament_index_python.packages import get_package_share_directory
from moon_warehouse_interfaces.msg import Detection2DArray
import rclpy
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

        self.publish_status()
        self.publish_plan()
        self.publish_current_task()

        self.get_logger().info(
            'Mission coordinator started. '
            'Call /mission/start to begin semantic processing.'
        )

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
        self.publish_plan()
        self.publish_current_task()
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
        plan = {
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
        }

        message = String()
        message.data = json.dumps(
            status,
            ensure_ascii=False,
        )

        self.status_publisher.publish(message)


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
