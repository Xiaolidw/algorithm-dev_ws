#!/usr/bin/env python3
"""Execute the semantic task queue as a navigation/manipulation flow."""

import json
import math
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from moon_warehouse_interfaces.action import ExecuteManipulation
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
        self.carry_status_subscription = self.create_subscription(
            String,
            '/manipulation/carry_status',
            self.carry_status_callback,
            10,
            callback_group=self.callback_group,
        )
        self.model_states_subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10,
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
        self.tasks = []
        self.current_task_index = 0
        self.completed_task_count = 0
        self.deadline_ns = 0
        self.expected_goal = None
        self.current_phase = None
        self.current_attempt = 0
        self.seen_current_navigation = False
        self.last_navigation_status = {}
        self.leg_history = []
        self.flow_started_monotonic = None
        self.current_manipulation = None
        self.manipulation_goal_handle = None
        self.latest_carry_status = None
        self.live_model_poses = {}

    def model_states_callback(self, message):
        self.live_model_poses = dict(zip(message.name, message.pose))

    def carry_status_callback(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(status, dict):
            return
        self.latest_carry_status = status
        if not self.active or self.state != 'NAVIGATING_DROPOFF':
            return
        if not self.tasks or self.current_task_index >= len(self.tasks):
            return
        expected = self.tasks[self.current_task_index]['object_id']
        if status.get('object_id') != expected:
            return
        if (not bool(status.get('healthy', False))
                and int(status.get('failure_count', 0)) >= 3):
            if self.navigation_cancel_client.service_is_ready():
                self.navigation_cancel_client.call_async(Trigger.Request())
            self.fail(f'OBJECT_LOST while transporting {expected}')

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

        self.reset_runtime()
        self.active = True
        self.flow_started_monotonic = time.monotonic()
        self.set_state(
            'REQUESTING_PLAN',
            'Calling /mission/start for a new official question',
            timeout_sec=self.plan_timeout_sec,
        )
        future = self.mission_start_client.call_async(Trigger.Request())
        future.add_done_callback(self.mission_start_response_callback)
        response.success = True
        response.message = 'Question-driven avoidance flow accepted'
        return response

    def stop_callback(self, request, response):
        del request
        if not self.active:
            response.success = False
            response.message = 'Flow is not active'
            return response

        self.active = False
        self.deadline_ns = 0
        self.cancel_retry_timer()
        if self.navigation_cancel_client.service_is_ready():
            self.navigation_cancel_client.call_async(Trigger.Request())
        if self.manipulation_goal_handle is not None:
            self.manipulation_goal_handle.cancel_goal_async()
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
            self.try_begin_plan()

    def try_begin_plan(self):
        if not self.active or self.state not in {
            'REQUESTING_PLAN',
            'WAITING_PLAN',
        }:
            return
        if not self.coordinator_plan_ready or self.latest_plan is None:
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
        # Re-evaluate after every completed placement/egress.  The original
        # plan remains visible, but execution swaps the nearest pending cube
        # into the current slot using the robot's live Gazebo position.
        robot = self.live_model_poses.get('six_arm')
        if robot is not None:
            ranked = []
            for index in range(self.current_task_index, len(self.tasks)):
                object_id = self.tasks[index]['object_id']
                cube = self.live_model_poses.get(object_id)
                if cube is None:
                    continue
                distance = math.hypot(
                    float(cube.position.x) - float(robot.position.x),
                    float(cube.position.y) - float(robot.position.y),
                )
                ranked.append((distance, index))
            if ranked:
                distance, nearest_index = min(ranked)
                if nearest_index != self.current_task_index:
                    self.tasks[self.current_task_index], self.tasks[nearest_index] = (
                        self.tasks[nearest_index], self.tasks[self.current_task_index])
                self.tasks[self.current_task_index][
                    'live_selection_distance_m'] = round(distance, 3)
        task = self.tasks[self.current_task_index]
        task['execution_status'] = 'NAVIGATING_TO_PICKUP'
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
        self.expected_goal = {
            'x': float(target['x']),
            'y': float(target['y']),
            'yaw': float(target['yaw']),
        }
        self.seen_current_navigation = False
        self.last_navigation_status = {}

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
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
            if phase == 'PICKUP'
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
            self.set_state(
                'OBJECT_GRASPED',
                f'Grasped {task["object_id"]}; navigating to '
                f'zone {task["destination"]}',
            )
            self.start_current_dropoff()
            return

        task['execution_status'] = 'COMPLETED'
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

    def complete_flow(self, detail):
        self.active = False
        self.deadline_ns = 0
        self.cancel_retry_timer()
        self.set_state('FLOW_COMPLETED', detail)

    def fail(self, detail):
        if not self.active and self.state == 'ERROR':
            return
        self.active = False
        self.deadline_ns = 0
        self.cancel_retry_timer()
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
    rclpy.init(args=args)
    node = MissionFlowExecutorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
