#!/usr/bin/env python3
"""Bridge a CoStudio question/mapping request to the validated batch runner."""

import json
import os
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


PHASE_STATES = {
    'INITIALIZING': 'PLAN_READY',
    'SELECTED': 'NAVIGATING_PICKUP',
    'NAV_PICKUP': 'NAVIGATING_PICKUP',
    'FINE_DOCK': 'PRECISION_DOCKING',
    'PICK': 'GRASPING',
    'NAV_DROPOFF': 'NAVIGATING_DROPOFF',
    'DROPOFF_FINE_APPROACH': 'DROPOFF_REACHED',
    'DROPOFF_ALIGN': 'DROPOFF_REACHED',
    'PLACE': 'PLACING',
    'COMPLETE': 'TASK_COMPLETED',
    'FAILED': 'ERROR',
}


class CoStudioMissionGateway(Node):
    def __init__(self):
        super().__init__('costudio_mission_gateway')
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, '/mission/status', latched)
        self.plan_pub = self.create_publisher(String, '/mission/plan', latched)
        self.current_pub = self.create_publisher(String, '/mission/current_task', latched)
        self.execution_pub = self.create_publisher(
            String, '/mission/execution_status', latched)
        self.question_pub = self.create_publisher(
            String, '/semantic/question_raw', 10)
        self.create_subscription(
            String, '/costudio/mission_request', self.request_callback, 10)
        self.create_subscription(
            String, '/semantic/variables', self.variables_callback, 10)
        self.create_subscription(
            String, '/pick_place/navigation_status', self.nav_callback, 10)
        self.active = False
        self.pending = None
        self.tasks = []
        self.completed = 0
        self.started_at = 0.0
        self.semantic_started_at = 0.0
        self.total_recoveries = 0
        self.process = None
        self.publish_state('IDLE', '等待 CoStudio 发布赛题与映射')
        self.get_logger().info(
            'CoStudio gateway ready on /costudio/mission_request')

    @staticmethod
    def _decode(message):
        value = json.loads(message.data)
        if not isinstance(value, dict):
            raise ValueError('request must be a JSON object')
        return value

    def request_callback(self, message):
        if self.active:
            self.publish_state('ERROR', '已有任务运行，拒绝重复发布')
            return
        try:
            request = self._decode(message)
            question = str(request.get('question', '')).strip()
            mapping = request.get('mapping')
            if not question:
                raise ValueError('question is empty')
            if not isinstance(mapping, list) or not mapping:
                raise ValueError('mapping must be a non-empty list')
            normalized = []
            seen = set()
            for row in mapping:
                variable = str(row.get('variable', '')).strip()
                color = str(row.get('color', '')).strip().lower()
                destination = str(row.get('destination', '')).strip().upper()
                if not variable or variable in seen:
                    raise ValueError('mapping variables must be unique')
                if color not in ('red', 'blue'):
                    raise ValueError(f'unsupported color: {color}')
                if destination not in ('A', 'B', 'C'):
                    raise ValueError(f'unsupported destination: {destination}')
                normalized.append({
                    'variable': variable, 'color': color,
                    'destination': destination,
                })
                seen.add(variable)
            self.pending = {'question': question, 'mapping': normalized}
            self.active = True
            self.completed = 0
            self.tasks = []
            self.total_recoveries = 0
            self.started_at = time.monotonic()
            self.semantic_started_at = self.started_at
            self.publish_state('WAITING_SEMANTIC', '本地大模型正在解析题目')
            self.question_pub.publish(String(data=question))
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.publish_state('ERROR', f'赛题请求无效：{exc}')

    def variables_callback(self, message):
        if not self.active or self.pending is None or self.process is not None:
            return
        try:
            data = self._decode(message)
            variables = data.get('variables', {})
            expected = {row['variable'] for row in self.pending['mapping']}
            if set(variables) != expected:
                raise ValueError(
                    f'变量不匹配 expected={sorted(expected)}, got={sorted(variables)}')
            tasks = []
            for row in self.pending['mapping']:
                count = variables[row['variable']]
                if isinstance(count, bool) or not isinstance(count, int) or count < 0:
                    raise ValueError(f"{row['variable']} 必须是非负整数")
                for _ in range(count):
                    tasks.append({
                        'sequence': len(tasks) + 1,
                        'object_id': f"fastest-{row['color']}",
                        'color': row['color'],
                        'destination': row['destination'],
                        'status': 'PENDING',
                        'execution_status': 'PENDING',
                    })
            if not 1 <= len(tasks) <= 5:
                raise ValueError(f'本轮物块总数必须为1至5，实际为{len(tasks)}')
            self.tasks = tasks
            self.publish_plan()
            self.publish_state(
                'PLAN_READY',
                f'本地大模型解析成功，生成 {len(tasks)} 个任务',
                variables=variables,
                semantic_elapsed_s=round(time.monotonic() - self.semantic_started_at, 3))
            batch = ','.join(
                f"{task['object_id']}:{task['destination']}" for task in tasks)
            thread = threading.Thread(
                target=self.run_batch, args=(batch,), daemon=True)
            thread.start()
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            self.active = False
            self.pending = None
            self.publish_state('ERROR', f'语义结果无效：{exc}')

    def run_batch(self, batch):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = f'/home/ros/dev_ws/logs/costudio_mission_{stamp}.log'
        command = [
            'ros2', 'run', 'moon_warehouse_coordinator',
            'navigation_pick_place_test', '--batch', batch,
        ]
        try:
            with open(log_path, 'w', encoding='utf-8') as stream:
                self.process = subprocess.Popen(
                    command, stdout=stream, stderr=subprocess.STDOUT)
                return_code = self.process.wait()
            elapsed = time.monotonic() - self.started_at
            if return_code == 0 and self.completed == len(self.tasks):
                self.publish_state(
                    'MISSION_COMPLETED',
                    f'{len(self.tasks)}/{len(self.tasks)} 完成',
                    active=False, elapsed_time_s=round(elapsed, 3),
                    recoveries=self.total_recoveries, log_path=log_path)
            else:
                self.publish_state(
                    'ERROR', f'执行器退出 code={return_code}', active=False,
                    elapsed_time_s=round(elapsed, 3), log_path=log_path)
        except OSError as exc:
            self.publish_state('ERROR', f'无法启动执行器：{exc}', active=False)
        finally:
            self.active = False
            self.pending = None
            self.process = None

    def nav_callback(self, message):
        if not self.active or not self.tasks:
            return
        try:
            data = self._decode(message)
        except (ValueError, json.JSONDecodeError):
            return
        event = str(data.get('event', ''))
        phase = str(data.get('phase', ''))
        index = min(self.completed, len(self.tasks) - 1)
        task = self.tasks[index]
        if event == 'object_selected' and data.get('object_id'):
            task['object_id'] = str(data['object_id'])
        status = PHASE_STATES.get(phase, phase or 'NAVIGATING')
        task['status'] = status
        task['execution_status'] = status
        recoveries = int(data.get('recoveries', 0) or 0)
        task['recoveries'] = max(int(task.get('recoveries', 0)), recoveries)
        self.total_recoveries = sum(
            int(item.get('recoveries', 0)) for item in self.tasks)
        if event == 'acceptance_passed':
            task['status'] = 'COMPLETED'
            task['execution_status'] = 'COMPLETED'
            self.completed += 1
            status = 'TASK_COMPLETED'
        elif event == 'acceptance_failed':
            status = 'ERROR'
        self.publish_plan()
        self.publish_state(
            status,
            f'{min(self.completed + 1, len(self.tasks))}/{len(self.tasks)} '
            f"{task['color']} → {task['destination']}",
            active=True, recoveries=self.total_recoveries)

    def publish_plan(self):
        payload = {'task_count': len(self.tasks), 'tasks': self.tasks}
        self.plan_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def publish_state(self, state, detail, **extra):
        current_index = min(self.completed, max(len(self.tasks) - 1, 0))
        current_task = self.tasks[current_index] if self.tasks else None
        payload = {
            'state': state, 'detail': detail,
            'active': extra.pop('active', self.active),
            'task_count': len(self.tasks),
            'completed_task_count': self.completed,
            'current_task_index': current_index,
            'current_task': current_task,
            'tasks': self.tasks,
            **extra,
        }
        encoded = String(data=json.dumps(payload, ensure_ascii=False))
        self.status_pub.publish(encoded)
        self.execution_pub.publish(encoded)
        self.current_pub.publish(String(data=json.dumps({
            'current_task_index': current_index,
            'current_task': current_task,
        }, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = CoStudioMissionGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
