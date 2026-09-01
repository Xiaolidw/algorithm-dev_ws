#!/usr/bin/env python3
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class FiveRunSupervisor(Node):
    def __init__(self):
        super().__init__('five_run_supervisor')
        self.target_runs = 5
        self.current_run = 1
        self.waiting_for_active = False
        self.terminal_handled = False
        self.next_run_at = None
        self.question = ''
        self.variables = {}
        self.last_state = ''
        self.output = Path(
            '/home/ros/dev_ws/runtime_logs/five_run_summary.jsonl'
        )
        self.output.write_text('', encoding='utf-8')

        self.create_subscription(
            String, '/semantic/question_raw', self.question_callback, 10
        )
        self.create_subscription(
            String, '/semantic/variables', self.variables_callback, 10
        )
        self.create_subscription(
            String,
            '/mission/execution_status',
            self.execution_callback,
            10,
        )
        self.client = self.create_client(Trigger, '/mission/run_flow')
        self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Attached to active run 1/5')

    def question_callback(self, message):
        self.question = message.data

    def variables_callback(self, message):
        try:
            self.variables = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.variables = {'raw': message.data}

    def write_summary(self, status, outcome):
        record = {
            'run': self.current_run,
            'outcome': outcome,
            'state': status.get('state'),
            'detail': status.get('detail'),
            'question': self.question,
            'variables': self.variables,
            'task_count': status.get('task_count'),
            'completed_task_count': status.get('completed_task_count'),
            'elapsed_time_s': status.get('elapsed_time_s'),
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        }
        with self.output.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + '\n')
        self.get_logger().info(
            f"RUN_RESULT {self.current_run}/{self.target_runs} "
            f"{outcome}: {record['detail']}"
        )

    def execution_callback(self, message):
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return

        state = str(status.get('state', ''))
        active = bool(status.get('active'))
        if state != self.last_state:
            self.last_state = state
            self.get_logger().info(
                f'RUN_STATE {self.current_run}/{self.target_runs} {state}'
            )

        if self.waiting_for_active:
            if active:
                self.waiting_for_active = False
            else:
                return

        if self.terminal_handled:
            return
        if state == 'FLOW_COMPLETED':
            self.terminal_handled = True
            self.write_summary(status, 'SUCCESS')
            if self.current_run < self.target_runs:
                self.next_run_at = time.monotonic() + 3.0
            else:
                self.get_logger().info('ALL_RUNS_COMPLETED')
                rclpy.shutdown()
        elif state in {'ERROR', 'STOPPED'}:
            self.terminal_handled = True
            self.write_summary(status, 'FAILED')
            self.get_logger().error('SUPERVISOR_STOPPED_ON_FAILURE')
            rclpy.shutdown()

    def timer_callback(self):
        if self.next_run_at is None or time.monotonic() < self.next_run_at:
            return
        self.next_run_at = None
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/mission/run_flow unavailable')
            rclpy.shutdown()
            return

        self.current_run += 1
        self.question = ''
        self.variables = {}
        self.last_state = ''
        self.terminal_handled = False
        self.waiting_for_active = True
        future = self.client.call_async(Trigger.Request())
        future.add_done_callback(self.run_response_callback)
        self.get_logger().info(
            f'START_REQUEST {self.current_run}/{self.target_runs}'
        )

    def run_response_callback(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Run request failed: {error}')
            rclpy.shutdown()
            return
        if not response.success:
            self.get_logger().error(
                f'Run request rejected: {response.message}'
            )
            rclpy.shutdown()


def main():
    rclpy.init()
    node = FiveRunSupervisor()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
