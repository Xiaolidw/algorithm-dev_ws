import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from moon_warehouse_semantic.providers import (
    DeepSeekProvider,
    ProviderError,
)


class SemanticSolverNode(Node):

    def __init__(self):
        super().__init__('semantic_solver_node')

        self.declare_parameter(
            'base_url',
            'https://api.deepseek.com',
        )
        self.declare_parameter(
            'model',
            'deepseek-v4-flash',
        )
        self.declare_parameter(
            'request_timeout_sec',
            30.0,
        )
        # 比赛现场网络抖动很常见: 超时/限流直接放弃会毁掉整轮任务,
        # 因此带退避重试, 仅在最后一次仍失败才发布 ERROR。
        self.declare_parameter(
            'solve_attempts',
            3,
        )
        self.declare_parameter(
            'retry_backoff_sec',
            2.0,
        )

        self.provider = DeepSeekProvider(
            base_url=self.get_parameter('base_url').value,
            model=self.get_parameter('model').value,
            timeout_sec=float(
                self.get_parameter('request_timeout_sec').value
            ),
        )

        self.variables_publisher = self.create_publisher(
            String,
            '/semantic/variables',
            10,
        )

        self.status_publisher = self.create_publisher(
            String,
            '/semantic/solver_status',
            10,
        )

        self.question_subscription = self.create_subscription(
            String,
            '/semantic/question_raw',
            self.question_callback,
            10,
        )

        self.publish_status('IDLE')

        self.get_logger().info('Semantic solver node started')
        self.get_logger().info(
            'Waiting for /semantic/question_raw'
        )

    def publish_status(self, status: str):
        message = String()
        message.data = status
        self.status_publisher.publish(message)

    def question_callback(self, message: String):
        question = message.data.strip()

        if not question:
            self.publish_status('ERROR')
            self.get_logger().error('Received an empty question')
            return

        self.publish_status('SOLVING')
        self.get_logger().info('Sending question to DeepSeek')

        attempts = max(1, int(self.get_parameter('solve_attempts').value))
        backoff = float(self.get_parameter('retry_backoff_sec').value)

        result = None
        for attempt in range(1, attempts + 1):
            try:
                result = self.provider.solve(question)
                break
            except ProviderError as exc:
                last_error = str(exc)
                if attempt >= attempts:
                    self.publish_status('ERROR')
                    self.get_logger().error(
                        f'Solve failed after {attempts} attempt(s): '
                        f'{last_error}'
                    )
                    return
                self.publish_status('RETRYING')
                self.get_logger().warning(
                    f'Solve attempt {attempt}/{attempts} failed ({last_error}); '
                    f'retrying in {backoff:.1f}s'
                )
                self._sleep(backoff * attempt)
            except Exception as exc:
                self.publish_status('ERROR')
                self.get_logger().error(
                    f'Unexpected solver error: {exc}'
                )
                return

        if result is None:
            return

        output = {
            'variables': result.variables,
            'provider': result.provider,
            'model': result.model,
        }

        output_message = String()
        output_message.data = json.dumps(
            output,
            ensure_ascii=False,
        )

        self.variables_publisher.publish(output_message)
        self.publish_status('SOLVED')

        self.get_logger().info(
            f'Variables solved: {result.variables}'
        )

    def _sleep(self, seconds: float):
        import time
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    node = SemanticSolverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
