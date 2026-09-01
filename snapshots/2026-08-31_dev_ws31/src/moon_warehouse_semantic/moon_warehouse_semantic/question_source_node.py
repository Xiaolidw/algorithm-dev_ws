import os
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class QuestionSourceNode(Node):

    def __init__(self):
        super().__init__('question_source_node')

        # 参数可以在启动节点时覆盖
        self.declare_parameter(
            'generator_path',
            '/home/ros/dev_ws/题目生成器/TMSCQtest_x86_x64.bin',
        )
        self.declare_parameter(
            'generator_workdir',
            '/home/ros/dev_ws/题目生成器',
        )
        self.declare_parameter('generator_timeout_sec', 5.0)

        # 发布官方程序生成的原始题目
        self.question_publisher = self.create_publisher(
            String,
            '/semantic/question_raw',
            10,
        )

        # 发布当前模块状态
        self.status_publisher = self.create_publisher(
            String,
            '/semantic/status',
            10,
        )

        # 调用该服务时生成一道题
        self.generate_service = self.create_service(
            Trigger,
            '/semantic/generate',
            self.generate_question_callback,
        )

        self.publish_status('IDLE')

        self.get_logger().info('Question source node started')
        self.get_logger().info(
            'Call /semantic/generate to generate one question'
        )

    def publish_status(self, status: str):
        message = String()
        message.data = status
        self.status_publisher.publish(message)

    def generate_question_callback(self, request, response):
        del request

        generator_path = Path(
            os.path.expanduser(
                self.get_parameter('generator_path').value
            )
        )

        generator_workdir = Path(
            os.path.expanduser(
                self.get_parameter('generator_workdir').value
            )
        )

        timeout_sec = float(
            self.get_parameter('generator_timeout_sec').value
        )

        self.publish_status('READING')
        self.get_logger().info('Running question generator')

        try:
            if not generator_path.is_file():
                raise FileNotFoundError(
                    f'Generator not found: {generator_path}'
                )

            if not os.access(generator_path, os.X_OK):
                raise PermissionError(
                    f'Generator is not executable: {generator_path}'
                )

            if not generator_workdir.is_dir():
                raise FileNotFoundError(
                    f'Working directory not found: {generator_workdir}'
                )

            result = subprocess.run(
                [str(generator_path)],
                cwd=str(generator_workdir),
                capture_output=True,
                text=True,
                encoding='utf-8',
                errors='replace',
                timeout=timeout_sec,
                check=True,
            )

            question = result.stdout.strip()

            if not question:
                raise RuntimeError(
                    'Question generator returned empty output'
                )

            question_message = String()
            question_message.data = question
            self.question_publisher.publish(question_message)

            self.publish_status('READY')

            response.success = True
            response.message = 'Question generated and published'

            self.get_logger().info(
                f'Question published, length={len(question)}'
            )

        except subprocess.TimeoutExpired:
            error = (
                f'Question generator timed out after '
                f'{timeout_sec:.1f} seconds'
            )
            self.handle_error(error, response)

        except subprocess.CalledProcessError as exc:
            stderr = (exc.stderr or '').strip()
            error = (
                f'Question generator failed, '
                f'return code={exc.returncode}, stderr={stderr}'
            )
            self.handle_error(error, response)

        except (
            FileNotFoundError,
            PermissionError,
            OSError,
            RuntimeError,
        ) as exc:
            self.handle_error(str(exc), response)

        return response

    def handle_error(self, error: str, response):
        self.publish_status('ERROR')
        self.get_logger().error(error)

        response.success = False
        response.message = error


def main(args=None):
    rclpy.init(args=args)

    node = QuestionSourceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
