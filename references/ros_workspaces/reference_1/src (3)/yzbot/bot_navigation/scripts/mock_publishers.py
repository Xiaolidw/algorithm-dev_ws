#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class MockPublishers(Node):
    def __init__(self):
        super().__init__('mock_publishers')

        # 定义三个发布器
        self.pub_target = self.create_publisher(String, 'detected_target', 10)
        self.pub_area = self.create_publisher(String, 'place_area', 10)
        self.pub_arm_status = self.create_publisher(String, 'arm_status', 10)

        self.get_logger().info("✅ 模拟发布节点已启动！")

        # 启动测试
        self.timer = self.create_timer(5.0, self.publish_messages)
        self.step = 0

    def publish_messages(self):
        if self.step == 0:
            # 模拟视觉检测目标
            target_msg = String()
            data = {"x": 1.0, "y": 1.0, "yaw": 0.0}
            target_msg.data = json.dumps(data)
            self.pub_target.publish(target_msg)
            self.get_logger().info(f"📍 发布视觉目标: {target_msg.data}")
            self.step += 1

        elif self.step == 1:
            # 模拟语言模块口令
            area_msg = String()
            area_msg.data = "A"
            self.pub_area.publish(area_msg)
            self.get_logger().info(f"🎯 发布放置区域: {area_msg.data}")
            self.step += 1

        elif self.step == 2:
            # 等待导航接近完成后，模拟机械臂反馈
            time.sleep(10)
            status_msg = String()
            status_msg.data = "DONE"
            self.pub_arm_status.publish(status_msg)
            self.get_logger().info(f"🤖 发布机械臂完成反馈: {status_msg.data}")
            self.step += 1

        else:
            self.get_logger().info("✅ 模拟任务流程完成。可在日志中查看 MissionController 的执行情况。")


def main(args=None):
    rclpy.init(args=args)
    node = MockPublishers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

