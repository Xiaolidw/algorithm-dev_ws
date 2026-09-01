#!/usr/bin/env python3
"""Synthetic RGB publisher for a repeatable perception smoke test."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class MockCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('mock_camera_node')
        self.declare_parameter('image_topic', '/mock_camera/image_raw')
        self.publisher = self.create_publisher(
            Image, self.get_parameter('image_topic').value, qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.publish_frame)
        self.get_logger().info('Mock camera publishing a red and a blue target.')

    def publish_frame(self) -> None:
        image = np.full((480, 640, 3), 35, dtype=np.uint8)
        image[140:301, 90:231] = (0, 0, 255)
        image[150:311, 390:541] = (255, 0, 0)
        message = Image()
        message.height, message.width = image.shape[:2]
        message.encoding = 'bgr8'
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = image.tobytes()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'mock_camera_optical_frame'
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = MockCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
