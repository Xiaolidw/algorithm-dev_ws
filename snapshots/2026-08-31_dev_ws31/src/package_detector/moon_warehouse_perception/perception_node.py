#!/usr/bin/env python3
"""Publish structured red/blue 2D detections from an RGB image stream."""

from __future__ import annotations

from typing import Iterable

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from moon_warehouse_interfaces.msg import Detection2D, Detection2DArray


class PerceptionNode(Node):
    """HSV-based, camera-only colour detector with no control side effects."""

    def __init__(self) -> None:
        super().__init__('perception_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/perception/detections_2d')
        self.declare_parameter('annotated_image_topic', '/perception/annotated_image')
        self.declare_parameter('min_area_px', 500.0)
        self.declare_parameter('max_bbox_area_ratio', 0.12)
        self.declare_parameter('min_aspect_ratio', 0.65)
        self.declare_parameter('max_aspect_ratio', 1.55)
        self.declare_parameter('min_fill_ratio', 0.72)
        self.declare_parameter('min_solidity', 0.85)
        self.declare_parameter('min_confidence', 0.80)
        self.declare_parameter('process_every_n_frames', 1)
        self.declare_parameter('red_lower_1', [0, 150, 120])
        self.declare_parameter('red_upper_1', [8, 255, 255])
        self.declare_parameter('red_lower_2', [172, 150, 120])
        self.declare_parameter('red_upper_2', [179, 255, 255])
        self.declare_parameter('blue_lower', [100, 140, 100])
        self.declare_parameter('blue_upper', [130, 255, 255])
        self.declare_parameter('morphology_kernel_px', 5)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.next_id = 1
        self.min_area = float(self.get_parameter('min_area_px').value)
        self.max_bbox_area_ratio = float(
            self.get_parameter('max_bbox_area_ratio').value)
        self.min_aspect_ratio = float(
            self.get_parameter('min_aspect_ratio').value)
        self.max_aspect_ratio = float(
            self.get_parameter('max_aspect_ratio').value)
        self.min_fill_ratio = float(
            self.get_parameter('min_fill_ratio').value)
        self.min_solidity = float(
            self.get_parameter('min_solidity').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.process_every = max(1, int(self.get_parameter('process_every_n_frames').value))
        kernel_size = max(1, int(self.get_parameter('morphology_kernel_px').value))
        self.kernel = np.ones((kernel_size, kernel_size), np.uint8)
        self.ranges = {
            'red': (
                self._range('red_lower_1', 'red_upper_1'),
                self._range('red_lower_2', 'red_upper_2'),
            ),
            'blue': (self._range('blue_lower', 'blue_upper'),),
        }
        self.detections_pub = self.create_publisher(
            Detection2DArray, self.get_parameter('detections_topic').value, 10)
        self.annotated_pub = self.create_publisher(
            Image, self.get_parameter('annotated_image_topic').value, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.image_callback,
            qos_profile_sensor_data)
        self.get_logger().info('Perception ready: camera-only HSV detection with structured 2D output.')

    def _range(self, lower_name: str, upper_name: str) -> tuple[np.ndarray, np.ndarray]:
        lower = np.array(self.get_parameter(lower_name).value, dtype=np.uint8)
        upper = np.array(self.get_parameter(upper_name).value, dtype=np.uint8)
        return lower, upper

    def image_callback(self, image_msg: Image) -> None:
        self.frame_count += 1
        if self.frame_count % self.process_every:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as exc:  # cv_bridge reports varying exception classes.
            self.get_logger().error(f'Image conversion failed: {exc}')
            return
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detections = list(self._detect(hsv, image_msg))
        result = Detection2DArray()
        result.header = image_msg.header
        result.detections = detections
        self.detections_pub.publish(result)
        annotated = self._annotate(image, detections)
        annotated_msg = self._bgr_to_image(annotated, image_msg)
        self.annotated_pub.publish(annotated_msg)

    @staticmethod
    def _bgr_to_image(image: np.ndarray, source: Image) -> Image:
        """Create a BGR image message without cv_bridge's cv2_to_imgmsg path."""
        message = Image()
        message.header = source.header
        message.height, message.width = image.shape[:2]
        message.encoding = 'bgr8'
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = image.tobytes()
        return message

    def _detect(self, hsv: np.ndarray, image_msg: Image) -> Iterable[Detection2D]:
        image_area = float(hsv.shape[0] * hsv.shape[1])
        for class_name, ranges in self.ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue
                x, y, width, height = cv2.boundingRect(contour)
                bbox_area = float(width * height)
                if bbox_area > image_area * self.max_bbox_area_ratio:
                    continue

                aspect_ratio = float(width) / max(1.0, float(height))
                if not self.min_aspect_ratio <= aspect_ratio <= self.max_aspect_ratio:
                    continue

                fill_ratio = float(area) / max(1.0, bbox_area)
                if fill_ratio < self.min_fill_ratio:
                    continue

                hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
                solidity = float(area) / max(1.0, hull_area)
                if solidity < self.min_solidity:
                    continue

                # A solid, square colour patch receives high confidence. The
                # minimum prevents one strong shape metric from hiding a weak
                # one and keeps the score meaningful to the coordinator.
                confidence = min(1.0, fill_ratio, solidity)
                if confidence < self.min_confidence:
                    continue
                detection = Detection2D()
                detection.header = image_msg.header
                detection.id = self.next_id
                self.next_id += 1
                detection.class_name = class_name
                detection.confidence = confidence
                detection.x, detection.y = x, y
                detection.width, detection.height = width, height
                detection.center_x = x + width / 2.0
                detection.center_y = y + height / 2.0
                yield detection

    @staticmethod
    def _annotate(image: np.ndarray, detections: Iterable[Detection2D]) -> np.ndarray:
        output = image.copy()
        colours = {'red': (0, 0, 255), 'blue': (255, 0, 0)}
        for item in detections:
            colour = colours.get(item.class_name, (0, 255, 0))
            cv2.rectangle(output, (item.x, item.y),
                          (item.x + item.width, item.y + item.height), colour, 2)
            cv2.putText(output, f'{item.class_name} {item.confidence:.2f}',
                        (item.x, max(18, item.y - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, colour, 2, cv2.LINE_AA)
        return output


def main() -> None:
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
