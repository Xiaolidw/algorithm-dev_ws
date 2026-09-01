#!/usr/bin/env python3
"""Publish one compact competition status stream and persist JSONL evidence."""

import json
import math
import os
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String


class RuntimeObservabilityNode(Node):
    """Aggregate task, navigation, safety and feed health for the dashboard."""

    REQUIRED_TOPICS = (
        '/camera/image_raw',
        '/perception/annotated_image',
        '/map',
        '/moon_warehouse/dynamic_obstacle_markers',
        '/mission/execution_status',
    )

    def __init__(self):
        super().__init__('runtime_observability_node')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('log_directory', '/home/ros/dev_ws/logs')

        self.started_mono = time.monotonic()
        self.mission = {}
        self.navigation = {}
        self.sipp_state = 'UNKNOWN'
        self.carry_state = 'released'
        self.base_pose = None
        self.last_camera_info_mono = None
        self.last_payload = None

        self.create_subscription(
            String, '/mission/execution_status', self._mission_cb, 10)
        self.create_subscription(
            String, '/mission/navigation_status', self._navigation_cb, 10)
        self.create_subscription(String, '/sipp/state', self._sipp_cb, 10)
        self.create_subscription(
            String, '/manipulation/carry_status', self._carry_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._camera_cb, 10)

        self.status_pub = self.create_publisher(String, '/system/status', 10)
        self.text_pub = self.create_publisher(
            String, '/system/status_text', 10)

        log_dir = str(self.get_parameter('log_directory').value)
        os.makedirs(log_dir, exist_ok=True)
        date_tag = datetime.now().strftime('%Y%m%d')
        self.log_path = os.path.join(
            log_dir, f'runtime_status_{date_tag}.jsonl')
        self.log_file = open(self.log_path, 'a', encoding='utf-8')

        rate = max(0.5, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'Runtime observability ready: /system/status -> {self.log_path}')

    @staticmethod
    def _decode(message):
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {'value': value}
        except (json.JSONDecodeError, TypeError):
            return {'value': message.data}

    def _mission_cb(self, message):
        self.mission = self._decode(message)

    def _navigation_cb(self, message):
        self.navigation = self._decode(message)

    def _sipp_cb(self, message):
        self.sipp_state = message.data.strip() or 'UNKNOWN'

    def _carry_cb(self, message):
        self.carry_state = message.data.strip() or 'released'

    def _odom_cb(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                         + orientation.z * orientation.z),
        )
        self.base_pose = {
            'x': round(float(position.x), 3),
            'y': round(float(position.y), 3),
            'yaw': round(float(yaw), 3),
        }

    def _camera_cb(self, _message):
        self.last_camera_info_mono = time.monotonic()

    def _publish(self):
        now_mono = time.monotonic()
        publishers = {
            topic: self.count_publishers(topic) > 0
            for topic in self.REQUIRED_TOPICS
        }
        camera_fresh = (
            self.last_camera_info_mono is not None
            and now_mono - self.last_camera_info_mono < 2.0
        )
        nodes = set(self.get_node_names())
        rosbridge_ready = 'rosbridge_websocket' in nodes and 'rosapi' in nodes
        required_ready = all(publishers.values()) and camera_fresh
        health = 'READY' if required_ready and rosbridge_ready else 'DEGRADED'

        payload = {
            'stamp': self.get_clock().now().nanoseconds / 1e9,
            'uptime_s': round(now_mono - self.started_mono, 1),
            'health': health,
            'mission': self.mission,
            'navigation': self.navigation,
            'sipp_state': self.sipp_state,
            'carry_state': self.carry_state,
            'base_pose': self.base_pose,
            'feeds': {
                **publishers,
                'camera_fresh': camera_fresh,
                'rosbridge_ready': rosbridge_ready,
            },
        }
        encoded = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self.status_pub.publish(String(data=encoded))

        state = self.mission.get('state', self.mission.get('value', 'IDLE'))
        detail = self.mission.get('detail', '')
        text = f'{health} | {state} | {detail}'.rstrip(' |')
        self.text_pub.publish(String(data=text))

        # Keep a replayable evidence trail without logging duplicate 2 Hz
        # samples when the robot and task state are unchanged.
        stable = dict(payload)
        stable.pop('stamp', None)
        stable.pop('uptime_s', None)
        stable_encoded = json.dumps(
            stable, ensure_ascii=False, sort_keys=True, separators=(',', ':'))
        if stable_encoded != self.last_payload:
            self.log_file.write(encoded + '\n')
            self.log_file.flush()
            self.last_payload = stable_encoded

    def destroy_node(self):
        if not self.log_file.closed:
            self.log_file.flush()
            self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RuntimeObservabilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

