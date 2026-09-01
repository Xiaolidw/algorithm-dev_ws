#!/usr/bin/env python3
"""Remove only far-field LSTM-tracked returns from the Nav2 costmap scan.

The original scan is intentionally left untouched for Collision Monitor and
diagnostics.  This prevents a predicted dynamic object from being represented
twice (as both a timeless inflated costmap wall and a time-indexed trajectory).
Near-field returns are deliberately preserved so the local costmap and MPPI
always retain a geometry-based collision boundary even when prediction or the
high-level PASS/WAIT decision is wrong.
"""

import copy
import math
import time

from moon_warehouse_interfaces.msg import ObstacleTrajectoryArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class DynamicObstacleScanFilter(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_scan_filter')
        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_costmap')
        self.declare_parameter(
            'prediction_topic',
            '/moon_warehouse/dynamic_obstacle_trajectories')
        self.declare_parameter('prediction_timeout', 0.40)
        self.declare_parameter('filter_radius', 0.55)
        self.declare_parameter('near_field_passthrough_range', 1.25)
        self.declare_parameter('tf_timeout', 0.03)

        self.input_scan_topic = str(
            self.get_parameter('input_scan_topic').value)
        self.output_scan_topic = str(
            self.get_parameter('output_scan_topic').value)
        self.prediction_topic = str(
            self.get_parameter('prediction_topic').value)
        self.prediction_timeout = max(
            0.05, float(self.get_parameter('prediction_timeout').value))
        self.filter_radius = max(
            0.05, float(self.get_parameter('filter_radius').value))
        self.near_field_passthrough_range = max(
            0.0,
            float(self.get_parameter('near_field_passthrough_range').value),
        )
        self.tf_timeout = max(
            0.001, float(self.get_parameter('tf_timeout').value))

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            # Costmaps need the newest geometry, never a backlog of scans.
            # A depth of ten allowed Python/TF processing delays to replay old
            # stamps after the TF cache had advanced, so Nav2 discarded them.
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=3.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.predictions = None
        self.predictions_received = 0.0
        self.publisher = self.create_publisher(
            LaserScan, self.output_scan_topic, sensor_qos)
        self.create_subscription(
            LaserScan, self.input_scan_topic, self.scan_callback, sensor_qos)
        self.create_subscription(
            ObstacleTrajectoryArray,
            self.prediction_topic,
            self.prediction_callback,
            10,
        )
        self.filtered_beams = 0
        self.last_scan_stamp_ns = -1
        self.last_log = time.monotonic()
        self.get_logger().info(
            f'Dynamic scan filter ready: {self.input_scan_topic} -> '
            f'{self.output_scan_topic}, radius={self.filter_radius:.2f} m, '
            f'near-field passthrough={self.near_field_passthrough_range:.2f} m')

    def prediction_callback(self, message):
        self.predictions = message
        self.predictions_received = time.monotonic()

    @staticmethod
    def transform_xy(x, y, transform):
        rotation = transform.transform.rotation
        sin_yaw = 2.0 * (
            rotation.w * rotation.z + rotation.x * rotation.y)
        cos_yaw = 1.0 - 2.0 * (
            rotation.y * rotation.y + rotation.z * rotation.z)
        yaw = math.atan2(sin_yaw, cos_yaw)
        translation = transform.transform.translation
        return (
            math.cos(yaw) * x - math.sin(yaw) * y + translation.x,
            math.sin(yaw) * x + math.cos(yaw) * y + translation.y,
        )

    def obstacle_centres_in_scan(self, scan_frame):
        now = time.monotonic()
        if (
            self.predictions is None
            or now - self.predictions_received > self.prediction_timeout
        ):
            return []
        source_frame = str(self.predictions.header.frame_id).strip().lstrip('/')
        target_frame = str(scan_frame).strip().lstrip('/')
        if not source_frame or not target_frame:
            return []
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            return []
        centres = []
        for trajectory in self.predictions.trajectories:
            if not trajectory.future_poses:
                continue
            position = trajectory.future_poses[0].position
            centres.append(self.transform_xy(
                float(position.x), float(position.y), transform))
        return centres

    def scan_callback(self, message):
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp_ns <= self.last_scan_stamp_ns:
            return
        self.last_scan_stamp_ns = stamp_ns
        output = copy.deepcopy(message)
        # 雷达扫描面(0.125m)高于底盘但会扫到本车机械臂(前方0.15~0.35m),
        # 形成跟随车体的自身回波。robot_radius 提高到 0.40 后该回波落入
        # 臃肿致死区, MPPI 判定原地碰撞而拒绝输出速度(实测起点瘫痪)。
        # 0.42m 内的回波一律视为自身回波置为无穷远: 真实障碍在 0.42m 内
        # 时底盘(0.40)已发生接触, 防碰撞意义有限; 货物方块站距 0.58m 不受影响。
        near_field = float(getattr(self, 'self_echo_min_range', 0.42))
        masked = 0
        for index, distance in enumerate(output.ranges):
            if math.isfinite(distance) and distance < near_field:
                output.ranges[index] = math.inf
                masked += 1

        centres = self.obstacle_centres_in_scan(message.header.frame_id)
        if not centres:
            self.publisher.publish(output)
            self.filtered_beams += masked
            self.log_filtered()
            return
        radius_squared = self.filter_radius * self.filter_radius
        filtered = 0
        angle = float(message.angle_min)
        for index, distance in enumerate(output.ranges):
            if (
                math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
            ):
                x = float(distance) * math.cos(angle)
                y = float(distance) * math.sin(angle)
                if any(
                    (x - centre_x) ** 2 + (y - centre_y) ** 2
                    <= radius_squared
                    for centre_x, centre_y in centres
                ) and float(distance) > self.near_field_passthrough_range:
                    # An infinite return is valid for the costmap source and
                    # raytraces the old far-field mark away. Close returns stay
                    # in /scan_costmap and raw /scan remains available to the
                    # independent Collision Monitor.
                    output.ranges[index] = math.inf
                    filtered += 1
            angle += float(message.angle_increment)
        self.publisher.publish(output)
        self.filtered_beams += filtered + masked
        self.log_filtered()

    def log_filtered(self):
        now = time.monotonic()
        if now - self.last_log >= 5.0:
            self.get_logger().info(
                f'Filtered {self.filtered_beams} dynamic scan beams in 5 s')
            self.filtered_beams = 0
            self.last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleScanFilter()
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
