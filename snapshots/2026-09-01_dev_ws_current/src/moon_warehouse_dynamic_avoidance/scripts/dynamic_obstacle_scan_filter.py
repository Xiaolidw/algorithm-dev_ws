#!/usr/bin/env python3
"""Build separate planning and hard-safety scans from the raw lidar scan.

``/scan_costmap`` removes only far-field returns belonging to a tracked moving
obstacle, preventing it from being represented twice by the costmap and MPPI.
``/scan_safety`` never removes external geometry and is reserved for Collision
Monitor.  Only returns inside the robot's own known footprint are removed from
both outputs.  Consequently a wall, cube or stone remains visible to the final
safety layer even if dynamic prediction is stale or wrong.
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
from gazebo_msgs.msg import ModelStates
from tf2_ros import Buffer, TransformException, TransformListener


class DynamicObstacleScanFilter(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_scan_filter')
        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_costmap')
        self.declare_parameter('safety_scan_topic', '/scan_safety')
        self.declare_parameter(
            'prediction_topic',
            '/moon_warehouse/dynamic_obstacle_trajectories')
        self.declare_parameter('prediction_timeout', 0.40)
        self.declare_parameter('filter_radius', 0.55)
        self.declare_parameter('near_field_passthrough_range', 1.25)
        self.declare_parameter('tf_timeout', 0.03)
        self.declare_parameter('self_footprint_x_min', -0.24)
        self.declare_parameter('self_footprint_x_max', 0.48)
        self.declare_parameter('self_footprint_y_half', 0.24)
        self.declare_parameter('self_footprint_margin', 0.01)
        # A carried cube is part of this robot, not an external collision
        # threat.  Its centre is above the floor but still intersects the
        # 2D lidar plane, so exclude only its measured silhouette from the
        # two scan streams.  Do not enlarge the generic self footprint.
        self.declare_parameter('carried_cube_z_threshold', 0.12)
        self.declare_parameter('carried_cube_filter_radius', 0.085)
        self.declare_parameter('robot_model_name', 'six_arm')

        self.input_scan_topic = str(
            self.get_parameter('input_scan_topic').value)
        self.output_scan_topic = str(
            self.get_parameter('output_scan_topic').value)
        self.safety_scan_topic = str(
            self.get_parameter('safety_scan_topic').value)
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
        margin = max(
            0.0, float(self.get_parameter('self_footprint_margin').value))
        self.self_x_min = (
            float(self.get_parameter('self_footprint_x_min').value) + margin)
        self.self_x_max = (
            float(self.get_parameter('self_footprint_x_max').value) - margin)
        self.self_y_half = max(
            0.0,
            float(self.get_parameter('self_footprint_y_half').value) - margin,
        )
        self.carried_cube_z_threshold = float(
            self.get_parameter('carried_cube_z_threshold').value)
        self.carried_cube_filter_radius = max(
            0.045,
            float(self.get_parameter('carried_cube_filter_radius').value),
        )
        self.robot_model_name = str(
            self.get_parameter('robot_model_name').value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            # Costmaps need the newest geometry, never a backlog of scans.
            # A depth of ten allowed Python/TF processing delays to replay old
            # stamps after the TF cache had advanced, so Nav2 discarded them.
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Collision Monitor is the final independent safety layer.  Give its
        # scan stream reliable delivery without changing the low-latency,
        # best-effort planning stream consumed by the costmaps.
        safety_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=3.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.predictions = None
        self.predictions_received = 0.0
        self.robot_world_pose = None
        self.carried_cube_world_poses = []
        self.publisher = self.create_publisher(
            LaserScan, self.output_scan_topic, sensor_qos)
        self.safety_publisher = self.create_publisher(
            LaserScan, self.safety_scan_topic, safety_qos)
        self.create_subscription(
            LaserScan, self.input_scan_topic, self.scan_callback, sensor_qos)
        self.create_subscription(
            ObstacleTrajectoryArray,
            self.prediction_topic,
            self.prediction_callback,
            10,
        )
        self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self.model_states_callback, 10,
        )
        self.filtered_beams = 0
        self.last_scan_stamp_ns = -1
        self.last_log = time.monotonic()
        self.get_logger().info(
            f'Dynamic scan filter ready: {self.input_scan_topic} -> '
            f'{self.output_scan_topic} (planning) + '
            f'{self.safety_scan_topic} (safety), '
            f'radius={self.filter_radius:.2f} m, '
            f'near-field passthrough={self.near_field_passthrough_range:.2f} m')

    def prediction_callback(self, message):
        self.predictions = message
        self.predictions_received = time.monotonic()

    def model_states_callback(self, message):
        robot_pose = None
        carried = []
        for name, pose in zip(message.name, message.pose):
            if name == self.robot_model_name:
                robot_pose = pose
            elif (name.startswith(('red_cube_', 'blue_cube_'))
                  and float(pose.position.z) > self.carried_cube_z_threshold):
                carried.append(pose)
        self.robot_world_pose = robot_pose
        self.carried_cube_world_poses = carried

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

    def carried_cube_centres_in_scan(self, scan_frame):
        """Return truth-backed, robot-relative carried-cube centres.

        This avoids masking any arbitrary near-field return.  We transform
        only cubes whose current Gazebo height proves that this robot is
        carrying them, then mask a radius comparable with their 9 cm body.
        """
        robot = self.robot_world_pose
        if robot is None or not self.carried_cube_world_poses:
            return []
        source_frame = 'base_footprint'
        target_frame = str(scan_frame).strip().lstrip('/')
        if not target_frame:
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
        q = robot.orientation
        robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        centres = []
        for cube in self.carried_cube_world_poses:
            dx = float(cube.position.x) - float(robot.position.x)
            dy = float(cube.position.y) - float(robot.position.y)
            base_x = math.cos(robot_yaw) * dx + math.sin(robot_yaw) * dy
            base_y = -math.sin(robot_yaw) * dx + math.cos(robot_yaw) * dy
            centres.append(self.transform_xy(base_x, base_y, transform))
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
        safety = copy.deepcopy(message)
        carried_centres = self.carried_cube_centres_in_scan(
            message.header.frame_id)
        carried_radius_squared = (
            self.carried_cube_filter_radius * self.carried_cube_filter_radius)
        # Do not use a radial cutoff here.  A radial cutoff also erases a real
        # wall exactly when the robot is closest to it.  The lidar is centred
        # on base_link, so self echoes can be identified by the asymmetric
        # robot/arm footprint in scan coordinates.
        masked = 0
        angle = float(message.angle_min)
        for index, distance in enumerate(message.ranges):
            if math.isfinite(distance):
                x = float(distance) * math.cos(angle)
                y = float(distance) * math.sin(angle)
                is_self_echo = (
                    self.self_x_min <= x <= self.self_x_max
                    and abs(y) <= self.self_y_half
                )
                is_carried_cube_echo = any(
                    (x - centre_x) ** 2 + (y - centre_y) ** 2
                    <= carried_radius_squared
                    for centre_x, centre_y in carried_centres
                )
            else:
                is_self_echo = False
                is_carried_cube_echo = False
            if is_self_echo or is_carried_cube_echo:
                output.ranges[index] = math.inf
                safety.ranges[index] = math.inf
                masked += 1
            angle += float(message.angle_increment)

        # Publish the independent hard-safety geometry before applying any
        # prediction-based filtering to the planner copy.
        self.safety_publisher.publish(safety)

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
                    # in /scan_costmap. /scan_safety retains the same object
                    # for the independent hard-stop Collision Monitor.
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
