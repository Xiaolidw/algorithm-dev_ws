#!/usr/bin/env python3
"""Publish map->odom from Gazebo truth for deterministic competition simulation."""

import math

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quat_multiply(a, b):
    return (
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    )


def quat_inverse(q):
    norm = sum(value * value for value in q)
    return (-q[0] / norm, -q[1] / norm, -q[2] / norm, q[3] / norm)


def rotate(q, vector):
    pure = (vector[0], vector[1], vector[2], 0.0)
    result = quat_multiply(quat_multiply(q, pure), quat_inverse(q))
    return result[:3]


class GazeboTruthLocalizer(Node):
    def __init__(self):
        super().__init__('gazebo_truth_localizer')
        self.declare_parameter('model_name', 'six_arm')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('tf_publish_rate_hz', 40.0)
        self.declare_parameter('transform_lead_sec', 0.3)
        self.model_name = str(self.get_parameter('model_name').value)
        self.minimum_period = 1.0 / float(
            self.get_parameter('publish_rate_hz').value
        )
        self.last_publish_seconds = -math.inf
        self.transform_lead_sec = float(
            self.get_parameter('transform_lead_sec').value)
        self.latest_transform = None
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10,
        )
        self.create_timer(
            1.0 / float(self.get_parameter('tf_publish_rate_hz').value),
            self.publish_latest_transform,
        )
        self.get_logger().info(
            f'Gazebo truth localization enabled for {self.model_name}'
        )

    def model_states_callback(self, message):
        try:
            index = message.name.index(self.model_name)
        except ValueError:
            return

        now = self.get_clock().now()
        now_seconds = now.nanoseconds * 1.0e-9
        if now_seconds - self.last_publish_seconds < self.minimum_period:
            return

        try:
            odom_to_base = self.buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return

        truth = message.pose[index]
        q_map_base = (
            truth.orientation.x,
            truth.orientation.y,
            truth.orientation.z,
            truth.orientation.w,
        )
        t_map_base = (
            truth.position.x,
            truth.position.y,
            truth.position.z,
        )
        transform = odom_to_base.transform
        q_odom_base = (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        t_odom_base = (
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        )

        q_base_odom = quat_inverse(q_odom_base)
        negative_odom_base = tuple(-value for value in t_odom_base)
        t_base_odom = rotate(q_base_odom, negative_odom_base)
        q_map_odom = quat_multiply(q_map_base, q_base_odom)
        rotated = rotate(q_map_base, t_base_odom)
        t_map_odom = tuple(
            t_map_base[i] + rotated[i] for i in range(3)
        )

        output = TransformStamped()
        # Gazebo model states arrive near 10 Hz while Nav2 controls at 20 Hz.
        # Publish with a short validity lead, as localization nodes do with
        # transform_tolerance, so requests between samples remain transformable.
        output.header.stamp = (now + Duration(
            seconds=self.transform_lead_sec)).to_msg()
        output.header.frame_id = 'map'
        output.child_frame_id = 'odom'
        output.transform.translation.x = t_map_odom[0]
        output.transform.translation.y = t_map_odom[1]
        output.transform.translation.z = t_map_odom[2]
        output.transform.rotation.x = q_map_odom[0]
        output.transform.rotation.y = q_map_odom[1]
        output.transform.rotation.z = q_map_odom[2]
        output.transform.rotation.w = q_map_odom[3]
        self.latest_transform = output
        self.publish_latest_transform()
        self.last_publish_seconds = now_seconds

    def publish_latest_transform(self):
        """Keep TF available between Gazebo model-state callbacks.

        Gazebo may momentarily publish /model_states slower than the Nav2
        controller during rendering or arm motion. Re-stamping the latest
        map->odom correction lets odometry bridge that short gap instead of
        making Nav2 reuse an old command against a wall.
        """
        if self.latest_transform is None:
            return
        now = self.get_clock().now()
        self.latest_transform.header.stamp = (now + Duration(
            seconds=self.transform_lead_sec)).to_msg()
        self.broadcaster.sendTransform(self.latest_transform)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboTruthLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
