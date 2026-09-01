#!/usr/bin/env python3
"""Publish a bounded burst of AMCL initial-pose messages for simulation."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('simulation_initial_pose_publisher')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('publish_count', 8)
        self.remaining = int(self.get_parameter('publish_count').value)
        self.done = False
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos)
        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        yaw = float(self.get_parameter('yaw').value)
        message = PoseWithCovarianceStamped()
        # A zero stamp asks AMCL to use the latest available transform and
        # avoids startup extrapolation while /clock is becoming available.
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 0
        message.header.frame_id = 'map'
        message.pose.pose.position.x = float(self.get_parameter('x').value)
        message.pose.pose.position.y = float(self.get_parameter('y').value)
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        self.publisher.publish(message)
        self.remaining -= 1
        if self.remaining <= 0:
            self.timer.cancel()
            self.done = True
            self.get_logger().info('Initial pose burst completed.')


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
