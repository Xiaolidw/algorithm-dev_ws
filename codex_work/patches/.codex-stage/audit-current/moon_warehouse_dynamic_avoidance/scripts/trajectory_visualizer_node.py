#!/usr/bin/env python3
"""Publish compact RViz/Foxglove markers for predicted moving obstacles."""

import colorsys

import rclpy
from geometry_msgs.msg import Point
from moon_warehouse_interfaces.msg import ObstacleTrajectoryArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_trajectory_visualizer')
        self.declare_parameter(
            'prediction_topic',
            '/moon_warehouse/dynamic_obstacle_trajectories',
        )
        self.declare_parameter(
            'marker_topic',
            '/moon_warehouse/dynamic_obstacle_markers',
        )
        self.declare_parameter('risk_diameter', 1.80)
        prediction_topic = str(self.get_parameter('prediction_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)
        self.risk_diameter = max(
            0.1, float(self.get_parameter('risk_diameter').value)
        )
        self.publisher = self.create_publisher(MarkerArray, marker_topic, 10)
        self.subscription = self.create_subscription(
            ObstacleTrajectoryArray,
            prediction_topic,
            self.on_prediction,
            10,
        )
        self.get_logger().info(
            f'Prediction markers: {prediction_topic} -> {marker_topic}'
        )

    @staticmethod
    def color(index, alpha=1.0):
        red, green, blue = colorsys.hsv_to_rgb((index * 0.37) % 1.0, 0.85, 1.0)
        return red, green, blue, alpha

    @staticmethod
    def set_color(marker, rgba):
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba

    def base_marker(self, message, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header = message.header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 600_000_000
        return marker

    def on_prediction(self, message):
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)

        for index, trajectory in enumerate(message.trajectories):
            if not trajectory.future_poses:
                continue
            rgba = self.color(index)

            line = self.base_marker(message, 'predicted_path', index * 10, Marker.LINE_STRIP)
            line.scale.x = 0.055
            self.set_color(line, rgba)
            points = []
            for pose in trajectory.future_poses:
                point = Point()
                point.x = pose.position.x
                point.y = pose.position.y
                point.z = max(0.08, pose.position.z + 0.08)
                points.append(point)
            line.points = points
            output.markers.append(line)

            samples = self.base_marker(
                message, 'predicted_samples', index * 10 + 1, Marker.SPHERE_LIST
            )
            samples.scale.x = samples.scale.y = samples.scale.z = 0.10
            self.set_color(samples, rgba)
            samples.points = points
            output.markers.append(samples)

            risk = self.base_marker(
                message, 'dynamic_risk_envelope', index * 10 + 2, Marker.CYLINDER
            )
            first_pose = trajectory.future_poses[0].position
            risk.pose.position.x = first_pose.x
            risk.pose.position.y = first_pose.y
            risk.pose.position.z = 0.025
            risk.scale.x = risk.scale.y = self.risk_diameter
            risk.scale.z = 0.05
            self.set_color(risk, (rgba[0], rgba[1], rgba[2], 0.16))
            output.markers.append(risk)

            label = self.base_marker(
                message, 'dynamic_obstacle_labels', index * 10 + 3, Marker.TEXT_VIEW_FACING
            )
            label.pose.position.x = first_pose.x
            label.pose.position.y = first_pose.y
            label.pose.position.z = 1.20
            label.scale.z = 0.24
            label.text = f'{trajectory.id}  LSTM/SIPP'
            self.set_color(label, rgba)
            output.markers.append(label)

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
