from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='moon_warehouse_perception', executable='mock_camera_node',
             name='mock_camera_node', output='screen'),
        Node(package='moon_warehouse_perception', executable='perception_node',
             name='perception_node', output='screen', parameters=[{
                 'image_topic': '/mock_camera/image_raw',
                 'detections_topic': '/perception/detections_2d',
                 'annotated_image_topic': '/perception/annotated_image',
                 'min_area_px': 500.0,
             }]),
    ])
