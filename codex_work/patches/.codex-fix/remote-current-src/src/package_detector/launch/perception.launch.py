from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution([
                FindPackageShare('moon_warehouse_perception'), 'config', 'perception.yaml']),
        ),
        Node(package='moon_warehouse_perception', executable='perception_node',
             name='perception_node', output='screen', parameters=[config]),
    ])
