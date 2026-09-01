from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('moon_warehouse_moveit'),
        'config',
        'fixed_manipulation.yaml',
    )
    dry_run = LaunchConfiguration('dry_run')

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='false'),
        Node(
            package='moon_warehouse_moveit',
            executable='fixed_manipulation_server',
            name='fixed_manipulation_server',
            output='screen',
            parameters=[config_file, {'dry_run': False}],
            condition=UnlessCondition(dry_run),
        ),
        Node(
            package='moon_warehouse_moveit',
            executable='fixed_manipulation_server',
            name='fixed_manipulation_server',
            output='screen',
            parameters=[config_file, {'dry_run': True}],
            condition=IfCondition(dry_run),
        ),
    ])
