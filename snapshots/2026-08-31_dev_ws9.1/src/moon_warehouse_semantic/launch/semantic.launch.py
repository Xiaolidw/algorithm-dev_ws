"""Launch the semantic question source and cloud solver nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'moon_warehouse_semantic'
    )
    config_file = os.path.join(
        package_share,
        'config',
        'semantic.yaml',
    )

    question_source = Node(
        package='moon_warehouse_semantic',
        executable='question_source_node',
        name='question_source_node',
        output='screen',
        parameters=[config_file],
    )

    semantic_solver = Node(
        package='moon_warehouse_semantic',
        executable='semantic_solver_node',
        name='semantic_solver_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        question_source,
        semantic_solver,
    ])

