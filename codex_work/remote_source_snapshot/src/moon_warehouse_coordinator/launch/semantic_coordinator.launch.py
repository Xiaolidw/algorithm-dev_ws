"""Launch semantic nodes and mission coordinator."""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node


def generate_launch_description():
    semantic_share = (
        get_package_share_directory(
            'moon_warehouse_semantic'
        )
    )

    semantic_launch = os.path.join(
        semantic_share,
        'launch',
        'semantic.launch.py',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                semantic_launch
            )
        ),

        Node(
            package='moon_warehouse_coordinator',
            executable='mission_coordinator_node',
            name='mission_coordinator_node',
            output='screen',
        ),
    ])
