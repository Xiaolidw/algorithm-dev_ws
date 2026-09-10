"""Launch semantic nodes and mission coordinator."""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                semantic_launch
            )
        ),

        Node(
            package='moon_warehouse_coordinator',
            executable='costudio_mission_gateway',
            name='costudio_mission_gateway',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
