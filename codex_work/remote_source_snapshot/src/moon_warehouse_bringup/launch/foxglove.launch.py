"""Launch the project Foxglove WebSocket bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('address', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'address': LaunchConfiguration('address'),
                'port': LaunchConfiguration('port'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'topic_whitelist': ['.*'],
                'service_whitelist': ['.*'],
                'param_whitelist': ['.*'],
                'client_topic_whitelist': ['.*'],
                'capabilities': [
                    'clientPublish',
                    'parameters',
                    'parametersSubscribe',
                    'services',
                    'connectionGraph',
                    'assets',
                ],
                'include_hidden': False,
                'send_buffer_limit': 10000000,
            }],
        ),
    ])
