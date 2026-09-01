"""Launch the dynamic obstacle prediction chain and final safety gate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('dynamic_params_file')

    common = {'use_sim_time': use_sim_time}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'dynamic_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('moon_warehouse_dynamic_avoidance'),
                'config',
                'dynamic_avoidance.yaml',
            ]),
        ),
        Node(
            package='moon_warehouse_dynamic_avoidance',
            executable='lstm_predictor_node.py',
            name='dynamic_obstacle_predictor_node',
            output='screen',
            parameters=[params_file, common],
        ),
        Node(
            package='moon_warehouse_dynamic_avoidance',
            executable='trajectory_visualizer_node.py',
            name='dynamic_obstacle_trajectory_visualizer',
            output='screen',
            parameters=[params_file, common],
        ),
        Node(
            package='moon_warehouse_dynamic_avoidance',
            executable='dynamic_obstacle_scan_filter.py',
            name='dynamic_obstacle_scan_filter',
            output='screen',
            parameters=[params_file, common],
        ),
        Node(
            package='moon_warehouse_dynamic_avoidance',
            executable='departure_heading_lock_node.py',
            name='departure_heading_lock',
            output='screen',
            parameters=[params_file, common],
        ),
        Node(
            package='moon_warehouse_dynamic_avoidance',
            executable='sipp_decision_node.py',
            name='sipp_decision_node',
            output='screen',
            parameters=[params_file, common],
        ),
        Node(
            package='moon_warehouse_dynamic_avoidance',
            executable='sipp_velocity_gate.py',
            name='sipp_velocity_gate',
            output='screen',
            parameters=[params_file, common],
        ),
    ])
