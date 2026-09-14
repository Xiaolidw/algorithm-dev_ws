"""Launch the dynamic obstacle prediction chain and final safety gate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        # The configured predictor runs the deterministic route-reflection
        # model (not neural inference), so it is both cheap and required: SIPP
        # otherwise receives no obstacle trajectories at all.  Keep it enabled
        # by default and retain the argument for explicit diagnostics.
        DeclareLaunchArgument(
            'start_lstm_predictor', default_value='true'),
        Node(condition=IfCondition(LaunchConfiguration('start_lstm_predictor')),
            package='moon_warehouse_dynamic_avoidance',
            executable='lstm_predictor_node_v2.py',
            name='dynamic_obstacle_predictor_node',
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
