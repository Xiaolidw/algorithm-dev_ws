"""Start the yzbot Gazebo simulation with optional GUI."""

import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _remove_xml_comments(text):
    return re.sub(r'<!--(.*?)-->', '', text, flags=re.DOTALL)


def generate_launch_description():
    description_share = get_package_share_directory('mybot_description')
    urdf_path = os.path.join(
        description_share, 'urdf', 'originbot_with_rgbd_gazebo_arm.xacro')
    document = xacro.parse(open(urdf_path))
    xacro.process_doc(document)
    robot_description = _remove_xml_comments(document.toxml())

    world_path = PathJoinSubstitution([
        FindPackageShare('mybot_description'), 'worlds', LaunchConfiguration('world')])
    gui = LaunchConfiguration('gui')

    gazebo_gui = ExecuteProcess(
        condition=IfCondition(gui),
        cmd=['gazebo', '--verbose', world_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )
    gazebo_server = ExecuteProcess(
        condition=UnlessCondition(gui),
        cmd=['gzserver', '--verbose', world_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
            {'publish_frequency': 15.0},
        ],
    )
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'six_arm', '-topic', 'robot_description'],
    )

    # Use controller_manager's dedicated spawner nodes.  Shelling out through
    # `ros2 control` makes simulation startup depend on the ros2cli daemon and
    # can fail before /controller_manager discovery completes in the VM.
    joint_state = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
            '--service-call-timeout', '30.0',
            '--switch-timeout', '30.0',
        ],
        output='screen',
    )
    arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
            '--service-call-timeout', '30.0',
            '--switch-timeout', '30.0',
        ],
        output='screen',
    )
    gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
            '--service-call-timeout', '30.0',
            '--switch-timeout', '30.0',
        ],
        output='screen',
    )

    load_joint_state = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot, on_exit=[joint_state]))
    load_arm = RegisterEventHandler(OnProcessExit(
        target_action=joint_state, on_exit=[arm]))
    load_gripper = RegisterEventHandler(OnProcessExit(
        target_action=arm, on_exit=[gripper]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='competition_v2_scoring.world',
            description=(
                'Official competition world with this project\'s scoring '
                'objects and moving obstacles.'
            ),
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        load_joint_state,
        load_arm,
        load_gripper,
        gazebo_gui,
        gazebo_server,
        state_publisher,
        spawn_robot,
    ])
