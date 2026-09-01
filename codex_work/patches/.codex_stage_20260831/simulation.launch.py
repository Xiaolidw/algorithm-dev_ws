"""Start the yzbot Gazebo simulation with optional GUI."""

import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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

    # Keep the physics server independent from the VMware/OpenGL client.
    # A gzclient rendering-context crash must not destroy the running task.
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )
    gazebo_gui = ExecuteProcess(
        condition=IfCondition(gui),
        cmd=['gzclient', '--verbose'],
        output='screen',
        respawn=True,
        respawn_delay=3.0,
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
        ],
        output='screen',
    )

    # 三个 spawner 并行启动: spawner 自带 controller_manager 等待超时。
    # 原先的"OnProcessExit 链"依赖上一个 spawner 退出, 但 spawner 进程在
    # 控制器激活后可能挂住不退出, 导致 gripper_controller 永远不会被生成
    # (实测机械臂抓取阶段因 /gripper_controller 不存在而失败)。
    load_controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[joint_state, arm, gripper]))

    # Gazebo owns /clock, sensors, controllers and the physics world.  Keeping
    # the remaining ROS graph alive after Gazebo exits creates a dangerous
    # false-healthy state: systemd reports active and coStudio connects, but
    # there is no camera, scan or simulation.  End the complete launch so the
    # systemd unit can restart one coherent stack.
    shutdown_if_gazebo_server_exits = RegisterEventHandler(OnProcessExit(
        target_action=gazebo_server,
        on_exit=[EmitEvent(event=Shutdown(
            reason='Gazebo server process exited'))],
    ))

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
        load_controllers,
        shutdown_if_gazebo_server_exits,
        gazebo_server,
        gazebo_gui,
        state_publisher,
        spawn_robot,
    ])
