#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 定义功能包路径
    mybot_dir = get_package_share_directory('mybot')
    bot_nav_dir = get_package_share_directory('bot_navigation')
    rosbridge_dir = get_package_share_directory('rosbridge_server')

    # 2. 启动仿真环境（机械臂+底盘+传感器）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mybot_dir, 'launch', 'gazebo_world.launch.py')
        )
    )

    # 3. 启动MoveIt（机械臂规划）- 延迟5秒启动（确保仿真加载完成）
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mybot_dir, 'launch', 'my_moveit_rviz.launch.py')
        )
    )
    delay_moveit = TimerAction(
        period=5.0,
        actions=[LogInfo(msg="仿真启动中，5秒后启动MoveIt..."), moveit_launch]
    )

    # 4. 启动Nav2导航
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_nav_dir, 'launch', 'nav_bringup_gazebo.launch.py')
        )
    )

    # 5. 启动rosbridge
    rosbridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # 6. 启动语言解析模块
    command_parser_node = Node(
        package='llama_command_parser',
        executable='command_parser',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 7. 启动导航模块
    navigator_node = Node(
        package='nav_simple',
        executable='simple_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. 启动机械臂控制模块
    arm_node = Node(
        package='arm_action_integration',
        executable='arm_grab_place_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 9. 启动视觉识别模块
    vision_node = Node(
        package='package_detector',
        executable='color_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 10. 启动主控模块
    main_node = Node(
        package='main_controller',
        executable='main_controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 组装所有启动项
    return LaunchDescription([
        gazebo_launch,
        delay_moveit,  # 延迟启动MoveIt
        nav2_launch,
        rosbridge_launch,
        command_parser_node,
        navigator_node,
        arm_node,
        vision_node,
        main_node
    ])
