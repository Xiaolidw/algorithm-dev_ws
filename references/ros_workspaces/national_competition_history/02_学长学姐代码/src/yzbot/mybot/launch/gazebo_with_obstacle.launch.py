#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取包的路径
    mybot_dir = get_package_share_directory('mybot')
    
    # 1. 启动Gazebo仿真环境
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mybot_dir, 'launch', 'gazebo_world.launch.py')
        )
    )
    
    # 2. 启动障碍物控制器（延迟5秒启动，确保Gazebo完全加载）
    obstacle_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'obstacle_controller', 'move_obstacle'],
        output='screen'
    )
    
    # 添加更长的延迟，确保障碍物控制器在Gazebo完全启动后再启动
    # 特别是第一次启动时，Gazebo需要下载模型，可能需要更长时间
    delayed_obstacle_controller = TimerAction(
        period=5.0,  # 延迟5秒（可以根据需要调整）
        actions=[
            LogInfo(msg="等待Gazebo完全启动，10秒后启动障碍物控制器..."),
            obstacle_controller
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        delayed_obstacle_controller
    ])
