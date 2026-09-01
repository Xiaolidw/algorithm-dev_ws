#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """生成启动描述"""
    mybot_dir = get_package_share_directory('mybot')
    
    # 启动Gazebo仿真环境
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mybot_dir, 'launch', 'gazebo_world.launch.py')
        )
    )
    
    # 延迟启动测试节点（20秒延迟）
    test_node = ExecuteProcess(
        cmd=['ros2', 'run', 'obstacle_controller', 'simple_test'],
        output='screen'
    )
    
    delayed_test_node = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg="等待20秒后启动障碍物测试节点..."),
            test_node
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        delayed_test_node
    ])
