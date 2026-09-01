"""Optional development tool: rebuild a map in the competition world."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    start_gazebo = LaunchConfiguration('start_gazebo')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    start_rviz = LaunchConfiguration('start_rviz')
    start_yolo = LaunchConfiguration('start_yolo')
    startup_delay = LaunchConfiguration('startup_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')
    yolo_delay = LaunchConfiguration('yolo_delay')
    slam_params = LaunchConfiguration('slam_params_file')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('moon_warehouse_bringup'),
            'launch', 'simulation.launch.py'])),
        condition=IfCondition(start_gazebo),
        launch_arguments={'world': world, 'gui': gazebo_gui}.items(),
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(start_rviz),
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('bot_navigation'), 'rviz', 'slam.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    yolo = Node(
        package='tools_demo',
        executable='vision_detect',
        name='color_detector',
        output='screen',
        condition=IfCondition(start_yolo),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    delayed_slam = TimerAction(
        period=startup_delay,
        actions=[slam],
    )
    delayed_rviz = TimerAction(period=rviz_delay, actions=[rviz])
    delayed_yolo = TimerAction(period=yolo_delay, actions=[yolo])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='competition_v1.world'),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_yolo', default_value='false'),
        DeclareLaunchArgument('startup_delay', default_value='5.0'),
        DeclareLaunchArgument('rviz_delay', default_value='8.0'),
        DeclareLaunchArgument('yolo_delay', default_value='15.0'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('moon_warehouse_bringup'),
                'config', 'slam_toolbox_mapping.yaml'])),
        gazebo,
        delayed_slam,
        delayed_rviz,
        delayed_yolo,
    ])
