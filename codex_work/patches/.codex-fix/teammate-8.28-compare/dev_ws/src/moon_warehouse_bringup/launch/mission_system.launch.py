"""Launch simulation, Nav2, semantic, perception, coordinator and Foxglove."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    start_rviz = LaunchConfiguration('start_rviz')
    start_perception = LaunchConfiguration('start_perception')
    start_foxglove = LaunchConfiguration('start_foxglove')
    start_manipulation = LaunchConfiguration('start_manipulation')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('moon_warehouse_bringup'),
            'launch',
            'navigation.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'gazebo_gui': gazebo_gui,
            'start_rviz': start_rviz,
            'start_yolo': 'false',
        }.items(),
    )

    semantic_and_coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('moon_warehouse_coordinator'),
            'launch',
            'semantic_coordinator.launch.py',
        ])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('moon_warehouse_moveit'),
            'launch',
            'fixed_manipulation.launch.py',
        ])),
        condition=IfCondition(start_manipulation),
        launch_arguments={'dry_run': 'false'}.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('moon_warehouse_perception'),
            'launch',
            'perception.launch.py',
        ])),
        condition=IfCondition(start_perception),
    )

    foxglove = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('moon_warehouse_bringup'),
            'launch',
            'foxglove.launch.py',
        ])),
        condition=IfCondition(start_foxglove),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value='competition_v2_scoring.world',
            description=(
                'Official competition world with this project\'s scoring '
                'objects and moving obstacles.'
            ),
        ),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('start_perception', default_value='true'),
        DeclareLaunchArgument('start_foxglove', default_value='true'),
        DeclareLaunchArgument('start_manipulation', default_value='true'),
        navigation,
        manipulation,
        semantic_and_coordinator,
        perception,
        foxglove,
    ])
