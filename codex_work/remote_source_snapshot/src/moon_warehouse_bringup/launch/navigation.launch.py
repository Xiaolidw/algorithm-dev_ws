"""Launch the competition world, static-map Nav2/AMCL, RViz and optional YOLO."""

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
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    start_gazebo = LaunchConfiguration('start_gazebo')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    start_yolo = LaunchConfiguration('start_yolo')
    start_goal_guard = LaunchConfiguration('start_goal_guard')
    auto_initial_pose = LaunchConfiguration('auto_initial_pose')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    startup_delay = LaunchConfiguration('startup_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')
    initial_pose_delay = LaunchConfiguration('initial_pose_delay')
    yolo_delay = LaunchConfiguration('yolo_delay')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('moon_warehouse_bringup'),
            'launch', 'simulation.launch.py'])),
        condition=IfCondition(start_gazebo),
        launch_arguments={'world': world, 'gui': gazebo_gui}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'slam': 'False',
            'autostart': 'True',
            # Isolate Nav2 servers: a controller plugin failure must not take
            # map_server and AMCL down with the same component container.
            'use_composition': 'False',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(start_rviz),
        arguments=['-d', rviz_config],
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

    goal_guard = Node(
        package='moon_warehouse_bringup',
        executable='safe_goal_bridge.py',
        name='safe_goal_bridge',
        output='screen',
        condition=IfCondition(start_goal_guard),
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_goal_distance_m': 3.0,
            'min_goal_distance_m': 0.15,
            'min_x_m': -20.0,
            'max_x_m': 20.0,
            'min_y_m': -15.0,
            'max_y_m': 15.0,
        }],
    )

    delayed_nav2 = TimerAction(
        period=startup_delay,
        actions=[nav2],
    )
    delayed_rviz = TimerAction(period=rviz_delay, actions=[rviz])
    delayed_yolo = TimerAction(period=yolo_delay, actions=[yolo])

    initial_pose = TimerAction(
        period=initial_pose_delay,
        actions=[Node(
            package='moon_warehouse_bringup',
            executable='initial_pose_publisher.py',
            name='simulation_initial_pose_publisher',
            output='screen',
            condition=IfCondition(auto_initial_pose),
            parameters=[{
                'use_sim_time': use_sim_time,
                'x': initial_x,
                'y': initial_y,
                'yaw': initial_yaw,
            }],
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='competition_v2.world'),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('moon_warehouse_bringup'),
                'rviz', 'competition_nav_topdown.rviz']),
            description='RViz configuration file. Defaults to the safe top-down map view.'),
        # Keep the expensive detector opt-in so navigation can be accepted
        # independently on resource-constrained virtual machines.
        DeclareLaunchArgument('start_yolo', default_value='false'),
        DeclareLaunchArgument(
            'start_goal_guard', default_value='true',
            description='Validate RViz /goal_pose requests before sending them to Nav2.'),
        DeclareLaunchArgument('auto_initial_pose', default_value='true'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('startup_delay', default_value='7.0'),
        DeclareLaunchArgument('rviz_delay', default_value='12.0'),
        DeclareLaunchArgument('initial_pose_delay', default_value='15.0'),
        # Delay model loading until Gazebo, robot controllers and Nav2 have
        # settled; on the current virtual machine this avoids startup spikes.
        DeclareLaunchArgument('yolo_delay', default_value='40.0'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('moon_warehouse_bringup'),
                'maps', 'competition_v2_map.yaml'])),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('moon_warehouse_bringup'),
                'config', 'competition_v2_amcl_stable.yaml'])),
        gazebo,
        delayed_nav2,
        delayed_rviz,
        delayed_yolo,
        goal_guard,
        initial_pose,
    ])
