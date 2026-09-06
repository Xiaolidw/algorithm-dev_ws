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
    auto_initial_pose = LaunchConfiguration('auto_initial_pose')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    startup_delay = LaunchConfiguration('startup_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')
    initial_pose_delay = LaunchConfiguration('initial_pose_delay')
    yolo_delay = LaunchConfiguration('yolo_delay')
    start_dynamic_avoidance = LaunchConfiguration('start_dynamic_avoidance')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('moon_warehouse_bringup'),
            'launch', 'simulation.launch.py'])),
        condition=IfCondition(start_gazebo),
        launch_arguments={'world': world, 'gui': gazebo_gui}.items(),
    )

    dynamic_avoidance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('moon_warehouse_dynamic_avoidance'),
            'launch', 'dynamic_avoidance.launch.py'])),
        condition=IfCondition(start_dynamic_avoidance),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'dynamic_params_file': PathJoinSubstitution([
                FindPackageShare('moon_warehouse_dynamic_avoidance'),
                'config', 'dynamic_avoidance.yaml',
            ]),
        }.items(),
    )

    cube_obstacles = Node(
        package='moon_warehouse_bringup',
        executable='cube_obstacle_map_node.py',
        name='cube_obstacle_map_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'mark_radius': 0.045,
            'mark_zone_areas': False,
        }],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('moon_warehouse_bringup'),
            'launch', 'nav2_bringup_sipp_launch.py'])),
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

    # In this simulation the Gazebo and occupancy-map origins are identical.
    # Use the controller odometry directly for a deterministic map transform;
    # AMCL still runs for diagnostics but does not broadcast a competing TF.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='simulation_map_to_odom',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
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
        DeclareLaunchArgument(
            'world',
            default_value='competition_v2_scoring.world',
            description=(
                'Official competition world with this project\'s scoring '
                'objects and moving obstacles.'
            ),
        ),
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
            'start_dynamic_avoidance',
            default_value='true',
            description='Start pose tracking, prediction and final velocity safety gate.'),
        DeclareLaunchArgument('auto_initial_pose', default_value='true'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        # Gazebo controllers are normally available by 5 s on the VM.  Nav2's
        # extended bond timeout below handles a bounded load spike safely.
        DeclareLaunchArgument('startup_delay', default_value='5.0'),
        DeclareLaunchArgument('rviz_delay', default_value='12.0'),
        DeclareLaunchArgument('initial_pose_delay', default_value='10.0'),
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
        cube_obstacles,
        dynamic_avoidance,
        map_to_odom,
        delayed_nav2,
        delayed_rviz,
        delayed_yolo,
        initial_pose,
    ])
