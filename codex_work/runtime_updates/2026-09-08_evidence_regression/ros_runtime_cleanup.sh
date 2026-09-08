#!/usr/bin/env bash
# Stop only processes used by this competition stack, then clear this user's
# stale Fast DDS shared-memory locks. Run before a clean simulation restart.

source /opt/ros/humble/setup.bash
set -euo pipefail

short_names=(gazebo gzserver gzclient rviz2)
command_paths=(
  '/opt/ros/humble/bin/ros2 launch moon_warehouse_bringup mission_system.launch.py'
  '/opt/ros/humble/bin/ros2 launch moon_warehouse_bringup navigation.launch.py'
  '/opt/ros/humble/bin/ros2 launch moon_warehouse_bringup simulation.launch.py'
  'gazebo --verbose share/mybot_description/worlds/competition_v2_scoring.world'
  '/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'
  '/opt/ros/humble/lib/nav2_map_server/map_server'
  '/opt/ros/humble/lib/nav2_amcl/amcl'
  '/opt/ros/humble/lib/nav2_controller/controller_server'
  '/opt/ros/humble/lib/nav2_planner/planner_server'
  '/opt/ros/humble/lib/nav2_behaviors/behavior_server'
  '/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator'
  '/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager'
  '/opt/ros/humble/lib/nav2_velocity_smoother/velocity_smoother'
  '/opt/ros/humble/lib/nav2_collision_monitor/collision_monitor'
  '/opt/ros/humble/lib/nav2_waypoint_follower/waypoint_follower'
  '/opt/ros/humble/lib/nav2_smoother/smoother_server'
  '/opt/ros/humble/lib/gazebo_ros/spawn_entity.py'
  '/opt/ros/humble/lib/tf2_ros/static_transform_publisher'
  '/home/ros/dev_ws/install/moon_warehouse_dynamic_avoidance/lib/moon_warehouse_dynamic_avoidance/lstm_predictor_node.py'
  '/home/ros/dev_ws/install/moon_warehouse_dynamic_avoidance/lib/moon_warehouse_dynamic_avoidance/dynamic_obstacle_scan_filter.py'
  '/home/ros/dev_ws/install/moon_warehouse_dynamic_avoidance/lib/moon_warehouse_dynamic_avoidance/departure_heading_lock_node.py'
  '/home/ros/dev_ws/install/moon_warehouse_dynamic_avoidance/lib/moon_warehouse_dynamic_avoidance/sipp_decision_node.py'
  '/home/ros/dev_ws/install/moon_warehouse_dynamic_avoidance/lib/moon_warehouse_dynamic_avoidance/sipp_velocity_gate.py'
  '/home/ros/dev_ws/install/moon_warehouse_bringup/lib/moon_warehouse_bringup/safe_goal_bridge.py'
  '/home/ros/dev_ws/install/moon_warehouse_bringup/lib/moon_warehouse_bringup/cube_obstacle_map_node.py'
  '/home/ros/dev_ws/install/moon_warehouse_moveit/lib/moon_warehouse_moveit/fixed_manipulation_server'
  '/home/ros/dev_ws/install/moon_warehouse_semantic/lib/moon_warehouse_semantic/question_source_node'
  '/home/ros/dev_ws/install/moon_warehouse_semantic/lib/moon_warehouse_semantic/semantic_solver_node'
  '/home/ros/dev_ws/install/moon_warehouse_coordinator/lib/moon_warehouse_coordinator/mission_coordinator_node'
  '/home/ros/dev_ws/install/moon_warehouse_coordinator/lib/moon_warehouse_coordinator/mission_flow_executor_node'
  '/home/ros/dev_ws/install/moon_warehouse_perception/lib/moon_warehouse_perception/perception_node'
  '/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge'
  '/home/ros/dev_ws/install/tools_demo/lib/tools_demo/vision_detect'
  '/home/ros/dev_ws/src/moon_warehouse_dynamic_avoidance/tools/dynamic_avoidance_patrol_test.py'
)

managed_processes_remain() {
  local name path
  for name in "${short_names[@]}"; do
    pgrep -x "$name" >/dev/null && return 0
  done
  for path in "${command_paths[@]}"; do
    pgrep -f "$path" >/dev/null && return 0
  done
  return 1
}

echo 'Stopping moon_warehouse/Gazebo/Nav2 processes...'
pkill -INT -f '/opt/ros/humble/bin/ros2 launch moon_warehouse_bringup navigation.launch.py' 2>/dev/null || true
pkill -INT -f '/opt/ros/humble/bin/ros2 launch moon_warehouse_bringup simulation.launch.py' 2>/dev/null || true
for name in "${short_names[@]}"; do
  pkill -INT -x "$name" 2>/dev/null || true
done
for path in "${command_paths[@]}"; do
  pkill -INT -f "$path" 2>/dev/null || true
done

for _ in {1..10}; do
  if ! managed_processes_remain; then
    break
  fi
  sleep 1
done

# Lifecycle managers can remain blocked while their managed servers disappear.
# Escalate only for the exact competition-process allowlist above.
if managed_processes_remain; then
  echo 'Some managed processes ignored SIGINT; escalating to SIGTERM...'
  for name in "${short_names[@]}"; do
    pkill -TERM -x "$name" 2>/dev/null || true
  done
  for path in "${command_paths[@]}"; do
    pkill -TERM -f "$path" 2>/dev/null || true
  done
  sleep 3
fi

if managed_processes_remain; then
  echo 'Some managed processes ignored SIGTERM; forcing the allowlisted processes down...'
  for name in "${short_names[@]}"; do
    pkill -KILL -x "$name" 2>/dev/null || true
  done
  for path in "${command_paths[@]}"; do
    pkill -KILL -f "$path" 2>/dev/null || true
  done
  sleep 1
fi

if managed_processes_remain; then
  echo 'Some managed processes remain; not removing Fast DDS locks.' >&2
  echo 'Inspect with: ps -ef | grep -E "gzserver|gzclient|nav2_|rviz2|vision_detect|safe_goal_bridge"' >&2
  exit 1
fi

# The ROS 2 CLI daemon is not part of the navigation stack, but it owns the
# same Fast DDS shared-memory port files. Stop it before deleting stale lock
# pairs; otherwise a surviving sem.fastrtps_port*_mutex can make every new
# participant report open_and_lock_file failures and prevent entity spawning.
ros2 daemon stop >/dev/null 2>&1 || true
pkill -TERM -f 'ros2cli.daemon.daemonize' 2>/dev/null || true
sleep 1

find /dev/shm -maxdepth 1 -type f -user "$USER" -name 'fastrtps_port*' -delete
find /dev/shm -maxdepth 1 -type f -user "$USER" -name 'sem.fastrtps_port*' -delete
# Fast DDS also leaves per-participant data/exclusive-lock pairs named
# fastrtps_<uuid> and fastrtps_<uuid>_el.  Once every allowlisted stack
# process and the CLI daemon are confirmed down, none of these files can be
# live.  Keeping stale pairs across many restarts can split discovery so that
# processes run but only a few nodes are mutually visible.
find /dev/shm -maxdepth 1 -type f -user "$USER" -name 'fastrtps_*' -delete
echo 'PASS: managed processes stopped and stale Fast DDS locks cleared.'
