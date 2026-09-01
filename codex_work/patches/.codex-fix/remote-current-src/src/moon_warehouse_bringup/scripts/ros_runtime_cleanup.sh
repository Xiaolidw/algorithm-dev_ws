#!/usr/bin/env bash
# Stop only processes used by this competition stack, then clear this user's
# stale Fast DDS shared-memory locks. Run before a clean simulation restart.

source /opt/ros/humble/setup.bash
set -euo pipefail

short_names=(gzserver gzclient rviz2)
command_paths=(
  '/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'
  '/opt/ros/humble/lib/nav2_map_server/map_server'
  '/opt/ros/humble/lib/nav2_amcl/amcl'
  '/opt/ros/humble/lib/nav2_controller/controller_server'
  '/opt/ros/humble/lib/nav2_planner/planner_server'
  '/opt/ros/humble/lib/nav2_behaviors/behavior_server'
  '/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator'
  '/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager'
  '/opt/ros/humble/lib/nav2_velocity_smoother/velocity_smoother'
  '/opt/ros/humble/lib/nav2_waypoint_follower/waypoint_follower'
  '/opt/ros/humble/lib/nav2_smoother/smoother_server'
  '/home/ros/dev_ws/install/moon_warehouse_bringup/lib/moon_warehouse_bringup/safe_goal_bridge.py'
  '/home/ros/dev_ws/install/tools_demo/lib/tools_demo/vision_detect'
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

if managed_processes_remain; then
  echo 'Some managed processes remain; not removing Fast DDS locks.' >&2
  echo 'Inspect with: ps -ef | grep -E "gzserver|gzclient|nav2_|rviz2|vision_detect|safe_goal_bridge"' >&2
  exit 1
fi

find /dev/shm -maxdepth 1 -type f -user "$USER" -name 'fastrtps_port*' -delete
echo 'PASS: managed processes stopped and stale Fast DDS locks cleared.'
