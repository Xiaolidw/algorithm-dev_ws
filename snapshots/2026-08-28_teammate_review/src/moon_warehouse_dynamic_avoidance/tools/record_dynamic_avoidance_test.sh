#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source "${HOME}/dev_ws/install/setup.bash"

set -euo pipefail

output_root="${1:-${HOME}/dynamic_avoidance_test_bags}"
timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${output_root}/dynamic_avoidance_${timestamp}"

mkdir -p "${output_root}"

topics=(
  /sipp/remaining_path
  /sipp/nearest_path_index
  /clock
  /cmd_vel
  /cmd_vel_smoothed
  /cmd_vel_nav_raw
  /cmd_vel_nav
  /cmd_vel_sipp
  /odom
  /plan
  /plan_smoothed
  /scan
  /scan_costmap
  /tf
  /tf_static
  /amcl_pose
  /behavior_tree_log
  /rosout
  /moon_warehouse/dynamic_obstacle_trajectories
  /moving_obstacle_1/current_pose
  /moving_obstacle_2/current_pose
  /sipp/state
  /sipp/speed_mode
  /sipp/decision
  /sipp/hold
  /mppi/sipp_feedback
  /sipp/stop_line
  /sipp/conflict_point
  /departure_heading/state
  /departure_heading/target
  /departure_heading/error
  /collision_monitor_state
  /local_costmap/costmap
  /local_costmap/costmap_updates
)

echo "Dynamic-avoidance recording will be saved to:"
echo "  ${output_dir}"
echo
echo "Start the patrol test in another terminal."
echo "Press Ctrl+C here after the test has finished."
echo

exec ros2 bag record \
  --output "${output_dir}" \
  "${topics[@]}"
