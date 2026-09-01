#!/usr/bin/env bash

# One-command repeatable navigation acceptance run: record first, then patrol.
source /opt/ros/humble/setup.bash
source "${HOME}/dev_ws/install/setup.bash"
set -euo pipefail

cycles="${1:-5}"
goal_timeout="${2:-90.0}"
output_root="${3:-${HOME}/navigation_rework_tests}"
traversals="${4:-}"
timestamp="$(date +%Y%m%d_%H%M%S)"
bag_dir="${output_root}/run_${timestamp}"
mkdir -p "${output_root}"

topics=(
  /clock /odom /plan /plan_smoothed /scan /scan_costmap /tf /tf_static /amcl_pose
  /cmd_vel_nav_raw /cmd_vel_nav /cmd_vel_sipp /cmd_vel
  /moon_warehouse/dynamic_obstacle_trajectories
  /moving_obstacle_1/current_pose /moving_obstacle_2/current_pose
  /sipp/state /sipp/speed_mode /sipp/decision /sipp/hold
  /mppi/sipp_feedback
  /sipp/remaining_path /sipp/nearest_path_index
  /sipp/stop_line /sipp/conflict_point
  /departure_heading/state /departure_heading/target /departure_heading/error
  /collision_monitor_state /rosout
)

echo "Checking the live navigation chain..."
# A valid bag must come from exactly one complete navigation launch.  A stale
# standalone simulation can otherwise keep publishing plausible odometry after
# the new Gazebo instance has failed to bind its port.
navigation_launch_count="$(pgrep -fc '[n]avigation.launch.py' || true)"
simulation_launch_count="$(pgrep -fc '[s]imulation.launch.py' || true)"
gzserver_count="$(pgrep -xc gzserver || true)"
if [[ "${navigation_launch_count}" -ne 1 ||
      "${simulation_launch_count}" -ne 0 ||
      "${gzserver_count}" -ne 1 ]]; then
  echo "FAIL: runtime is not a single clean stack:" >&2
  echo "  navigation.launch.py=${navigation_launch_count} (expected 1)" >&2
  echo "  simulation.launch.py=${simulation_launch_count} (expected 0)" >&2
  echo "  gzserver=${gzserver_count} (expected 1)" >&2
  echo "Run ros_runtime_cleanup.sh, then start navigation.launch.py once." >&2
  exit 2
fi
# The ROS 2 CLI daemon can be left in a bad rclpy state after repeated
# simulator restarts. Restarting only the discovery daemon does not affect
# the running ROS graph and keeps the acceptance check deterministic.
ros2 daemon stop >/dev/null 2>&1 || true
python3 "${HOME}/dev_ws/src/moon_warehouse_dynamic_avoidance/tools/navigation_runtime_health_check.py"
timeout 8 ros2 topic echo /scan_costmap --once >/dev/null

echo "BAG_DIR=${bag_dir}"
ros2 bag record --output "${bag_dir}" "${topics[@]}" &
bag_pid=$!
cleanup() {
  kill -INT "${bag_pid}" 2>/dev/null || true
  wait "${bag_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
sleep 2

patrol_args=(
  --cycles "${cycles}" \
  --pause-seconds 1.0 \
  --goal-timeout "${goal_timeout}" \
  --continue-on-failure
)
if [[ -n "${traversals}" ]]; then
  patrol_args+=(--traversals "${traversals}")
fi

set +e
start_seconds="$(date +%s)"
python3 "${HOME}/dev_ws/src/moon_warehouse_dynamic_avoidance/tools/dynamic_avoidance_patrol_test.py" \
  "${patrol_args[@]}"
patrol_rc=$?
elapsed_seconds="$(($(date +%s) - start_seconds))"
set -e

cleanup
trap - EXIT INT TERM
echo "PATROL_RC=${patrol_rc}"
echo "ELAPSED_SECONDS=${elapsed_seconds}"
echo "BAG_DIR=${bag_dir}"
if [[ "${patrol_rc}" -eq 0 ]]; then
  if [[ -n "${traversals}" ]]; then
    echo "ACCEPTANCE_RESULT=PASS (${traversals} traversals)"
  else
    echo "ACCEPTANCE_RESULT=PASS (${cycles} cycles, $((cycles * 2)) goals)"
  fi
else
  echo "ACCEPTANCE_RESULT=FAIL; inspect the bag above before changing parameters"
fi
exit "${patrol_rc}"
