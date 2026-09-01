#!/usr/bin/env bash
# Read-only health check for the navigation + YOLO acceptance session.

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [ -f "$HOME/dev_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/dev_ws/install/setup.bash"
fi

set -u

check_lifecycle() {
  local node="$1"
  local state
  state="$(timeout 8s ros2 lifecycle get "$node" 2>&1 || true)"
  if [[ "$state" == *"active [3]"* ]]; then
    printf 'PASS lifecycle %-22s %s\n' "$node" "$state"
  else
    printf 'FAIL lifecycle %-22s %s\n' "$node" "$state"
    return 1
  fi
}

check_topic() {
  local topic="$1"
  if timeout 8s ros2 topic list 2>/dev/null | grep -qx "$topic"; then
    printf 'PASS topic     %s\n' "$topic"
  else
    printf 'FAIL topic     %s is missing\n' "$topic"
    return 1
  fi
}

check_node() {
  local node="$1"
  if timeout 8s ros2 node list 2>/dev/null | grep -qx "$node"; then
    printf 'PASS node      %s\n' "$node"
  else
    printf 'FAIL node      %s is missing\n' "$node"
    return 1
  fi
}

failed=0
printf '%s\n' '=== Navigation + vision health check ==='
for node in /map_server /amcl /planner_server /controller_server /bt_navigator; do
  check_lifecycle "$node" || failed=1
done

for topic in /map /scan /odom /tf /tf_static /camera/image_raw /camera/camera_info; do
  check_topic "$topic" || failed=1
done

check_node /color_detector || failed=1
check_topic /perception/target_pose || failed=1
check_node /safe_goal_bridge || failed=1
check_topic /moon_warehouse/goal_guard/status || failed=1

printf '%s\n' '--- map to base_link TF ---'
if timeout 4s ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -q 'Translation:'; then
  printf '%s\n' 'PASS TF        map -> base_link'
else
  printf '%s\n' 'FAIL TF        map -> base_link unavailable'
  failed=1
fi

printf '%s\n' '--- camera rate (sample for 5 seconds) ---'
timeout 5s ros2 topic hz /camera/image_raw 2>&1 | tail -n 4 || true

if [ "$failed" -eq 0 ]; then
  printf '%s\n' 'RESULT: PASS - navigation and visual interfaces are ready for manual target/recognition acceptance.'
  exit 0
fi

printf '%s\n' 'RESULT: FAIL - inspect the first failed line before sending navigation goals.'
exit 1
