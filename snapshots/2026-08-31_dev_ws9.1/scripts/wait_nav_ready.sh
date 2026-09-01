#!/usr/bin/env bash
set -Eeo pipefail

source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
export ROS2CLI_NO_DAEMON=1

echo '[readiness] waiting for bt_navigator ACTIVE'
# The lifecycle manager intentionally starts at 20 s.  Do not flood an
# unavailable lifecycle service during Gazebo/robot/controller startup; stale
# timed-out requests can make the first real get_state response miss its
# client under Fast DDS load.
sleep 35
for attempt in $(seq 1 30); do
  # The generic `ros2 lifecycle get` command depends on graph introspection
  # and can report "Node not found" even while the lifecycle service is
  # available under Fast DDS load. Query the service directly instead.
  state="$(timeout 5 ros2 service call \
    /bt_navigator/get_state lifecycle_msgs/srv/GetState '{}' \
    2>/dev/null || true)"
  if grep -qi "label='active'" <<<"${state}"; then
    echo "[readiness] navigation ACTIVE after $((35 + attempt * 5)) s"
    exit 0
  fi
  sleep 5
done

echo '[readiness] navigation did not become ACTIVE; requesting clean service restart' >&2
exit 1
