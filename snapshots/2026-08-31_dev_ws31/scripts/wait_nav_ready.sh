#!/usr/bin/env bash
set -Eeo pipefail

source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
export ROS2CLI_NO_DAEMON=1

echo '[readiness] waiting for bt_navigator ACTIVE'
for attempt in $(seq 1 90); do
  # The generic `ros2 lifecycle get` command depends on graph introspection
  # and can report "Node not found" even while the lifecycle service is
  # available under Fast DDS load. Query the service directly instead.
  state="$(timeout 4 ros2 service call \
    /bt_navigator/get_state lifecycle_msgs/srv/GetState '{}' \
    2>/dev/null || true)"
  if grep -qi "label='active'" <<<"${state}"; then
    echo "[readiness] navigation ACTIVE after $((attempt * 2)) s"
    exit 0
  fi
  sleep 2
done

echo '[readiness] navigation did not become ACTIVE; requesting clean service restart' >&2
exit 1
