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
navigation_active=false
for attempt in $(seq 1 30); do
  # The generic `ros2 lifecycle get` command depends on graph introspection
  # and can report "Node not found" even while the lifecycle service is
  # available under Fast DDS load. Query the service directly instead.
  state="$(timeout 5 ros2 service call \
    /bt_navigator/get_state lifecycle_msgs/srv/GetState '{}' \
    2>/dev/null || true)"
  if grep -qi "label='active'" <<<"${state}"; then
    echo "[readiness] navigation ACTIVE after $((35 + attempt * 5)) s"
    navigation_active=true
    break
  fi
  sleep 5
done

if [[ "${navigation_active}" != true ]]; then
  echo '[readiness] navigation did not become ACTIVE; requesting clean service restart' >&2
  exit 1
fi

# A lifecycle-active BT is insufficient when the standalone map server never
# completed its own transition.  In that state every goal is accepted but the
# robot stays at the origin while both costmaps repeat "no map received".
# Require one transient-local map sample before systemd reports startup done.
echo '[readiness] waiting for /map OccupancyGrid'
for attempt in $(seq 1 12); do
  if timeout 8 ros2 topic echo /map --once >/dev/null 2>&1; then
    echo "[readiness] map received on attempt ${attempt}"
    exit 0
  fi
  sleep 2
done

echo '[readiness] /map was not published; requesting clean service restart' >&2
exit 1
