#!/usr/bin/env bash
set -Eeo pipefail

SERVICE="moon-warehouse.service"
WORLD="/home/ros/dev_ws/install/mybot_description/share/mybot_description/worlds/competition_v2_scoring.world"
RVIZ_CFG="/home/ros/dev_ws/install/moon_warehouse_bringup/share/moon_warehouse_bringup/rviz/competition_nav_topdown.rviz"
FORCE_RESTART=false
if [[ "${1:-}" == "--restart" ]]; then
  FORCE_RESTART=true
elif [[ $# -gt 0 ]]; then
  echo "usage: $0 [--restart]" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash

refresh_cli_discovery() {
  # ros2 lifecycle/action CLI queries use the background discovery daemon.
  # A daemon surviving the previous simulation can report zero servers even
  # though the new graph is healthy, so refresh only this user CLI cache.
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
  sleep 2
}

count_exact() {
  pgrep -af -- "$1" 2>/dev/null | wc -l
}

stack_is_ready() {
  [[ "$(systemctl --user is-active "${SERVICE}" 2>/dev/null || true)" == "active" ]] || return 1
  [[ "$(count_exact "^.*gzserver --verbose ${WORLD}")" -eq 1 ]] || return 1
  [[ "$(count_exact "^.*gzclient --verbose ${WORLD}")" -eq 1 ]] || return 1
  [[ "$(count_exact "^.*rviz2 -d ${RVIZ_CFG}")" -eq 1 ]] || return 1
  ros2 lifecycle get /map_server 2>/dev/null | grep -q "active" || return 1
  ros2 lifecycle get /controller_server 2>/dev/null | grep -q "active" || return 1
  ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "active" || return 1
  ros2 action info /navigate_to_pose 2>/dev/null | grep -q "Action servers: 1" || return 1
  # Lifecycle ACTIVE alone is insufficient: after a Fast-DDS startup race the
  # cube map publisher existed in discovery but neither costmap had received
  # its transient sample.  Require both static inputs to yield real data before
  # accepting a competition task.
  timeout 4 ros2 topic echo /map --once >/dev/null 2>&1 || return 1
  timeout 4 ros2 topic echo /cube_obstacle_map --once >/dev/null 2>&1 || return 1
}

if pgrep -af -- "navigation_pick_place_test" | grep -v -- "ensure_single_competition_stack" >/dev/null; then
  echo "[preflight] refusing stack cleanup while a pick/place task is active" >&2
  exit 2
fi

refresh_cli_discovery
if ! ${FORCE_RESTART} && stack_is_ready; then
  echo "[preflight] ready: one Gazebo server, one Gazebo GUI, one RViz, Nav2 active"
  exit 0
fi

echo "[preflight] unhealthy or duplicate stack detected; replacing only the competition service cgroup"
timeout 35 systemctl --user stop "${SERVICE}" || true
if [[ "$(systemctl --user is-active "${SERVICE}" 2>/dev/null || true)" != "inactive" ]]; then
  systemctl --user kill --kill-who=all --signal=SIGKILL "${SERVICE}" || true
  # A forced kill can satisfy Restart=always before the explicit stop job has
  # been recorded.  A second stop cancels that pending automatic restart.
  timeout 10 systemctl --user stop "${SERVICE}" || true
fi
systemctl --user reset-failed "${SERVICE}" || true

# The service cgroup is authoritative. These exact-path checks only remove a
# visual process left orphaned by an older non-systemd launch of this same world.
pkill -TERM -f "^.*gzserver --verbose ${WORLD}" 2>/dev/null || true
pkill -TERM -f "^.*gzclient --verbose ${WORLD}" 2>/dev/null || true
pkill -TERM -f "^.*rviz2 -d ${RVIZ_CFG}" 2>/dev/null || true
sleep 2
pkill -KILL -f "^.*gzserver --verbose ${WORLD}" 2>/dev/null || true
pkill -KILL -f "^.*gzclient --verbose ${WORLD}" 2>/dev/null || true
pkill -KILL -f "^.*rviz2 -d ${RVIZ_CFG}" 2>/dev/null || true

timeout 240 systemctl --user start "${SERVICE}"
refresh_cli_discovery
for _ in $(seq 1 90); do
  if stack_is_ready; then
    echo "[preflight] clean start ready: one Gazebo server, one Gazebo GUI, one RViz, Nav2 and both maps ready"
    exit 0
  fi
  sleep 2
done

echo "[preflight] stack did not reach the required unique/active state" >&2
systemctl --user status "${SERVICE}" --no-pager -l >&2 || true
exit 1
