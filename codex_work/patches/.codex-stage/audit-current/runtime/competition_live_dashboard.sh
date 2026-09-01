#!/usr/bin/env bash
set -Eeo pipefail

source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
export ROS2CLI_NO_DAEMON=1

while true; do
  clear
  echo "========== 赛题系统实时状态 $(date '+%F %T') =========="
  systemctl --user show moon-warehouse.service \
    --property=ActiveState,SubState,NRestarts --no-pager
  echo
  echo "关键接口:"
  for endpoint in /mission/run_flow /gazebo/get_entity_state /bt_navigator/get_state; do
    if timeout 2 ros2 service type "${endpoint}" >/dev/null 2>&1; then
      printf '  [OK] %s\n' "${endpoint}"
    else
      printf '  [--] %s\n' "${endpoint}"
    fi
  done
  if ss -ltn 2>/dev/null | grep -q ':9090 '; then
    echo '  [OK] Rosbridge :9090（Windows CoStudio）'
  else
    echo '  [--] Rosbridge :9090'
  fi
  echo
  echo '任务状态:'
  timeout 3 ros2 topic echo /mission/execution_status --once \
    2>/dev/null | sed -n '1,18p' || echo '  等待任务状态话题……'
  echo
  echo '按 Ctrl+C 仅关闭本状态面板，不会停止仿真。'
  sleep 2
done
