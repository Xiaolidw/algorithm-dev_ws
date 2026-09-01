#!/usr/bin/env bash
set -Eeo pipefail

SERVICE=moon-warehouse.service
WORKSPACE=/home/ros/dev_ws
RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
VIEWER_PID_FILE="${RUNTIME_DIR}/moon-warehouse-log-viewer.pid"

open_logs() {
  if [[ -s "${VIEWER_PID_FILE}" ]]; then
    viewer_pid="$(cat "${VIEWER_PID_FILE}")"
    if kill -0 "${viewer_pid}" 2>/dev/null; then
      return 0
    fi
  fi

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title='赛题系统实时日志' -- bash -lc \
      "echo \$\$ > '${VIEWER_PID_FILE}'; trap 'rm -f \"${VIEWER_PID_FILE}\"' EXIT; journalctl --user -fu '${SERVICE}' --output=cat; exec bash" \
      >/dev/null 2>&1 &
  fi
}

wait_for_service() {
  # The unit stays "activating" until ExecStartPost confirms that Nav2 is
  # ACTIVE.  This prevents a visible Gazebo process from being mistaken for
  # an execution-ready competition system on a heavily loaded VM login.
  for _ in $(seq 1 240); do
    if systemctl --user is-active --quiet "${SERVICE}"; then
      return 0
    fi
    sleep 1
  done
  echo "赛题服务未能在 240 秒内进入导航就绪状态。" >&2
  return 1
}

run_task() {
  source /opt/ros/humble/setup.bash
  source "${WORKSPACE}/install/setup.bash"
  export ROS2CLI_NO_DAEMON=1
  for _ in $(seq 1 120); do
    if timeout 3 ros2 service type /mission/run_flow \
        2>/dev/null | grep -q 'std_srvs/srv/Trigger'; then
      exec ros2 service call /mission/run_flow std_srvs/srv/Trigger '{}'
    fi
    sleep 1
  done
  echo "等待 /mission/run_flow 超时，请查看实时日志。" >&2
  return 1
}

case "${1:-start}" in
  start)
    systemctl --user start "${SERVICE}"
    wait_for_service
    open_logs
    ;;
  stop)
    systemctl --user stop "${SERVICE}"
    ;;
  restart)
    systemctl --user restart "${SERVICE}"
    wait_for_service
    open_logs
    ;;
  status)
    systemctl --user status "${SERVICE}" --no-pager
    ;;
  logs)
    open_logs
    ;;
  task)
    systemctl --user start "${SERVICE}"
    wait_for_service
    open_logs
    run_task
    ;;
  *)
    echo "用法: $0 {start|stop|restart|status|logs|task}" >&2
    exit 2
    ;;
esac
