#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE=/home/ros/dev_ws
LOG_DIR="${WORKSPACE}/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/competition_${STAMP}.log"

mkdir -p "${LOG_DIR}"
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RCUTILS_COLORIZED_OUTPUT=1
export PYTHONUNBUFFERED=1

echo "[launcher] workspace=${WORKSPACE}"
echo "[launcher] display=${DISPLAY:-unset} wayland=${WAYLAND_DISPLAY:-unset}"
echo "[launcher] log=${LOG_FILE}"

cd "${WORKSPACE}"
ros2 launch moon_warehouse_bringup mission_system.launch.py \
  start_rviz:=false \
  gazebo_gui:=true \
  start_perception:=true \
  start_manipulation:=true \
  start_foxglove:=false \
  start_rosbridge:=true \
  2>&1 | tee -a "${LOG_FILE}"
