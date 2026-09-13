#!/usr/bin/env bash
set -Eeo pipefail

source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
export ROS2CLI_NO_DAEMON=1
export RCUTILS_COLORIZED_OUTPUT=0

sleep 35

# Controller spawners can time out after Gazebo has loaded a controller but
# before it is configured.  Repair each required action controller from that
# single observable state once; never reload an existing controller and never
# run a background repair loop.
controllers=""
for attempt in $(seq 1 18); do
  controllers="$(timeout 8 ros2 service call \
    /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers '{}' 2>/dev/null || true)"
  if grep -q "name='arm_controller'" <<<"${controllers}" \
      && grep -q "name='gripper_controller'" <<<"${controllers}"; then
    break
  fi
  sleep 2
done

for controller_name in arm_controller gripper_controller; do
  controller_state="$(sed -n "s/.*name='${controller_name}', state='\([^']*\)'.*/\1/p" <<<"${controllers}")"
  case "${controller_state}" in
    active)
      echo "[readiness] ${controller_name} already active"
      ;;
    unconfigured)
      echo "[readiness] configuring ${controller_name} after spawner timeout"
      timeout 12 ros2 control set_controller_state "${controller_name}" inactive
      timeout 12 ros2 control set_controller_state "${controller_name}" active
      ;;
    inactive)
      echo "[readiness] activating ${controller_name} after spawner timeout"
      timeout 12 ros2 control set_controller_state "${controller_name}" active
      ;;
    *)
      echo "[readiness] ${controller_name} unavailable (state=${controller_state:-missing})" >&2
      exit 1
      ;;
  esac
done

controllers="$(timeout 8 ros2 service call \
  /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}' 2>/dev/null || true)"
for controller_name in arm_controller gripper_controller; do
  if ! grep -q "name='${controller_name}', state='active'" <<<"${controllers}"; then
    echo "[readiness] ${controller_name} did not become active" >&2
    exit 1
  fi
done

navigation_active=false
for attempt in $(seq 1 30); do
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
  echo '[readiness] navigation did not become ACTIVE' >&2
  exit 1
fi

# The map topic is transient-local and the ROS 2 CLI intermittently fails to
# match a late subscriber after Fast DDS cleanup.  The map server lifecycle is
# the authoritative readiness signal: it can only become active after loading
# the configured map, and the already-active planner confirms its consumer.
for attempt in $(seq 1 8); do
  map_state="$(timeout 5 ros2 service call \
    /map_server/get_state lifecycle_msgs/srv/GetState '{}' \
    2>/dev/null || true)"
  if grep -qi "label='active'" <<<"${map_state}"; then
    echo "[readiness] map_server ACTIVE (probe ${attempt}/8)"
    exit 0
  fi
  # Fast DDS discovery can transiently return an empty one-shot CLI response
  # immediately after bt_navigator becomes active.  Bound the retry instead
  # of tearing down an otherwise healthy Gazebo/Nav2 stack.
  sleep 2
done

echo '[readiness] map_server did not become ACTIVE' >&2
exit 1
