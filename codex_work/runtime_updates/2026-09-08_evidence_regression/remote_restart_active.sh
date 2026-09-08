#!/usr/bin/env bash
set -e
/home/ros/dev_ws/src/moon_warehouse_bringup/scripts/ros_runtime_cleanup.sh
source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ros/dev_ws/config/fastdds_udp_only.xml
export DISPLAY=:0
nohup ros2 launch moon_warehouse_bringup mission_system.launch.py start_foxglove:=false > /home/ros/dev_ws/logs/active_restart_stack_20260908.log 2>&1 </dev/null &
sleep 25
log=/home/ros/dev_ws/logs/active_restart_stack_20260908.log
managed_count=$(grep -c 'Managed nodes are active' "$log" || true)
arm_ok=$(grep -c 'Configured and activated.*arm_controller' "$log" || true)
gripper_ok=$(grep -c 'Configured and activated.*gripper_controller' "$log" || true)
if [ "$gripper_ok" -eq 0 ] \
    && grep -q "Loading controller 'gripper_controller'" "$log"; then
  # A delayed load reply can leave a successfully loaded controller in the
  # unconfigured state while the spawner exits on its retry. Recover exactly
  # that state once, after startup traffic has settled.
  timeout -k 1s 20s ros2 control set_controller_state gripper_controller inactive >/dev/null 2>&1 || true
  timeout -k 1s 20s ros2 control set_controller_state gripper_controller active >/dev/null 2>&1 || true
  controllers=$(timeout -k 1s 8s ros2 control list_controllers 2>/dev/null || true)
  if printf '%s' "$controllers" | grep -q 'gripper_controller.*active'; then
    gripper_ok=1
  fi
fi
# Controller spawner messages are not a reliable readiness source when their
# replies are delayed by Gazebo startup.  Always accept the controller
# manager's live state before declaring the stack unready.
controllers=$(timeout -k 1s 8s ros2 control list_controllers 2>/dev/null || true)
if printf '%s' "$controllers" | grep -q 'arm_controller.*active'; then
  arm_ok=1
fi
if printf '%s' "$controllers" | grep -q 'gripper_controller.*active'; then
  gripper_ok=1
fi
for _readiness_attempt in {1..12}; do
  controllers=$(timeout -k 1s 8s ros2 control list_controllers 2>/dev/null || true)
  arm_ok=0
  gripper_ok=0
  printf '%s' "$controllers" | grep -q 'arm_controller.*active' && arm_ok=1
  printf '%s' "$controllers" | grep -q 'gripper_controller.*active' && gripper_ok=1
  amcl_state=$(timeout -k 1s 4s ros2 lifecycle get /amcl 2>/dev/null || true)
  planner_state=$(timeout -k 1s 4s ros2 lifecycle get /planner_server 2>/dev/null || true)
  navigator_state=$(timeout -k 1s 4s ros2 lifecycle get /bt_navigator 2>/dev/null || true)
  amcl_pose_ok=0
  timeout -k 1s 4s ros2 topic echo --once /amcl_pose >/dev/null 2>&1 \
    && amcl_pose_ok=1
  manipulation_ok=0
  timeout -k 1s 6s ros2 action info /manipulation/execute -t 2>/dev/null \
    | grep -q 'Action servers: 1' && manipulation_ok=1
  printf 'MANAGED_ACTIVE_COUNT=%s ARM_ACTIVE=%s GRIPPER_ACTIVE=%s AMCL=%s PLANNER=%s NAVIGATOR=%s AMCL_POSE=%s MANIPULATION=%s\n' \
    "$managed_count" "$arm_ok" "$gripper_ok" "${amcl_state:-missing}" \
    "${planner_state:-missing}" "${navigator_state:-missing}" "$amcl_pose_ok" \
    "$manipulation_ok"
  if [ "$arm_ok" -eq 1 ] && [ "$gripper_ok" -eq 1 ] \
      && printf '%s' "$amcl_state" | grep -q 'active' \
      && printf '%s' "$planner_state" | grep -q 'active' \
      && printf '%s' "$navigator_state" | grep -q 'active' \
      && [ "$amcl_pose_ok" -eq 1 ] && [ "$manipulation_ok" -eq 1 ]; then
    exit 0
  fi
  sleep 2
done
echo 'STACK_NOT_READY'
exit 1
