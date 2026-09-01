#!/usr/bin/env bash
# Simulation-only recovery tool: seed AMCL from Gazebo's robot ground truth.
# It never sends velocity commands and does not move the robot.

set -euo pipefail

entity_name="${1:-six_arm}"
if [ "$#" -gt 1 ]; then
  echo "Usage: $0 [gazebo_entity_name]" >&2
  exit 2
fi

if ! command -v gz >/dev/null 2>&1; then
  echo 'FAIL: Gazebo Classic command `gz` is unavailable.' >&2
  exit 1
fi

if ! timeout 5s ros2 node list 2>/dev/null | grep -qx '/amcl'; then
  echo 'FAIL: /amcl is unavailable; start navigation.launch.py first.' >&2
  exit 1
fi

pose="$(timeout 5s gz model -m "$entity_name" -p 2>/dev/null || true)"
read -r x y z roll pitch yaw extra <<< "$pose"
number='^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$'
if [[ -n "${extra:-}" || ! "$x" =~ $number || ! "$y" =~ $number || ! "$yaw" =~ $number ]]; then
  echo "FAIL: could not read a six-value pose for Gazebo entity '$entity_name'." >&2
  echo "Received: ${pose:-<no response>}" >&2
  exit 1
fi

echo "Gazebo truth pose for $entity_name: x=$x, y=$y, yaw=$yaw rad"
echo 'Publishing a bounded AMCL initial-pose burst; the robot will not move.'
exec ros2 run moon_warehouse_bringup initial_pose_publisher.py --ros-args \
  -p x:="$x" -p y:="$y" -p yaw:="$yaw" -p publish_count:=8
