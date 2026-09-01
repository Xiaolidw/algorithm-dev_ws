#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash

set -euo pipefail

ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
  "{state: {name: moving_obstacle_1, pose: {position: {x: -2.97, y: 2.8, z: 0.505}, orientation: {w: 1.0}}, twist: {linear: {x: 0.18}}, reference_frame: world}}"

ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
  "{state: {name: six_arm, pose: {position: {x: -2.7147, y: 1.8811, z: 0.0}, orientation: {w: 1.0}}, twist: {}, reference_frame: world}}"

echo "controlled yield test positioned"
