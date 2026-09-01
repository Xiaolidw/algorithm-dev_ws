#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash

ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState \
  "{name: moving_obstacle_1, reference_frame: world}"
ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState \
  "{name: six_arm, reference_frame: world}"
