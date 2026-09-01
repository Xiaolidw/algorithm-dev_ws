#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState "{name: zone_a, reference_frame: world}"
ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState "{name: zone_b, reference_frame: world}"
ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState "{name: six_arm, reference_frame: world}"
ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState "{name: 'officeroom::Wall_81', reference_frame: world}"
ros2 param get /cube_obstacle_map_node zone_areas
timeout 5 ros2 topic echo /mission/execution_status --once || true
