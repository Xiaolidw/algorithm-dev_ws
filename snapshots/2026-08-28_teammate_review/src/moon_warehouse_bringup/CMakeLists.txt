cmake_minimum_required(VERSION 3.8)
project(moon_warehouse_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/initial_pose_publisher.py
    scripts/send_navigation_goal.py
    scripts/reset_amcl_from_gazebo_truth.sh
    scripts/ros_runtime_cleanup.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
