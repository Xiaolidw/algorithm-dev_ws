cmake_minimum_required(VERSION 3.8)
project(gazebo_obstacle_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Build plugin
add_library(auto_move_obstacle SHARED src/AutoMoveObstaclePlugin.cc)
target_include_directories(auto_move_obstacle PUBLIC include)
ament_target_dependencies(auto_move_obstacle
  gazebo_dev
  gazebo_msgs
  rclcpp
)

# Install plugin
install(TARGETS
  auto_move_obstacle
  LIBRARY DESTINATION lib
)

ament_package()
