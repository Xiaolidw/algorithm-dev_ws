cmake_minimum_required(VERSION 3.8)
project(gazebo_physics_obstacle_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Build simple move plugin
add_library(simple_move_plugin SHARED 
  src/SimpleMovePlugin.cc
)

target_include_directories(simple_move_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(simple_move_plugin
  gazebo_dev
  gazebo_msgs
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(simple_move_plugin
  ${GAZEBO_LIBRARIES}
)

# Build cube carry plugin: 物理循环内刚性载运方块(替代服务调用跟随)
add_library(cube_carry_plugin SHARED
  src/CubeCarryPlugin.cc
)

target_include_directories(cube_carry_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cube_carry_plugin
  gazebo_dev
  gazebo_msgs
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(cube_carry_plugin
  ${GAZEBO_LIBRARIES}
)

# Install plugin
install(TARGETS
  simple_move_plugin
  cube_carry_plugin
  LIBRARY DESTINATION lib
)

ament_package()
