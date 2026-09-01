cmake_minimum_required(VERSION 3.8)
project(moon_warehouse_prediction_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_library(moon_warehouse_prediction_layer SHARED
  src/prediction_layer.cpp
)
target_include_directories(moon_warehouse_prediction_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(moon_warehouse_prediction_layer
  rclcpp
  nav2_costmap_2d
  gazebo_msgs
  pluginlib
  nav2_util
  geometry_msgs
  builtin_interfaces
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS moon_warehouse_prediction_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(moon_warehouse_prediction_layer)
ament_export_dependencies(rclcpp)
ament_export_dependencies(nav2_costmap_2d)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(pluginlib)
ament_export_dependencies(nav2_util)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(builtin_interfaces)
ament_package()
