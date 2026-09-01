cmake_minimum_required(VERSION 3.8)
project(mppi_lstm_critic LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moon_warehouse_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/lstm_dynamic_obstacle_critic.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle nav2_costmap_2d nav2_mppi_controller
  pluginlib moon_warehouse_interfaces std_msgs)

pluginlib_export_plugin_description_file(
  nav2_mppi_controller lstm_dynamic_obstacle_critic_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES lstm_dynamic_obstacle_critic_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
