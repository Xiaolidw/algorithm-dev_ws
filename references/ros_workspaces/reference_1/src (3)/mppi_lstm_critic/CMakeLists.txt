cmake_minimum_required(VERSION 3.8)
project(mppi_lstm_critic LANGUAGES CXX)

# ===========================
# C++17
# ===========================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===========================
# 依赖
# ===========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_mppi_controller REQUIRED)  # ★ 确保依赖 nav2_mppi_controller
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mybot REQUIRED)

# ===========================
# 插件库
# ===========================
add_library(${PROJECT_NAME} SHARED
  src/lstm_dynamic_obstacle_critic.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  nav2_mppi_controller
  nav2_util
  pluginlib
  mybot
)

# ===========================
# 暂时关闭 ASan（你现在主要用 Valgrind 查问题）
# ===========================
# if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#   message(STATUS "Building ${PROJECT_NAME} with AddressSanitizer (Debug mode)")
#   target_compile_options(${PROJECT_NAME} PRIVATE
#     -fsanitize=address
#     -fno-omit-frame-pointer
#     -g
#     -O1
#   )
#   target_link_options(${PROJECT_NAME} PRIVATE
#     -fsanitize=address
#   )
# endif()

# ===========================
# ★ 关键：把插件描述导给 nav2_mppi_controller
# ===========================
pluginlib_export_plugin_description_file(
  nav2_mppi_controller             # ← 不是 mppi_lstm_critic !!!
  lstm_dynamic_obstacle_critic_plugin.xml
)

# ===========================
# 安装
# ===========================
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES lstm_dynamic_obstacle_critic_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()

