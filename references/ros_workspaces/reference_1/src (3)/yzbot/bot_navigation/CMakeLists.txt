cmake_minimum_required(VERSION 3.5)
project(bot_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


# 安装配置、地图等资源
install(
  DIRECTORY launch param maps config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# === 安装 Python 节点脚本，供 ros2 run 使用 ===
install(
  PROGRAMS
    bot_navigation/mission_controller.py
    bot_navigation/system_status_node.py
    scripts/goto_goal.py
    scripts/mock_publishers.py
    scripts/data_logger_node.py
    scripts/lstm_predictor_node.py
    scripts/obstacle_trajectory_viz.py
    bot_navigation/behavior_manager_node.py

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

