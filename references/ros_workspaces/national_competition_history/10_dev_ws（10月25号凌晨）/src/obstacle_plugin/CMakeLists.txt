cmake_minimum_required(VERSION 3.8)
project(obstacle_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找Gazebo开发包
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ament_cmake REQUIRED)

# 包含目录
include_directories(
  include
)

# 创建插件库
add_library(moving_obstacle_plugin SHARED
  src/moving_obstacle_plugin.cc
)

# 链接Gazebo库
target_link_libraries(moving_obstacle_plugin
  ${GAZEBO_LIBRARIES}
)

# 设置插件输出路径
set_target_properties(moving_obstacle_plugin PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib
)

# 安装插件
install(TARGETS
  moving_obstacle_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

