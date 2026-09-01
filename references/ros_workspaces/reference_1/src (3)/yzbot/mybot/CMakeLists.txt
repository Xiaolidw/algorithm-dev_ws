cmake_minimum_required(VERSION 3.22)
project(mybot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==========================================================
# 🔹 原有逻辑：安装 Python、launch、config
# ==========================================================
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  auto_grasp_moveit.py
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# ==========================================================
# 🔹 新增逻辑：自定义消息 + Gazebo 插件
# ==========================================================

# 1️⃣ 查找依赖包
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ⭐ Gazebo 头文件与库路径
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# 2️⃣ 生成自定义消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectPose.msg"
  "msg/ObjectPoseArray.msg"
  "msg/GraspCommand.msg"
  "msg/GraspFeedback.msg"
  "msg/ObstacleTrajectory.msg"
  "msg/ObstacleTrajectoryArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ==========================================================
# 🔸 插件：ObjectPose Gazebo 插件（保留）
# ==========================================================
add_library(gazebo_object_pose_plugin SHARED
  gazebo_object_pose_plugin.cpp
)

ament_target_dependencies(gazebo_object_pose_plugin
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_dev
  gazebo_ros
  tf2
  tf2_ros
)

rosidl_get_typesupport_target(
  typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(gazebo_object_pose_plugin
  ${typesupport_target}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS
  gazebo_object_pose_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================================
# 🔹 导出依赖并结束
# ==========================================================
ament_export_dependencies(rosidl_default_runtime tf2 tf2_ros)
ament_package()

