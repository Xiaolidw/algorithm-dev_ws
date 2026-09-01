cmake_minimum_required(VERSION 3.8)
project(moon_warehouse_dynamic_avoidance)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/lstm_predictor_node.py
    scripts/sipp_decision_node.py
    scripts/sipp_velocity_gate.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
