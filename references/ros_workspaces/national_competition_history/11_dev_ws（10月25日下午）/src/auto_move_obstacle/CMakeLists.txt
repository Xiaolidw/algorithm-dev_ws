cmake_minimum_required(VERSION 3.0)
project(auto_move_obstacle)

find_package(gazebo REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(auto_move_obstacle SHARED auto_move_obstacle.cc)
target_link_libraries(auto_move_obstacle ${GAZEBO_LIBRARIES})

install(TARGETS auto_move_obstacle
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

