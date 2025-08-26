cmake_minimum_required(VERSION 3.5)
project(robot_buoyancy_wave)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(robot_buoyancy_wave SHARED robot_buoyancy_wave.cpp)
target_link_libraries(robot_buoyancy_wave ${GAZEBO_LIBRARIES})
