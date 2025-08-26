cmake_minimum_required(VERSION 3.0)
project(SurfaceMotionPlugin)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(SurfaceMotionPlugin SHARED SurfaceMotionPlugin.cc)
target_link_libraries(SurfaceMotionPlugin ${GAZEBO_LIBRARIES})