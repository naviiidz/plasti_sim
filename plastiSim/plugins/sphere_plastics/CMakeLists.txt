cmake_minimum_required(VERSION 3.10)
project(MicroplasticPlugin)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(MicroplasticPlugin SHARED MicroplasticPlugin.cc)
target_link_libraries(MicroplasticPlugin ${GAZEBO_LIBRARIES})