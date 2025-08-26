cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(usv_thrust_plugin)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(thrust_plugin SHARED thrust_plugin.cpp)
target_link_libraries(thrust_plugin ${GAZEBO_LIBRARIES})

# Optional: install target
install(TARGETS thrust_plugin LIBRARY DESTINATION lib)