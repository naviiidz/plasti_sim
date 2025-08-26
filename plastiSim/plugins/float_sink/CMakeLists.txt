cmake_minimum_required(VERSION 3.5)
project(float_sink_plugin)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(FloatSinkPlugin SHARED FloatSinkPlugin.cc)
target_link_libraries(FloatSinkPlugin ${GAZEBO_LIBRARIES})

# Install the plugin library
install(TARGETS FloatSinkPlugin
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
