cmake_minimum_required(VERSION 3.5)
project(ocean_current_wave_plugin)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(OceanCurrentWavePlugin SHARED OceanCurrentWavePlugin.cc)
target_link_libraries(OceanCurrentWavePlugin ${GAZEBO_LIBRARIES})

# Install the plugin library
install(TARGETS OceanCurrentWavePlugin
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
