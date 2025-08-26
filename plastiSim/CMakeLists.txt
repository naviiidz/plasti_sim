cmake_minimum_required(VERSION 3.8)
project(plastiSim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Find Gazebo packages
find_package(gz-cmake3 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Add executable for debris spawner
add_executable(debris_spawner src/debris_spawner.cpp)

# Link dependencies
ament_target_dependencies(debris_spawner
  rclcpp
)

target_link_libraries(debris_spawner
  gz-transport13::core
  gz-msgs10::core
)

# Install model files
install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models/
)

# Install plugin libraries
install(DIRECTORY
  plugins/
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "build" EXCLUDE
  PATTERN "*.cc" EXCLUDE
  PATTERN "CMakeLists.txt" EXCLUDE
)

# Install plugin source code and build files for reference
install(DIRECTORY
  plugins/
  DESTINATION share/${PROJECT_NAME}/src/
  FILES_MATCHING
  PATTERN "*.cc"
  PATTERN "*.h" 
  PATTERN "*.hpp"
  PATTERN "CMakeLists.txt"
)

# Install executables
install(TARGETS
  debris_spawner
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

ament_package()
