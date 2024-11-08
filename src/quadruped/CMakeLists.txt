cmake_minimum_required(VERSION 3.8)
project(quadruped)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "messages/LegCommand.msg"
)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ros_gz_sim_INCLUDE_DIRS}
  ${gz-sim8_INCLUDE_DIRS}
)

# Add all .cpp files in src directory to the build
file(GLOB_RECURSE SOURCES src/*.cpp)

# Create library
add_library(${PROJECT_NAME}_lib ${SOURCES})

# Add include directories to the target
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  hardware_interface
  controller_manager
  rclcpp_lifecycle
  ros2_control
  pluginlib
  ros_gz_sim
  gz-sim8
  gz-plugin2
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the library
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install scripts
install(
  PROGRAMS
    scripts/LegController.py
    scripts/GaitPatternGenerator.py
    scripts/leg_class.py
    scripts/gait_pattern_class.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install urdf files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install message files
install(
  DIRECTORY messages/
  DESTINATION share/${PROJECT_NAME}/messages
)

# Install any additional files from the install directory
install(
  DIRECTORY install/
  DESTINATION share/${PROJECT_NAME}/install
)

# Export targets
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  hardware_interface
  controller_manager
  rclcpp_lifecycle
  ros2_control
  pluginlib
  ros_gz_sim
  gz-sim8
  gz-plugin2
)

# Enable ament packaging
ament_package()