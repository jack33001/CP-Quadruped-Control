cmake_minimum_required(VERSION 3.8)
project(quadruped_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

# Create library
add_library(${PROJECT_NAME}_lib
            src/balance_controller.cpp
            src/foot_trajectory_generator.cpp
            src/leg_controller.cpp
            src/state_estimator.cpp)

# Add include directories to the target
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  controller_interface
  rclcpp_lifecycle
  pluginlib
  pinocchio
  Eigen3
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

# Install the plugin_description.xml file
install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export targets
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  controller_interface
  rclcpp_lifecycle
  pluginlib
  pinocchio
  Eigen3
)

pluginlib_export_plugin_description_file(controller_interface plugin_description.xml)

# Enable ament packaging
ament_package()