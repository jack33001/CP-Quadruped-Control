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

# Add CasADi
set(CASADI_INSTALL_DIR "/home/casadi/casadi-3.6.7/build")
link_directories(${CASADI_INSTALL_DIR}/lib)
include_directories(${CASADI_INSTALL_DIR}/include)

# Add ACADOS
set(ACADOS_INSTALL_DIR "/home/acados/acados")
link_directories(${ACADOS_INSTALL_DIR}/lib)
include_directories(${ACADOS_INSTALL_DIR}/include)

# Create library with name matching plugin_description.xml
add_library(quadruped_mpc_controllers SHARED
  # Controller Interfaces
  src/controller_interfaces/balance_controller.cpp
  src/controller_interfaces/state_estimator.cpp
  src/controller_interfaces/quadruped_broadcaster.cpp
  # Utilities
  src/utilities/shared_quadruped_info.cpp
  # Control Laws
  #src/control_laws/balance_controller.cpp
)

# Single unified dependency and configuration section
ament_target_dependencies(quadruped_mpc_controllers
  rclcpp
  controller_interface
  pluginlib
  rclcpp_lifecycle
  pinocchio
)

target_include_directories(quadruped_mpc_controllers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${CASADI_INCLUDE_DIR}
    ${acados_INCLUDE_DIRS}
)

# Consolidate all linking into a single target_link_libraries call
target_link_libraries(quadruped_mpc_controllers
    ${CASADI_INSTALL_DIR}/lib/libcasadi.so  # Use full path and filename
    acados
    blasfeo
    hpipm
)

# Remove target_link_directories as it's not needed when using plain linking

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the library
install(TARGETS quadruped_mpc_controllers
  EXPORT ${PROJECT_NAME}_targets  # Changed from export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install exports
install(
  EXPORT ${PROJECT_NAME}_targets
  FILE ${PROJECT_NAME}_targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Export targets
ament_export_targets(
  ${PROJECT_NAME}_targets HAS_LIBRARY_TARGET
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