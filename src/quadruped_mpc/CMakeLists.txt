cmake_minimum_required(VERSION 3.8)
project(quadruped_mpc)

# Add cmake modules path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

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
include_directories(
    ${ACADOS_INSTALL_DIR}/include
    ${ACADOS_INSTALL_DIR}/interfaces/acados_c
    ${ACADOS_INSTALL_DIR}/external/blasfeo/include
    ${ACADOS_INSTALL_DIR}/external/hpipm/include
)

# Set include directories for all targets
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/acados_generated
    ${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/quadruped_ode_model
)

# Set include directories
include_directories(include)

# Fix linking order - create generated code library first
add_library(balance_controller_generated SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/acados_sim_solver_quadruped_ode.c
  ${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/acados_solver_quadruped_ode.c
  ${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/quadruped_ode_model/quadruped_ode_expl_ode_fun.c
  ${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/quadruped_ode_model/quadruped_ode_expl_vde_adj.c
  ${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/quadruped_ode_model/quadruped_ode_expl_vde_forw.c
)

target_link_libraries(balance_controller_generated
  acados
  hpipm
  blasfeo
)

# Create library with name matching plugin_description.xml
add_library(quadruped_mpc_controllers SHARED
  # Controller Interfaces
  src/controller_interfaces/balance_controller.cpp
  src/controller_interfaces/state_estimator.cpp
  src/controller_interfaces/quadruped_broadcaster.cpp
  # Utilities
  src/utilities/shared_quadruped_info.cpp
  # Remove optimal_pd.cpp since we're using balance_controller instead
)

# Single unified dependency and configuration section
ament_target_dependencies(quadruped_mpc_controllers
  rclcpp
  controller_interface
  pluginlib
  rclcpp_lifecycle
  pinocchio
)

target_include_directories(quadruped_mpc_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIR}
  ${ACADOS_INSTALL_DIR}/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/acados_generated>
)

# Consolidate all linking into a single target_link_libraries call
target_link_libraries(quadruped_mpc_controllers
  balance_controller_generated
  ${CASADI_INSTALL_DIR}/lib/libcasadi.so
  acados
  hpipm
  blasfeo
)

# Remove target_link_directories as it's not needed when using plain linking

# Remove separate balance_controller target since it's now part of quadruped_mpc_controllers

# Single consolidated install and export configuration
install(
  TARGETS 
    quadruped_mpc_controllers
    balance_controller_generated
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install exports
install(
  EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Install supporting files
install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY 
    config/
    launch/
    include/
  DESTINATION share/${PROJECT_NAME}
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