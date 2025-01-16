cmake_minimum_required(VERSION 3.8)
project(quadruped_mpc)

# Ensure ament is configured correctly first
find_package(ament_cmake REQUIRED CONFIG)

# Basic CMake setup first
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find core packages first
find_package(ament_cmake REQUIRED)

# Add Python dependency
find_package(Python3 REQUIRED)

# Rest of find_package commands
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)  # Add this line
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# Add cmake modules path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Add CasADi
set(CASADI_INSTALL_DIR "/home/casadi/casadi-3.6.7/build")
link_directories(${CASADI_INSTALL_DIR}/lib)
include_directories(${CASADI_INSTALL_DIR}/include)

# Add ACADOS 
set(ACADOS_INSTALL_DIR "/home/acados/acados")
link_directories(${ACADOS_INSTALL_DIR}/lib)
link_directories(${ACADOS_INSTALL_DIR}/external/qpoases/bin)
include_directories(
    ${ACADOS_INSTALL_DIR}
    ${ACADOS_INSTALL_DIR}/include
    ${ACADOS_INSTALL_DIR}/interfaces/acados_c
    ${ACADOS_INSTALL_DIR}/external/blasfeo/include
    ${ACADOS_INSTALL_DIR}/external/hpipm/include
    ${ACADOS_INSTALL_DIR}/external/qpoases/include
)

# Set include directories for all targets
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Set include directories
include_directories(include)

# Set ACADOS generated directory and ensure it exists
set(ACADOS_GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/quadruped_mpc/acados_generated)

# Set ACADOS generated files list (match exactly what's generated)
set(ACADOS_GENERATED_FILES
    ${ACADOS_GENERATED_DIR}/acados_sim_solver_quadruped_ode.c
    ${ACADOS_GENERATED_DIR}/acados_solver_quadruped_ode.c
    ${ACADOS_GENERATED_DIR}/quadruped_ode_model/quadruped_ode_expl_ode_fun.c
    ${ACADOS_GENERATED_DIR}/quadruped_ode_model/quadruped_ode_expl_vde_adj.c
    ${ACADOS_GENERATED_DIR}/quadruped_ode_model/quadruped_ode_expl_vde_forw.c
)

# Generate ACADOS code during configure
message(STATUS "Running ACADOS code generation to ${ACADOS_GENERATED_DIR}")
execute_process(
    COMMAND ${CMAKE_COMMAND} -E echo "Starting code generation in ${CMAKE_CURRENT_SOURCE_DIR}"
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/acados/generate_controller.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}  # Changed to package root
    RESULT_VARIABLE ACADOS_GEN_RESULT
    OUTPUT_VARIABLE ACADOS_GEN_OUTPUT
    ERROR_VARIABLE ACADOS_GEN_ERROR
)

# Add explicit qpOASES static library
set(QPOASES_LIB ${ACADOS_INSTALL_DIR}/external/qpoases/bin/libqpOASES_e.a)

# Create the generated code library with explicit C flags
add_library(acados_generated_code SHARED ${ACADOS_GENERATED_FILES})
set_target_properties(acados_generated_code PROPERTIES LINKER_LANGUAGE C)

target_include_directories(acados_generated_code PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${ACADOS_INSTALL_DIR}/include
    ${ACADOS_INSTALL_DIR}/interfaces/acados_c
    ${ACADOS_INSTALL_DIR}/external/blasfeo/include
    ${ACADOS_INSTALL_DIR}/external/hpipm/include
)

# Add explicit dependencies for acados_generated_code
target_link_libraries(acados_generated_code PUBLIC
    acados
    hpipm
    blasfeo
    ${QPOASES_LIB}  # Use explicit path to static library
)

# Create the main controller library
add_library(quadruped_mpc_controllers SHARED
  src/controller_interfaces/balance_controller.cpp
  src/controller_interfaces/state_estimator.cpp
  src/controller_interfaces/quadruped_broadcaster.cpp
  src/utilities/shared_quadruped_info.cpp
)

# Single unified dependency and configuration section
ament_target_dependencies(quadruped_mpc_controllers PUBLIC
  rclcpp
  controller_interface
  hardware_interface  # Add this line
  pluginlib
  rclcpp_lifecycle
  pinocchio
)

target_include_directories(quadruped_mpc_controllers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${CASADI_INCLUDE_DIR}
    ${ACADOS_INSTALL_DIR}/include  # Main ACADOS include
    ${ACADOS_INSTALL_DIR}/external/blasfeo/include
    ${ACADOS_INSTALL_DIR}/external/hpipm/include
    ${ACADOS_INSTALL_DIR}/external/qpoases//include
)

# Link quadruped_mpc_controllers with exxplicit dependencies
target_link_libraries(quadruped_mpc_controllers PUBLIC
    acados_generated_code
    ${CASADI_INSTALL_DIR}/lib/libcasadi.so
    acados
    hpipm
    blasfeo
    qpOASES_e
    pinocchio::pinocchio
    ${EIGEN3_LIBRARIES}
)

# Remove target_link_directories as it's not needed when using plain linking

# Remove separate balance_controller target since it's now part of quadruped_mpc_controllers

# Move exports before install commands
ament_export_dependencies(
  rclcpp
  controller_interface
  hardware_interface  # Add this line
  rclcpp_lifecycle
  pluginlib
  pinocchio
  Eigen3
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Single consolidated install and export configuration
install(
  TARGETS 
    quadruped_mpc_controllers
    acados_generated_code
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install ACADOS generated files to include directory
install(
  DIRECTORY ${ACADOS_GENERATED_DIR}/
  DESTINATION include/quadruped_mpc/acados_generated
  FILES_MATCHING PATTERN "*.h"
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

# Modify the install directive to ensure config files are installed correctly
install(
  DIRECTORY 
    config
    launch
    include
    scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS  # Add this to preserve permissions
  PATTERN "*.pyc" EXCLUDE  # Exclude Python cache files
  PATTERN "__pycache__" EXCLUDE
)

pluginlib_export_plugin_description_file(controller_interface plugin_description.xml)

# Must be absolute last line
ament_package()
