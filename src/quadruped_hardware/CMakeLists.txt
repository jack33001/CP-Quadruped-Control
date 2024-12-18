cmake_minimum_required(VERSION 3.8)
project(quadruped_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rclcpp_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
  ${controller_interface_INCLUDE_DIRS}
)

# Create the hardware interface library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/CANInterface.cpp
  src/MotorDriver.cpp
  
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
)

# Add ROS2 CAN Interface executable
add_executable(ros2_can_interface 
  src/ROS2_CAN_Interface.cpp
)

target_include_directories(ros2_can_interface
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ros2_can_interface
  ${PROJECT_NAME}
)

ament_target_dependencies(ros2_can_interface
  rclcpp
  sensor_msgs
  std_msgs
)

# Install targets - all in one command
install(
  TARGETS 
    ${PROJECT_NAME}
    ros2_can_interface
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install directories
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()