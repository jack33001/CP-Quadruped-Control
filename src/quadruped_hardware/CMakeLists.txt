cmake_minimum_required(VERSION 3.5)
project(quadruped_hardware)

# find_library(SH2_LIBRARY sh2 REQUIRED)
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ROS2_CAN_Motor.cpp
  src/ROS2_CAN_Interface.cpp
  src/CANInterface.cpp
  src/MotorDriver.cpp
  src/ROS2_BNO08x.cpp
  src/i2c_device.cpp
  src/bno08x_driver.cpp

  src/sh2.c
  src/sh2_util.c
  src/shtp.c
  src/euler.c
  include/quadruped_hardware/bno_HAL.hpp

)


include_directories(/usr/include)

# add_executable(i2c_tools src/i2c_tools.cpp)

# Add the include directory for this target
# target_include_directories(i2c_tools PRIVATE /home/ws/include)


target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  
)

pluginlib_export_plugin_description_file(hardware_interface ros2CANInterface_Description.xml)
pluginlib_export_plugin_description_file(hardware_interface ros2CANMotor_Description.xml)
pluginlib_export_plugin_description_file(hardware_interface ros2BNO08X_Description.xml)





install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)



install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
# #
# install(TARGETS ros2_control_node 
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rclcpp hardware_interface pluginlib sensor_msgs std_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()