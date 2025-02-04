cmake_minimum_required(VERSION 3.5)
project(quadruped_hardware)

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

  src/hex_utils.cpp
  src/CANtester.cpp
  src/ROS2_CAN_Interface.cpp
  src/CANInterface.cpp
  src/MotorDriver.cpp
)


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


# Install the plugin description file
install(FILES ros2_Cantester.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface ros2_Cantester.xml)


# # Install the shared library
# install(TARGETS CAN_Tester
#   EXPORT export_CAN_Tester
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


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