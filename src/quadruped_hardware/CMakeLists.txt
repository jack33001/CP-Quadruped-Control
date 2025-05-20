cmake_minimum_required(VERSION 3.12)
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



include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/quadruped_hardware")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/sh2")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")

add_library(${PROJECT_NAME} SHARED
  src/ROS2_CAN_Motor.cpp
  src/CANInterface.cpp
  src/MotorDriver.cpp
  src/watchdog.cpp
 
)

# JSON library dependency____________
include(FetchContent)

# FetchContent_Declare(
#   nlohmann_json
#   GIT_REPOSITORY https://github.com/nlohmann/json.git
#   GIT_TAG v3.11.2 # Replace with the desired version
  
# )
# # set(BUILD_EXAMPLES OFF CACHE INTERNAL "" )
# FetchContent_MakeAvailable(nlohmann_json)

# target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json)
# end JSON library dependency__________


target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/quadruped_hardware>

    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:src>

    $<INSTALL_INTERFACE:/include/quadruped_hardware>
    
)



include_directories(/usr/include)
include_directories(/usr/include/quadruped_hardware)





ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface ros2CANMotor_Description.xml)


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