cmake_minimum_required(VERSION 3.5)
project(quadruped_utils)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

add_library(quadruped_utils_controllers SHARED
   
    src/ZeroJointsController.cpp

)

target_include_directories(quadruped_utils_controllers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(quadruped_utils_controllers
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  controller_manager_msgs
)

install(
  TARGETS quadruped_utils_controllers
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(controller_interface ZeroJointsController.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()