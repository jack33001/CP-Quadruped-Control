cmake_minimum_required(VERSION 3.12)
project(jetson_interface)

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
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/JetsonGPIO")


add_library(${PROJECT_NAME} SHARED
  src/jetson_gpio_interface.cpp
 
)

# JSON library dependency____________
include(FetchContent)



target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/jetson_interface>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:src>
    $<INSTALL_INTERFACE:/include/jetson_interface>
    
)

include_directories(/usr/include)
include_directories(/usr/include/jetson_interface)


ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface jetson_gpio_interface.xml)


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