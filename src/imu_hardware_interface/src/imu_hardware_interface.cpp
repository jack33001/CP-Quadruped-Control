#include "imu_hardware_interface/imu_hardware_interface.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace imu_hardware_interface
{

hardware_interface::CallbackReturn IMUHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Get parameters from URDF if available
  if (info_.hardware_parameters.count("imu_topic") > 0) {
    imu_topic_ = info_.hardware_parameters.at("imu_topic");
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("IMUHardwareInterface"), 
    "IMU hardware interface initialized successfully. Using topic: %s", 
    imu_topic_.c_str());
    
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IMUHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create a node for the IMU subscription
  node_ = std::make_shared<rclcpp::Node>("imu_hardware_interface_node");
  
  // Create a subscription to the IMU topic
  imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 10, 
    std::bind(&IMUHardwareInterface::imuCallback, this, std::placeholders::_1));
    
  RCLCPP_INFO(
    rclcpp::get_logger("IMUHardwareInterface"), 
    "IMU hardware interface configured and subscribed to %s", 
    imu_topic_.c_str());
    
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IMUHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Remove the subscription and node
  imu_subscription_.reset();
  node_.reset();
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IMUHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("IMUHardwareInterface"), 
    "IMU hardware interface activated");
    
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IMUHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("IMUHardwareInterface"), 
    "IMU hardware interface deactivated");
    
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> IMUHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Add Orientation interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu", "orientation_x", &orientation_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu", "orientation_y", &orientation_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu", "orientation_z", &orientation_z_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu", "orientation_w", &orientation_w_));
    
  // Add Orientation Covariance interfaces
  for (size_t i = 0; i < 9; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "orientation_covariance_" + std::to_string(i), &orientation_covariance_[i]));
  }
  
  // Add Angular Velocity interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu", "angular_velocity_x", &angular_velocity_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu", "angular_velocity_y", &angular_velocity_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu", "angular_velocity_z", &angular_velocity_z_));
    
  // Add Angular Velocity Covariance interfaces
  for (size_t i = 0; i < 9; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "angular_velocity_covariance_" + std::to_string(i), &angular_velocity_covariance_[i]));
  }
  
  // Add Linear Acceleration interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu", "linear_acceleration_x", &linear_acceleration_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu", "linear_acceleration_y", &linear_acceleration_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu", "linear_acceleration_z", &linear_acceleration_z_));
    
  // Add Linear Acceleration Covariance interfaces
  for (size_t i = 0; i < 9; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu", "linear_acceleration_covariance_" + std::to_string(i), &linear_acceleration_covariance_[i]));
  }
  
  return state_interfaces;
}

hardware_interface::return_type IMUHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Spin the node to process callbacks
  rclcpp::spin_some(node_);
  
  if (!data_received_) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("IMUHardwareInterface"),
      *node_->get_clock(), 
      5000,  // Throttle period in ms
      "No IMU data received yet on topic %s", 
      imu_topic_.c_str());
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IMUHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{


  // print all the data received
  RCLCPP_INFO(
    rclcpp::get_logger("IMUHardwareInterface"), 
    "Orientation: [%f, %f, %f, %f]", 
    orientation_x_, orientation_y_, orientation_z_, orientation_w_);
  RCLCPP_INFO(
    rclcpp::get_logger("IMUHardwareInterface"), 
    "Angular Velocity: [%f, %f, %f]", 
    angular_velocity_x_, angular_velocity_y_, angular_velocity_z_);
  RCLCPP_INFO(
    rclcpp::get_logger("IMUHardwareInterface"), 
    "Linear Acceleration: [%f, %f, %f]", 
    linear_acceleration_x_, linear_acceleration_y_, linear_acceleration_z_);
  // This is a read-only interface, nothing to write
  return hardware_interface::return_type::OK;
}

void IMUHardwareInterface::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Update orientation
  orientation_x_ = msg->orientation.x;
  orientation_y_ = msg->orientation.y;
  orientation_z_ = msg->orientation.z;
  orientation_w_ = msg->orientation.w;
  
  // Update orientation covariance
  for (size_t i = 0; i < 9; ++i) {
    orientation_covariance_[i] = msg->orientation_covariance[i];
  }
  
  // Update angular velocity
  angular_velocity_x_ = msg->angular_velocity.x;
  angular_velocity_y_ = msg->angular_velocity.y;
  angular_velocity_z_ = msg->angular_velocity.z;
  
  // Update angular velocity covariance
  for (size_t i = 0; i < 9; ++i) {
    angular_velocity_covariance_[i] = msg->angular_velocity_covariance[i];
  }
  
  // Update linear acceleration
  linear_acceleration_x_ = msg->linear_acceleration.x;
  linear_acceleration_y_ = msg->linear_acceleration.y;
  linear_acceleration_z_ = msg->linear_acceleration.z;
  
  // Update linear acceleration covariance
  for (size_t i = 0; i < 9; ++i) {
    linear_acceleration_covariance_[i] = msg->linear_acceleration_covariance[i];
  }
  
  data_received_ = true;
}

}  // namespace imu_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  imu_hardware_interface::IMUHardwareInterface,
  hardware_interface::SystemInterface)