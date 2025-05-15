#ifndef IMU_HARDWARE_INTERFACE_HPP_
#define IMU_HARDWARE_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace imu_hardware_interface
{

class IMUHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IMUHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Orientation values
  double orientation_x_ = 0.0;
  double orientation_y_ = 0.0;
  double orientation_z_ = 0.0;
  double orientation_w_ = 1.0;
  std::array<double, 9> orientation_covariance_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Angular velocity
  double angular_velocity_x_ = 0.0;
  double angular_velocity_y_ = 0.0;
  double angular_velocity_z_ = 0.0;
  std::array<double, 9> angular_velocity_covariance_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Linear acceleration
  double linear_acceleration_x_ = 0.0;
  double linear_acceleration_y_ = 0.0;
  double linear_acceleration_z_ = 0.0;
  std::array<double, 9> linear_acceleration_covariance_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // ROS 2 Components
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  std::shared_ptr<rclcpp::Node> node_;
  
  // Topic configuration
  std::string imu_topic_ = "/imu_sensor";
  
  // Callback to handle IMU data
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  
  // Flag to indicate if new data has been received
  bool data_received_ = false;
};

}  // namespace imu_hardware_interface

#endif  // IMU_HARDWARE_INTERFACE_HPP_