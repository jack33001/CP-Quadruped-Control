#ifndef MY_CONTROLLER_HPP_
#define MY_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace leg_controller {

class LegController : public controller_interface::ControllerInterface
{
public:
  LegController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  

protected:
  // Parameters
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interfaces_;
  std::vector<std::string> state_interfaces_;

  // Command subscribers and publishers
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr commands_subscriber_;
  
  // Real-time safe buffers
  realtime_tools::RealtimeBuffer<std::vector<double>> rt_command_ptr_;

  // Command interfaces
  std::vector<std::string> command_interface_types_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_command_interfaces_;

  // State interfaces
  std::vector<std::string> state_interface_types_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_state_interfaces_;
};

} // namespace leg_controller

#endif  // MY_CONTROLLER_HPP_