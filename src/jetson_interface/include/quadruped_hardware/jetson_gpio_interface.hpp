#ifndef JETSON_GPIO_INTERFACE_HPP_
#define JETSON_GPIO_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <cstdint> 

namespace jetson_interface
{

class JetsonGPIOInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JetsonGPIOInterface);

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

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // GPIO pin number to control
  int gpio_pin_;
  
  // State values
  double monitored_state_;
  double gpio_state_;
  
  // Helper methods for GPIO control
  bool export_gpio(int pin);
  bool unexport_gpio(int pin);
  bool set_gpio_direction(int pin, const std::string & direction);
  bool set_gpio_value(int pin, int value);
  int read_gpio_value(int pin);
  
  // Flag to track if GPIO has been initialized
  bool gpio_initialized_;
};

}  // namespace jetson_gpio_interface

#endif  // JETSON_GPIO_INTERFACE_HPP_