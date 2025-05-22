#include "jetson_gpio_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <fstream>
#include <string>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace jetson_interface
{

hardware_interface::CallbackReturn JetsonGPIOInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  try {
    gpio_pin_ = std::stoi(info_.hardware_parameters.at("gpio_pin"));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("JetsonGPIOInterface"),
      "Failed to parse gpio_pin parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state values
  monitored_state_ = 0.0;
  gpio_state_ = 0.0;
  gpio_initialized_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("JetsonGPIOInterface"),
    "Successfully initialized with GPIO pin: %d", gpio_pin_);
    
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetsonGPIOInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("JetsonGPIOInterface"),
    "Configuring GPIO interface");
    
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetsonGPIOInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (gpio_initialized_) {
    if (!unexport_gpio(gpio_pin_)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("JetsonGPIOInterface"),
        "Failed to unexport GPIO pin: %d", gpio_pin_);
    }
    gpio_initialized_ = false;
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetsonGPIOInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize GPIO
  if (!gpio_initialized_) {
    if (!export_gpio(gpio_pin_)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("JetsonGPIOInterface"),
        "Failed to export GPIO pin: %d", gpio_pin_);
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    // Give system time to create GPIO control files
    usleep(100000);
    
    if (!set_gpio_direction(gpio_pin_, "out")) {
      RCLCPP_ERROR(
        rclcpp::get_logger("JetsonGPIOInterface"),
        "Failed to set GPIO direction for pin: %d", gpio_pin_);
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    gpio_initialized_ = true;
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("JetsonGPIOInterface"),
    "Successfully activated hardware interface");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetsonGPIOInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (gpio_initialized_) {
    // Set GPIO low before deactivating
    set_gpio_value(gpio_pin_, 0);
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JetsonGPIOInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Export the state of the monitored hardware interface
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "jetson_gpio", "monitored_state", &monitored_state_));
      
  // Export the current GPIO state
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "jetson_gpio", "gpio_state", &gpio_state_));
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JetsonGPIOInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  // No commands are needed as we're only reacting to another hardware interface's state
  
  return command_interfaces;
}

hardware_interface::return_type JetsonGPIOInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read the current GPIO state
  if (gpio_initialized_) {
    gpio_state_ = static_cast<double>(read_gpio_value(gpio_pin_));
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JetsonGPIOInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // If monitored_state is 1, set GPIO high, otherwise set it low
  if (gpio_initialized_) {
    int value = (std::abs(monitored_state_ - 1.0) < 0.1) ? 1 : 0;
    
    if (!set_gpio_value(gpio_pin_, value)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("JetsonGPIOInterface"),
        "Failed to set GPIO value for pin: %d", gpio_pin_);
      return hardware_interface::return_type::ERROR;
    }
    
    // Update the GPIO state to reflect what we just wrote
    gpio_state_ = static_cast<double>(value);
  }
  
  return hardware_interface::return_type::OK;
}

// Helper methods for GPIO control
bool JetsonGPIOInterface::export_gpio(int pin)
{
  std::ofstream export_file("/sys/class/gpio/export");
  if (!export_file.is_open()) {
    return false;
  }
  export_file << pin;
  return export_file.good();
}

bool JetsonGPIOInterface::unexport_gpio(int pin)
{
  std::ofstream unexport_file("/sys/class/gpio/unexport");
  if (!unexport_file.is_open()) {
    return false;
  }
  unexport_file << pin;
  return unexport_file.good();
}

bool JetsonGPIOInterface::set_gpio_direction(int pin, const std::string & direction)
{
  std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
  std::ofstream direction_file(path);
  if (!direction_file.is_open()) {
    return false;
  }
  direction_file << direction;
  return direction_file.good();
}

bool JetsonGPIOInterface::set_gpio_value(int pin, int value)
{
  std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
  std::ofstream value_file(path);
  if (!value_file.is_open()) {
    return false;
  }
  value_file << value;
  return value_file.good();
}

int JetsonGPIOInterface::read_gpio_value(int pin)
{
  std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
  std::ifstream value_file(path);
  if (!value_file.is_open()) {
    return -1;
  }
  int value;
  value_file >> value;
  return value;
}

}  // namespace jetson_gpio_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  jetson_interface::JetsonGPIOInterface,
  hardware_interface::SystemInterface
)