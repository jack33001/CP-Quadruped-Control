#include "quadruped_mpc/controller_interfaces/foot_controller.hpp"
#include "quadruped_mpc/control_laws/FootControl.hpp"  // Add this include

#include <algorithm>
#include <string>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace quadruped_mpc
{

FootController::FootController()
: controller_interface::ControllerInterface()
{
  // Initialize foot forces
  foot_forces_.has_data = false;
}

controller_interface::CallbackReturn FootController::on_init()
{
  try {
    // Get parameters from node
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>());
    
    controller_name_ = get_node()->get_name();
    
    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during on_init stage with message: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}

controller_interface::CallbackReturn FootController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  num_joints_ = joint_names_.size();
  
  if (num_joints_ == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints provided");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  command_interface_types_ = get_node()->get_parameter("command_interfaces").as_string_array();
  if (command_interface_types_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No command interfaces provided");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Initialize the realtime buffers
  foot_forces_buffer_ = std::make_shared<realtime_tools::RealtimeBuffer<quadruped_msgs::msg::FootForces>>();
  state_buffer_ = std::make_shared<realtime_tools::RealtimeBuffer<quadruped_msgs::msg::QuadrupedState>>();
  gait_buffer_ = std::make_shared<realtime_tools::RealtimeBuffer<quadruped_msgs::msg::GaitPattern>>();
  
  RCLCPP_INFO(get_node()->get_logger(), "Configured FootController with %zu joints", num_joints_);
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FootController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint : joint_names_) {
    for (const auto & interface_type : command_interface_types_) {
      command_interfaces_config.names.push_back(joint + "/" + interface_type);
    }
  }
  
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration FootController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return state_interfaces_config;
}

controller_interface::CallbackReturn FootController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Resize vectors first (clear and reserve instead of resize since we can't default construct these interfaces)
  joint_command_interfaces_.clear();
  
  // Reserve space to avoid reallocations
  joint_command_interfaces_.reserve(num_joints_ * command_interface_types_.size());
  
  // Initialize interfaces by moving them into our vectors
  for (size_t i = 0; i < num_joints_; ++i) {
    for (size_t j = 0; j < command_interface_types_.size(); ++j) {
      joint_command_interfaces_.push_back(std::move(command_interfaces_[i * command_interface_types_.size() + j]));
    }
  }
  
  // Setup subscriber for foot forces
  foot_forces_subscription_ = get_node()->create_subscription<quadruped_msgs::msg::FootForces>(
    "/quadruped/commands/forces", rclcpp::SensorDataQoS(),
    [this](const quadruped_msgs::msg::FootForces::SharedPtr msg) -> void {
      this->footForcesCallback(msg);
    });
    
  // Setup subscriber for state estimation
  state_subscription_ = get_node()->create_subscription<quadruped_msgs::msg::QuadrupedState>(
    "/quadruped/state/state_estimate", rclcpp::SensorDataQoS(),
    [this](const quadruped_msgs::msg::QuadrupedState::SharedPtr msg) -> void {
      this->stateCallback(msg);
    });
  RCLCPP_INFO(get_node()->get_logger(), "Subscribed to state estimation topic");
  
  // Setup subscriber for gait pattern
  gait_subscription_ = get_node()->create_subscription<quadruped_msgs::msg::GaitPattern>(
    "/quadruped/gait/gait_pattern", rclcpp::SensorDataQoS(),
    [this](const quadruped_msgs::msg::GaitPattern::SharedPtr msg) -> void {
      this->gaitCallback(msg);
    });
  RCLCPP_INFO(get_node()->get_logger(), "Subscribed to gait pattern topic");
  
  if (!reset()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "FootController activated successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FootController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Unsubscribe from topics
  foot_forces_subscription_.reset();
  state_subscription_.reset();
  gait_subscription_.reset();
  
  // Clear interfaces
  joint_command_interfaces_.clear();
  
  RCLCPP_INFO(get_node()->get_logger(), "FootController deactivated successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FootController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!read_topics(*this)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read topics");
    return controller_interface::return_type::ERROR;
  }

  if (!determine_forces(*this)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to determine forces");
    return controller_interface::return_type::ERROR;
  }

  // Execute the swing trajectory generation in its own try/catch block
  try {
    if (!swing_trajectory(*this, time)) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to generate swing trajectory");
      // Continue execution even if swing trajectory fails
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in swing_trajectory: %s", e.what());
    // Continue execution even if swing trajectory throws an exception
  }
  
  // Apply Jacobians to convert foot forces to joint torques in its own try/catch block
  try {
    if (!apply_jacobians(*this)) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to apply Jacobians");
      return controller_interface::return_type::ERROR;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in apply_jacobians: %s", e.what());
    return controller_interface::return_type::ERROR;
  }
  
  // If we got here, everything was successful
  return controller_interface::return_type::OK;
}

void FootController::footForcesCallback(const quadruped_msgs::msg::FootForces::SharedPtr msg)
{
  // Remove verbose logging
  foot_forces_buffer_->writeFromNonRT(*msg);
}

void FootController::stateCallback(const quadruped_msgs::msg::QuadrupedState::SharedPtr msg)
{
  // Remove verbose logging
  state_buffer_->writeFromNonRT(*msg);
}

void FootController::gaitCallback(const quadruped_msgs::msg::GaitPattern::SharedPtr msg)
{
  // Remove verbose logging
  gait_buffer_->writeFromNonRT(*msg);
}

bool FootController::reset()
{
  // Reset foot forces commands to all joints
  foot_forces_.has_data = false;
  
  // Send zero commands to all joints
  for (auto & interface : joint_command_interfaces_) {
    // Check the return value to avoid the warning
    if (!interface.set_value(0.0)) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set command interface value during reset");
      return false;
    }
  }
  
  return true;
}

} // namespace quadruped_mpc

// Register the controller as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_mpc::FootController, controller_interface::ControllerInterface)
