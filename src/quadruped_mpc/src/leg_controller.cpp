#include "quadruped_mpc/leg_controller.hpp"
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace leg_controller {

LegController::LegController() : controller_interface::ControllerInterface() {}

CallbackReturn LegController::on_init()
{
  try {
    // Declare parameters
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("state_interfaces", std::vector<std::string>());
    
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn LegController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  command_interfaces_ = get_node()->get_parameter("command_interfaces").as_string_array();
  state_interfaces_ = get_node()->get_parameter("state_interfaces").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints provided");
    return CallbackReturn::ERROR;
  }

  // Setup subscriber for commands
  commands_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      rt_command_ptr_.writeFromNonRT(msg->data);
    });

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
LegController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint : joint_names_) {
    for (const auto & interface : command_interfaces_) {
      config.names.push_back(joint + "/" + interface);
    }
  }
  
  return config;
}

controller_interface::InterfaceConfiguration 
LegController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint : joint_names_) {
    for (const auto & interface : state_interfaces_) {
      config.names.push_back(joint + "/" + interface);
    }
  }
  
  return config;
}

CallbackReturn LegController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Initialize your controller state here
  return CallbackReturn::SUCCESS;
}

CallbackReturn LegController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Clean up your controller state here
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type LegController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Implement your control logic here
  
  // Example of reading commands from real-time buffer
  auto commands = rt_command_ptr_.readFromRT();
  
  if (!commands || commands->empty()) {
    return controller_interface::return_type::OK;
  }

  // Example of writing to command interfaces
  for (size_t i = 0; i < joint_command_interfaces_.size(); ++i) {
    joint_command_interfaces_[i].get().set_value((*commands)[i]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace leg_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(leg_controller::LegController, controller_interface::ControllerInterface)