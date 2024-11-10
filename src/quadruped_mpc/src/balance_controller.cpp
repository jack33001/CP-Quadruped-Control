#include "quadruped_mpc/balance_controller.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "std_msgs/msg/string.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace balance_controller {

BalanceController::BalanceController() : controller_interface::ControllerInterface(), F_star_prev_(3, 1, 4) 
{
  F_star_prev_.setZero();
}

CallbackReturn BalanceController::on_init()
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

void BalanceController::init_pinocchio(const std::string& urdf_str) 
{
  // Set up Pinocchio model from the URDF string
  pinocchio::Model model;
  try {
    pinocchio::urdf::buildModelFromXML(urdf_str, model);
    data_ = std::make_unique<pinocchio::Data>(model);
    model_ = std::make_unique<pinocchio::Model>(model);
    RCLCPP_INFO(get_node()->get_logger(), "Successfully built pinocchio model from URDF");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to build pinocchio model from URDF: %s", e.what());
  }
}

CallbackReturn BalanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  command_interfaces_ = get_node()->get_parameter("command_interfaces").as_string_array();
  state_interfaces_ = get_node()->get_parameter("state_interfaces").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints provided");
    return CallbackReturn::ERROR;
  }

  // Subscribe to robot_description topic
  robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
    "/robot_description", 
    rclcpp::QoS(1).transient_local(),  // Use transient local to get the last published message
    [this](const std_msgs::msg::String::SharedPtr msg) {
      init_pinocchio(msg->data);
    });

  // Setup subscriber for commands
  commands_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      rt_command_ptr_.writeFromNonRT(msg->data);
    });

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
BalanceController::command_interface_configuration() const
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
BalanceController::state_interface_configuration() const
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

CallbackReturn BalanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Initialize your controller state here
  return CallbackReturn::SUCCESS;
}

CallbackReturn BalanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Clean up your controller state here
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type BalanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Implement your control logic here
  
  // First, read from all the state interfaces and update variables
  // Next, calculate the control law
  // Finally, write to the control interfaces

  return controller_interface::return_type::OK;
}

void BalanceController::read_state_interfaces()
{

}

}  // namespace balance_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceController, controller_interface::ControllerInterface)