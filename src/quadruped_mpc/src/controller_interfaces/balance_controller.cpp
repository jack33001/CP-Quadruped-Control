#include "quadruped_mpc/controller_interfaces/balance_controller.hpp"

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace quadruped_mpc
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

BalanceController::BalanceController() : controller_interface::ControllerInterface() {}

controller_interface::InterfaceConfiguration 
BalanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Add command interfaces for each joint
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/effort");
  }
  
  return config;
}

controller_interface::InterfaceConfiguration 
BalanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Add state interfaces for each joint
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
    config.names.push_back(joint + "/effort");
  }
  
  // Add IMU interfaces
  config.names.push_back("imu_sensor/orientation.x");
  config.names.push_back("imu_sensor/orientation.y");
  config.names.push_back("imu_sensor/orientation.z");
  config.names.push_back("imu_sensor/orientation.w");
  config.names.push_back("imu_sensor/angular_velocity.x");
  config.names.push_back("imu_sensor/angular_velocity.y");
  config.names.push_back("imu_sensor/angular_velocity.z");
  
  return config;
}

controller_interface::return_type
BalanceController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  try {
    // Read from shared quadruped info
    auto& info = SharedQuadrupedInfo::getInstance();
    std::lock_guard<std::mutex> lock(info.mutex_);

    if (!info.is_initialized_) {
      return controller_interface::return_type::OK;
    }

    // Read current state from state interfaces
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      current_state_[i] = state_interfaces_[i * 3].get_value();  // position
      current_state_[i + joint_names_.size()] = state_interfaces_[i * 3 + 1].get_value();  // velocity
    }

    // Set current state in solver using the nlp interface
    ocp_nlp_out_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "x", current_state_.data());
    
    // Solve OCP
    int solver_status = quadruped_ode_acados_solve(solver_);
    if (solver_status != 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Solver failed with status %d", solver_status);
      return controller_interface::return_type::ERROR;
    }
    
    // Get optimal control from nlp interface
    ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "u", optimal_control_.data());
    
    // Store command result for hardware interface
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto result = command_interfaces_[i].set_value(optimal_control_[i]);
      if (!result) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set command for joint %zu", i);
      }
    }

    return controller_interface::return_type::OK;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in balance control: %s", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

BalanceController::CallbackReturn BalanceController::on_init()
{
  try {
    // Get parameters from yaml
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
}

BalanceController::CallbackReturn BalanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
    return CallbackReturn::ERROR;
  }
  
  // Initialize arrays with zeros
  std::fill(current_state_.begin(), current_state_.end(), 0.0);
  std::fill(desired_state_.begin(), desired_state_.end(), 0.0);
  std::fill(optimal_control_.begin(), optimal_control_.end(), 0.0);
  
  // Create and initialize ACADOS solver
  solver_ = quadruped_ode_acados_create_capsule();
  if (solver_ == nullptr) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create ACADOS solver capsule");
    return CallbackReturn::ERROR;
  }

  int status = quadruped_ode_acados_create(solver_);
  if (status != 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize ACADOS solver");
    return CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

BalanceController::CallbackReturn BalanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // The interfaces are already stored in command_interfaces_ and state_interfaces_
  // by the controller manager, so we don't need to do anything here
  return CallbackReturn::SUCCESS;
}

BalanceController::CallbackReturn BalanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

}  // namespace quadruped_mpc

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  quadruped_mpc::BalanceController, controller_interface::ControllerInterface)
