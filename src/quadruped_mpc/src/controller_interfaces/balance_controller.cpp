#include "quadruped_mpc/controller_interfaces/balance_controller.hpp"
#include "quadruped_mpc/control_laws/BalanceControl.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace quadruped_mpc
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

BalanceController::BalanceController() : controller_interface::ControllerInterface() 
{
  fprintf(stderr, "Balance controller constructor called\n");  // Use fprintf since logger isn't ready
}

controller_interface::InterfaceConfiguration 
BalanceController::command_interface_configuration() const
{
  // Change to NONE since balance_controller will no longer write to command interfaces directly
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration 
BalanceController::state_interface_configuration() const
{
  // Empty configuration since we're reading from shared state
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::return_type
BalanceController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!update_state()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to update state");
    return controller_interface::return_type::ERROR;
  }

  if (!update_control()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to update control");
    return controller_interface::return_type::ERROR;
  }

  if (!update_commands()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to update commands");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

BalanceController::CallbackReturn BalanceController::on_init()
{
  fprintf(stderr, "Balance controller on_init starting\n");
  
  try {
    fprintf(stderr, "Balance controller trying to declare parameters\n");
    // Get parameters from yaml
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    fprintf(stderr, "Balance controller parameters declared\n");
    
    fprintf(stderr, "Balance controller on_init completed successfully\n");
    fprintf(stderr, "Balance controller waiting for on_configure\n");
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
}

BalanceController::CallbackReturn BalanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  fprintf(stderr, "Balance controller on_configure starting\n");
  RCLCPP_INFO(get_node()->get_logger(), "Starting balance controller configuration");

  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Balance controller configured with %zu joints", joint_names_.size());

  RCLCPP_INFO(get_node()->get_logger(), "Initializing state arrays");
  // Initialize arrays with zeros
  std::fill(current_state_.begin(), current_state_.end(), 0.0);
  std::fill(desired_state_.begin(), current_state_.end(), 0.0);
  std::fill(optimal_control_.begin(), optimal_control_.end(), 0.0);
  desired_state_[2] = 0.15;  // Set desired height
  desired_state_[3] = 1.0;  // Forward facing quaternion has w=1

  // Find package share directory
  RCLCPP_INFO(get_node()->get_logger(), "Finding ACADOS library path");
  const auto package_path = ament_index_cpp::get_package_share_directory("quadruped_mpc");
  std::string lib_path = package_path + "/include/quadruped_mpc/acados_generated";
  RCLCPP_INFO(get_node()->get_logger(), "ACADOS library path: %s", lib_path.c_str());

  // Check if ACADOS library exists
  std::string lib_file = lib_path + "/libacados_ocp_solver_quadruped_ode.so";
  if (access(lib_file.c_str(), F_OK) != 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "ACADOS library not found at %s", lib_file.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Found ACADOS library at %s", lib_file.c_str());

  // Add to LD_LIBRARY_PATH
  RCLCPP_INFO(get_node()->get_logger(), "Updating LD_LIBRARY_PATH");
  std::string current_ld_path = std::getenv("LD_LIBRARY_PATH") ? std::getenv("LD_LIBRARY_PATH") : "";
  std::string new_ld_path = lib_path + ":" + current_ld_path;
  setenv("LD_LIBRARY_PATH", new_ld_path.c_str(), 1);
  RCLCPP_INFO(get_node()->get_logger(), "LD_LIBRARY_PATH updated to %s", new_ld_path.c_str());

  // Create ACADOS solver capsule
  RCLCPP_INFO(get_node()->get_logger(), "Creating ACADOS solver capsule");
  solver_ = quadruped_ode_acados_create_capsule();
  if (solver_ == nullptr) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create ACADOS solver capsule - library loading error");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "ACADOS solver capsule created");

  // Initialize ACADOS solver
  RCLCPP_INFO(get_node()->get_logger(), "Initializing ACADOS solver");
  int status = quadruped_ode_acados_create(solver_);
  if (status != 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize ACADOS solver with status %d", status);
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "ACADOS solver initialized successfully");

  RCLCPP_INFO(get_node()->get_logger(), "Balance controller configuration completed successfully");

  // Add subscriber initialization before returning
  cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
    "quadruped/cmd/single_state", 10,
    std::bind(&BalanceController::cmd_callback, this, std::placeholders::_1));
  RCLCPP_INFO(get_node()->get_logger(), "Subscribed to quadruped/cmd/single_state topic");

  // Set up state subscriber
  state_sub_ = get_node()->create_subscription<quadruped_msgs::msg::QuadrupedState>(
    "/quadruped/state/state_estimate", 10,
    std::bind(&BalanceController::state_callback, this, std::placeholders::_1));
  RCLCPP_INFO(get_node()->get_logger(), "Subscribed to state estimation topic");

  // Set up gait pattern subscriber
  gait_sub_ = get_node()->create_subscription<quadruped_msgs::msg::GaitPattern>(
    "/quadruped/gait/gait_pattern", 10,
    std::bind(&BalanceController::gait_callback, this, std::placeholders::_1));
  RCLCPP_INFO(get_node()->get_logger(), "Subscribed to gait pattern topic");

  // Set up foot forces publisher
  foot_forces_pub_ = get_node()->create_publisher<quadruped_msgs::msg::FootForces>(
    "/quadruped/commands/forces", rclcpp::SensorDataQoS());
  foot_forces_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<quadruped_msgs::msg::FootForces>>(foot_forces_pub_);
  RCLCPP_INFO(get_node()->get_logger(), "Created foot forces publisher");

  return CallbackReturn::SUCCESS;
}

BalanceController::CallbackReturn BalanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_activate called");
  RCLCPP_INFO(get_node()->get_logger(), "Balance controller activated");
  return CallbackReturn::SUCCESS;
}

BalanceController::CallbackReturn BalanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_deactivate called");
  return CallbackReturn::SUCCESS;
}

void BalanceController::cmd_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  latest_cmd_ = msg;
  new_cmd_received_ = true;
}

void BalanceController::state_callback(const quadruped_msgs::msg::QuadrupedState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_state_ = msg;
  new_state_received_ = true;
}

void BalanceController::gait_callback(const quadruped_msgs::msg::GaitPattern::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(gait_mutex_);
  latest_gait_ = msg;
  new_gait_received_ = true;
}

}  // namespace quadruped_mpc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_mpc::BalanceController, controller_interface::ControllerInterface)
