#include "quadruped_mpc/controller_interfaces/balance_controller.hpp"
#include "quadruped_mpc/control_laws/BalanceControl.hpp"

#include <string>
#include <vector>
#include <unistd.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace quadruped_mpc
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

BalanceController::BalanceController() : controller_interface::ControllerInterface() 
{
  fprintf(stderr, "Balance controller constructor called\n");
}

controller_interface::InterfaceConfiguration 
BalanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration 
BalanceController::state_interface_configuration() const
{
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
  try {
    // Get parameters from yaml
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    RCLCPP_INFO(get_node()->get_logger(), "Balance controller initialized");
    
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

BalanceController::CallbackReturn BalanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Starting balance controller configuration");

  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
    return CallbackReturn::ERROR;
  }

  // Initialize arrays with zeros
  std::fill(current_state_.begin(), current_state_.end(), 0.0);
  std::fill(desired_state_.begin(), current_state_.end(), 0.0);
  std::fill(optimal_control_.begin(), optimal_control_.end(), 0.0);
  desired_state_[2] = 0.15;  // Set desired height
  desired_state_[3] = 1.0;  // Forward facing quaternion has w=1

  // Get stages parameter for prediction arrays
  int stages = 50;  // Default value
  try {
    get_node()->declare_parameter("stages", 50);
    stages = get_node()->get_parameter("stages").as_int();
    RCLCPP_INFO(get_node()->get_logger(), "Using %d stages for prediction arrays", stages);
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_node()->get_logger(), "Error getting stages parameter: %s. Using default of 50", e.what());
  }

  // Initialize prediction arrays with default values
  foot_states_prediction_.resize(stages);
  foot_phases_prediction_.resize(stages);
  
  // Fill prediction arrays with default values (-2 for states, 0 for phases)
  for (auto& state_array : foot_states_prediction_) {
    state_array.fill(-2);  // -2 as default/invalid state value
  }
  
  for (auto& phase_array : foot_phases_prediction_) {
    phase_array.fill(0.0f);  // 0.0 as default phase value
  }

  // Read the COM offset from parameters
  try {
    // Declare and get the COM offset parameter with default values
    get_node()->declare_parameter("com_offset", std::vector<double>{0.0, 0.0, -0.1});
    com_offset_ = get_node()->get_parameter("com_offset").as_double_array();
    
    RCLCPP_INFO(get_node()->get_logger(), "Using COM offset: [%.3f, %.3f, %.3f]", 
                com_offset_[0], com_offset_[1], com_offset_[2]);
                
    // Apply the COM offset to the desired state immediately
    desired_state_[0] += com_offset_[0];
    desired_state_[1] += com_offset_[1];
    desired_state_[2] += com_offset_[2];
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_node()->get_logger(), "Error getting COM offset: %s. Using defaults [0.0, 0.0, -0.1]", e.what());
    com_offset_ = {0.0, 0.0, -0.1};
  }

  // Find package share directory
  const auto package_path = ament_index_cpp::get_package_share_directory("quadruped_mpc");
  std::string lib_path = package_path + "/include/quadruped_mpc/acados_generated";

  // Check if ACADOS library exists
  std::string lib_file = lib_path + "/libacados_ocp_solver_quadruped_ode.so";
  if (access(lib_file.c_str(), F_OK) != 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "ACADOS library not found at %s", lib_file.c_str());
    return CallbackReturn::ERROR;
  }

  // Add to LD_LIBRARY_PATH
  std::string current_ld_path = std::getenv("LD_LIBRARY_PATH") ? std::getenv("LD_LIBRARY_PATH") : "";
  std::string new_ld_path = lib_path + ":" + current_ld_path;
  setenv("LD_LIBRARY_PATH", new_ld_path.c_str(), 1);
  RCLCPP_INFO(get_node()->get_logger(), "Updated LD_LIBRARY_PATH with ACADOS path");

  // Create ACADOS solver capsule
  solver_ = quadruped_ode_acados_create_capsule();
  if (solver_ == nullptr) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create ACADOS solver capsule");
    return CallbackReturn::ERROR;
  }

  // Initialize ACADOS solver
  int status = quadruped_ode_acados_create(solver_);
  if (status != 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize ACADOS solver with status %d", status);
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "ACADOS solver initialized successfully");

  // Subscribe to command topics
  pose_cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
    "/quadruped/cmd/pose_cmd", 10,
    std::bind(&BalanceController::pose_cmd_callback, this, std::placeholders::_1));
  
  twist_cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "/quadruped/cmd/twist_cmd", 10,
    std::bind(&BalanceController::twist_cmd_callback, this, std::placeholders::_1));

  // Set up state subscriber
  state_sub_ = get_node()->create_subscription<quadruped_msgs::msg::QuadrupedState>(
    "/quadruped/state/state_estimate", 10,
    std::bind(&BalanceController::state_callback, this, std::placeholders::_1));

  // Set up gait pattern subscriber
  gait_sub_ = get_node()->create_subscription<quadruped_msgs::msg::GaitPattern>(
    "/quadruped/gait/gait_pattern", 10,
    std::bind(&BalanceController::gait_callback, this, std::placeholders::_1));

  // Set up foot forces publisher
  foot_forces_pub_ = get_node()->create_publisher<quadruped_msgs::msg::FootForces>(
    "/quadruped/commands/forces", rclcpp::SensorDataQoS());
  foot_forces_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<quadruped_msgs::msg::FootForces>>(foot_forces_pub_);

  RCLCPP_INFO(get_node()->get_logger(), "Balance controller configuration completed");
  return CallbackReturn::SUCCESS;
}

BalanceController::CallbackReturn BalanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Balance controller activated");
  return CallbackReturn::SUCCESS;
}

BalanceController::CallbackReturn BalanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Balance controller deactivated");
  return CallbackReturn::SUCCESS;
}

void BalanceController::pose_cmd_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  latest_pose_cmd_ = msg;
  new_pose_cmd_received_ = true;
}

void BalanceController::twist_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  latest_twist_cmd_ = msg;
  new_twist_cmd_received_ = true;
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
