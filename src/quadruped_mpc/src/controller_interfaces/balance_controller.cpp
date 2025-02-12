#include "quadruped_mpc/controller_interfaces/balance_controller.hpp"
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
  // Empty configuration since we're reading from shared state
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::return_type
BalanceController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  try {
    static bool first_update = true;
    static int update_count = 0;
    update_count++;
    
    if (first_update) {
      RCLCPP_INFO(get_node()->get_logger(), "Balance controller received first update call");
      first_update = false;
    }

    // Read from shared quadruped info
    auto& info = SharedQuadrupedInfo::getInstance();
    std::lock_guard<std::mutex> lock(info.mutex_);

    if (!info.is_initialized_) {
      if (update_count % 100 == 0) {
        RCLCPP_INFO(get_node()->get_logger(), "Waiting for initialization...");
      }
      return controller_interface::return_type::OK;
    }

    // Update foot positions for the solver
    p1_ = {info.state_.p1[0], info.state_.p1[1], info.state_.p1[2]};
    p2_ = {info.state_.p2[0], info.state_.p2[1], info.state_.p2[2]};
    p3_ = {info.state_.p3[0], info.state_.p3[1], info.state_.p3[2]};
    p4_ = {info.state_.p4[0], info.state_.p4[1], info.state_.p4[2]};
    com_ = {info.state_.pc[0], info.state_.pc[1], info.state_.pc[2]};

    // Fill current state vector according to quadruped_model.py:
    // [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz, p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z, p4x, p4y, p4z]
    // Position (from center of mass position)
    current_state_[0] = info.state_.pc[0];  // x
    current_state_[1] = info.state_.pc[1];  // y
    current_state_[2] = info.state_.pc[2];  // z

    // Convert quaternion to euler angles once
    Eigen::Quaterniond q(
        info.state_.orientation_quat[3],  // w
        info.state_.orientation_quat[0],  // x
        info.state_.orientation_quat[1],  // y
        info.state_.orientation_quat[2]   // z
    );
    
    // Orientation (as quaternion) - use coeffs() instead of w(), x(), y(), z()
    current_state_[3] = q.coeffs()[3];  // w
    current_state_[4] = q.coeffs()[0];  // x
    current_state_[5] = q.coeffs()[1];  // y
    current_state_[6] = q.coeffs()[2];  // z

    // Linear velocity (from center of mass velocity)
    current_state_[7] = info.state_.vc[0];  // vx
    current_state_[8] = info.state_.vc[1];  // vy
    current_state_[9] = info.state_.vc[2];  // vz

    // Angular velocity (directly from IMU)
    current_state_[10] = info.state_.angular_velocity[0];   // wx
    current_state_[11] = info.state_.angular_velocity[1];  // wy
    current_state_[12] = info.state_.angular_velocity[2];  // wz

    // Foot positions
    for (int i = 0; i < 3; i++) {
      current_state_[13 + i] = info.state_.p1[i];  // Foot 1
      current_state_[16 + i] = info.state_.p2[i];  // Foot 2
      current_state_[19 + i] = info.state_.p3[i];  // Foot 3
      current_state_[22 + i] = info.state_.p4[i];  // Foot 4
    }

    // Update desired state from latest command if available
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (new_cmd_received_ && latest_cmd_) {
        desired_state_[0] = latest_cmd_->position.x;    // x
        desired_state_[1] = latest_cmd_->position.y;    // y
        desired_state_[2] = latest_cmd_->position.z;    // z
        desired_state_[3] = latest_cmd_->orientation.w;  // qw
        desired_state_[4] = latest_cmd_->orientation.x;  // qx
        desired_state_[5] = latest_cmd_->orientation.y;  // qy
        desired_state_[6] = latest_cmd_->orientation.z;  // qz
        new_cmd_received_ = false;
      }
    }

    // Set desired state as reference for all nodes in prediction horizon
    for (int stage = 0; stage <= solver_->nlp_solver_plan->N; stage++) {
        ocp_nlp_cost_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, stage, "yref", desired_state_.data());
    }

    // Set initial state and solve the optimization problem
    ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "lbx", current_state_.data());
    ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "ubx", current_state_.data());
    
    int solve_status = quadruped_ode_acados_solve(solver_);
    if (solve_status != 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "ACADOS solver failed with status %d", solve_status);
        return controller_interface::return_type::ERROR;
    }

    // Get optimal control from nlp interface - these are FOOT FORCES, not joint efforts!
    ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "u", optimal_control_.data());

    // Convert foot forces to joint efforts using the Jacobian transpose
    std::array<double, 8> joint_efforts{};  // Initialize all elements to 0
    double scaler = 1;

    // Process each leg (2 joints each)
    // FL leg (indices 0,1)
    Eigen::Vector3d f1(optimal_control_[0], optimal_control_[1], optimal_control_[2]);
    Eigen::Vector2d tau1 = info.state_.J1.transpose() * -f1 * scaler;
    joint_efforts[0] = tau1[0];  // FL hip
    joint_efforts[1] = tau1[1];  // FL knee

    // FR leg (indices 2,3)
    Eigen::Vector3d f2(optimal_control_[3], optimal_control_[4], optimal_control_[5]);
    Eigen::Vector2d tau2 = info.state_.J2.transpose() * -f2 * scaler;
    joint_efforts[2] = tau2[0];  // FR hip
    joint_efforts[3] = tau2[1];  // FR knee
    
    // RL leg (indices 4,5)
    Eigen::Vector3d f3(optimal_control_[6], optimal_control_[7], optimal_control_[8]);
    Eigen::Vector2d tau3 = info.state_.J3.transpose() * -f3 * scaler;
    joint_efforts[4] = tau3[0];  // RL hip
    joint_efforts[5] = tau3[1];  // RL knee
    
    // RR leg (indices 6,7)
    Eigen::Vector3d f4(optimal_control_[9], optimal_control_[10], optimal_control_[11]);
    Eigen::Vector2d tau4 = info.state_.J4.transpose() * -f4 * scaler;
    joint_efforts[6] = tau4[0];  // RR hip
    joint_efforts[7] = tau4[1];  // RR knee

    // Store command result for hardware interface using computed joint efforts
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        auto result = command_interfaces_[i].set_value(joint_efforts[i]);
        if (!result) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set command for joint %zu", i);
        }
    }

    return controller_interface::return_type::OK;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in balance control: %s", e.what());
    return controller_interface::return_type::ERROR;
  }
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
  //RCLCPP_INFO(get_node()->get_logger(), "ACADOS solver initialized successfully");

  //RCLCPP_INFO(get_node()->get_logger(), "Balance controller configuration completed successfully");

  // Add subscriber initialization before returning
  cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
    "quadruped_cmd", 10,
    std::bind(&BalanceController::cmd_callback, this, std::placeholders::_1));
  RCLCPP_INFO(get_node()->get_logger(), "Subscribed to quadruped_cmd topic");

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

}  // namespace quadruped_mpc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_mpc::BalanceController, controller_interface::ControllerInterface)
