#include "quadruped_mpc/controller_interfaces/balance_controller.hpp"

#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

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

    // Update parameters in solver (foot positions and COM)
    double params[15];  // 3 coordinates × 5 points (4 feet + COM)
    std::copy(p1_.begin(), p1_.end(), params);
    std::copy(p2_.begin(), p2_.end(), params + 3);
    std::copy(p3_.begin(), p3_.end(), params + 6);
    std::copy(p4_.begin(), p4_.end(), params + 9);
    std::copy(com_.begin(), com_.end(), params + 12);

    // Fix: Use the correct generated ACADOS function for parameter setting
    for (int stage = 0; stage <= solver_->nlp_solver_plan->N; stage++) {
        quadruped_ode_acados_update_params(solver_, stage, params, 15);
    }

    // Create single stringstream for all logging
    std::stringstream ss;
    ss << "\n=== Current State ===\n";
    
    // State vector with better formatting
    ss << "State Vector:\n";
    ss << "┌──────────┬────────────┬────────────┐\n";
    ss << "│  State   │   Value    │   Units    │\n";
    ss << "├──────────┼────────────┼────────────┤\n";
    
    const std::array<std::string, 12> state_names = {
        "pos_x", "pos_y", "pos_z",
        "roll", "pitch", "yaw",
        "vel_x", "vel_y", "vel_z",
        "ang_x", "ang_y", "ang_z"
    };
    const std::array<std::string, 12> units = {
        "m", "m", "m",
        "rad", "rad", "rad",
        "m/s", "m/s", "m/s",
        "rad/s", "rad/s", "rad/s"
    };
    
    for (size_t i = 0; i < 12; ++i) {
        ss << "│" << std::setw(10) << state_names[i] << " │"
           << std::setw(10) << std::fixed << std::setprecision(3) << current_state_[i] << " │"
           << std::setw(10) << units[i] << " │\n";
    }
    ss << "└──────────┴────────────┴────────────┘\n\n";
    
    // Foot positions with better formatting
    ss << "Foot Positions:\n";
    ss << "┌──────┬────────────┬────────────┬────────────┐\n";
    ss << "│ Foot │     X      │     Y      │     Z      │\n";
    ss << "├──────┼────────────┼────────────┼────────────┤\n";
    ss << "│  FL  │" << std::setw(10) << std::fixed << std::setprecision(3) << p1_[0] << " │"
       << std::setw(10) << p1_[1] << " │" << std::setw(10) << p1_[2] << " │\n";
    ss << "│  FR  │" << std::setw(10) << p2_[0] << " │"
       << std::setw(10) << p2_[1] << " │" << std::setw(10) << p2_[2] << " │\n";
    ss << "│  RL  │" << std::setw(10) << p3_[0] << " │"
       << std::setw(10) << p3_[1] << " │" << std::setw(10) << p3_[2] << " │\n";
    ss << "│  RR  │" << std::setw(10) << p4_[0] << " │"
       << std::setw(10) << p4_[1] << " │" << std::setw(10) << p4_[2] << " │\n";
    ss << "└──────┴────────────┴────────────┘\n\n";
    
    RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());

    // Fill current state vector according to quadruped_model.py:
    // [x1, y1, z1, theta, phi, psi, vx, vy, vz, wx, wy, wz]
    
    // Position (from center of mass position)
    current_state_[0] = info.state_.pc[0];  // x
    current_state_[1] = info.state_.pc[1];  // y
    current_state_[2] = info.state_.pc[2];  // z

    // Angular position (from IMU orientation)
    // Assuming IMU orientation quaternion is already in state_
    // Convert quaternion to euler angles (roll, pitch, yaw)
    // Note: You might want to add a quaternion-to-euler utility function
    current_state_[3] = info.state_.orientation[0];  // theta (roll)
    current_state_[4] = info.state_.orientation[1];  // phi (pitch)
    current_state_[5] = info.state_.orientation[2];  // psi (yaw)

    // Linear velocity (from center of mass velocity)
    current_state_[6] = info.state_.vc[0];  // vx
    current_state_[7] = info.state_.vc[1];  // vy
    current_state_[8] = info.state_.vc[2];  // vz

    // Angular velocity (directly from IMU)
    current_state_[9] = info.state_.angular_velocity[0];   // wx
    current_state_[10] = info.state_.angular_velocity[1];  // wy
    current_state_[11] = info.state_.angular_velocity[2];  // wz

    // Set current state in solver using the nlp interface
    ocp_nlp_out_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "x", current_state_.data());
    
    // Solve OCP
    RCLCPP_INFO(get_node()->get_logger(), "Solving optimization problem...");
    int solver_status = quadruped_ode_acados_solve(solver_);
    if (solver_status != 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "Solver failed with status %d", solver_status);
        return controller_interface::return_type::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Solver completed successfully");

    // Get optimal control from nlp interface - these are FOOT FORCES, not joint efforts!
    ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "u", optimal_control_.data());
    
    // Log the optimal foot forces first
    ss.str("");  // Clear stringstream
    ss << "\n=== Optimal Forces ===\n";
    ss << "Foot Forces (N):\n";
    ss << "┌──────┬────────────┬────────────┬────────────┐\n";
    ss << "│ Foot │    F_x     │    F_y     │    F_z     │\n";
    ss << "├──────┼────────────┼────────────┼────────────┤\n";
    for (size_t i = 0; i < 4; ++i) {  // Loop through 4 feet
        ss << "│  " << (i==0 ? "FL" : i==1 ? "FR" : i==2 ? "RL" : "RR") << "  │"
           << std::setw(10) << std::scientific << std::setprecision(3) << optimal_control_[i*3] << " │"
           << std::setw(10) << optimal_control_[i*3 + 1] << " │"
           << std::setw(10) << optimal_control_[i*3 + 2] << " │\n";
    }
    ss << "└──────┴────────────┴────────────┘\n\n";

    RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());

    // Convert foot forces to joint efforts using the Jacobian transpose
    std::array<double, 8> joint_efforts;  // 2 joints per leg × 4 legs
    
    // Process each leg (2 joints each)
    // FL leg (indices 0,1)
    Eigen::Vector3d f1(optimal_control_[0], optimal_control_[1], optimal_control_[2]);
    Eigen::Vector2d tau1 = info.state_.J1.transpose() * f1;
    joint_efforts[0] = tau1[0];  // FL hip
    joint_efforts[1] = tau1[1];  // FL knee
    
    // FR leg (indices 2,3)
    Eigen::Vector3d f2(optimal_control_[3], optimal_control_[4], optimal_control_[5]);
    Eigen::Vector2d tau2 = info.state_.J2.transpose() * f2;
    joint_efforts[2] = tau2[0];  // FR hip
    joint_efforts[3] = tau2[1];  // FR knee
    
    // RL leg (indices 4,5)
    Eigen::Vector3d f3(optimal_control_[6], optimal_control_[7], optimal_control_[8]);
    Eigen::Vector2d tau3 = info.state_.J3.transpose() * f3;
    joint_efforts[4] = tau3[0];  // RL hip
    joint_efforts[5] = tau3[1];  // RL knee
    
    // RR leg (indices 6,7)
    Eigen::Vector3d f4(optimal_control_[9], optimal_control_[10], optimal_control_[11]);
    Eigen::Vector2d tau4 = info.state_.J4.transpose() * f4;
    joint_efforts[6] = tau4[0];  // RR hip
    joint_efforts[7] = tau4[1];  // RR knee

    // Add joint efforts table to logging
    ss << "\nJoint Efforts (Nm):\n";
    ss << "┌──────────┬────────────┐\n";
    ss << "│  Joint   │   Effort   │\n";
    ss << "├──────────┼────────────┤\n";
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        ss << "│" << std::setw(10) << joint_names_[i] << " │"
           << std::setw(10) << std::fixed << std::setprecision(3) << joint_efforts[i] << " │\n";
    }
    ss << "└──────────┴────────────┘\n";
    
    RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());

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

  // Change to INFO level
  RCLCPP_INFO(get_node()->get_logger(), "Balance controller configured with %zu joints", joint_names_.size());
  
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
  RCLCPP_INFO(get_node()->get_logger(), "Balance controller activated");
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
