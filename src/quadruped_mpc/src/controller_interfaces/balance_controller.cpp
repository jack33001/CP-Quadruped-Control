#include "quadruped_mpc/controller_interfaces/balance_controller.hpp"

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
    // [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
    
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
    
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);  // Roll, Pitch, Yaw

    RCLCPP_INFO(get_node()->get_logger(), "Quaternion: [%.3f, %.3f, %.3f, %.3f] (w, x, y, z)", 
        q.w(), q.x(), q.y(), q.z());
    RCLCPP_INFO(get_node()->get_logger(), "Euler angles: [%.3f, %.3f, %.3f] (roll, pitch, yaw)",
          euler[0], euler[1], euler[2]);
    
    // Orientation (as euler angles)
    current_state_[3] = euler[0];  // roll
    current_state_[4] = euler[1];  // pitch
    current_state_[5] = euler[2];  // yaw
    // Linear velocity (from center of mass velocity)
    current_state_[6] = info.state_.vc[0];  // vx
    current_state_[7] = info.state_.vc[1];  // vy
    current_state_[8] = info.state_.vc[2];  // vz

    // Angular velocity (directly from IMU)
    current_state_[9] = info.state_.angular_velocity[0];   // wx
    current_state_[10] = info.state_.angular_velocity[1];  // wy
    current_state_[11] = info.state_.angular_velocity[2];  // wz

    // Foot 1 position
    current_state_[12] = info.state_.p1[0];  // x
    current_state_[13] = info.state_.p1[1];  // y
    current_state_[14] = info.state_.p1[2];  // z

    // Foot 2 position
    current_state_[15] = info.state_.p2[0];  // x
    current_state_[16] = info.state_.p2[1];  // y
    current_state_[17] = info.state_.p2[2];  // z

    // Foot 3 position
    current_state_[18] = info.state_.p3[0];  // x
    current_state_[19] = info.state_.p3[1];  // y
    current_state_[20] = info.state_.p3[2];  // z

    // Foot 4 position
    current_state_[21] = info.state_.p4[0];  // x
    current_state_[22] = info.state_.p4[1];  // y
    current_state_[23] = info.state_.p4[2];  // z

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

    // Get solver state and reference state for comparison before printing tables
    std::array<double, 24> solver_state;
    std::array<double, 24> solver_reference = desired_state_;  // Just use our desired state instead of trying to read from solver
    ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "x", solver_state.data());
    
    // Remove the problematic get operation that was causing the segfault
    ocp_nlp_cost_model_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "yref", solver_reference.data());

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

    // Now generate all tables
    std::stringstream ss;
    ss << "\n╭───────────────────────────────────────────────────────────────────────────────────────────────────────────────────╮\n";
    ss << "│                                                 State Vector                                                        │\n";
    ss << "├───────────────┬──────────────────┬──────────────────┬──────────────────┬──────────────────┬──────────────────┬────┤\n";
    ss << "│ Variable      │ Input State      │ Solver State     │ Solver Reference │ Desired State    │ Error            │\n";
    ss << "├───────────────┼──────────────────┼──────────────────┼──────────────────┼──────────────────┼──────────────────┤\n";
    
    for (int i = 0; i < 24; i++) {
        const char* var_names[] = {"x", "y", "z", 
                                 "roll", "pitch", "yaw", 
                                 "vel_x", "vel_y", "vel_z", 
                                 "ang_vel_x", "ang_vel_y", "ang_vel_z",
                                 "foot1_x", "foot1_y", "foot1_z",
                                 "foot2_x", "foot2_y", "foot2_z",
                                 "foot3_x", "foot3_y", "foot3_z",
                                 "foot4_x", "foot4_y", "foot4_z"};

        double error = desired_state_[i] - current_state_[i];
        ss << "│ " << std::left << std::setw(12) << var_names[i] 
           << "  │" << std::right << std::setw(16) << std::fixed << std::setprecision(3) << current_state_[i] 
           << "  │" << std::setw(16) << solver_state[i]
           << "  │" << std::setw(16) << solver_reference[i]
           << "  │" << std::setw(16) << desired_state_[i]
           << "  │" << std::setw(16) << error << "  │\n";
    }

    // Add bottom border to state vector table
    ss << "╰───────────────┴──────────────────┴──────────────────┴──────────────────┴──────────────────┴──────────────────╯\n\n";

    // Add core states prediction table
    ss << "╭──────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────╮\n";
    ss << "│                                            Core  States Prediction                                          │\n";
    ss << "├──────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┤\n";
    ss << "│ Stage    │    x   │    y   │    z   │  roll  │  pitch │   yaw  │ vel_x  │ vel_y  │ vel_z  │ ang_x  │ ang_y  │\n";
    ss << "├──────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┤\n";

    // Get states for each prediction step
    std::array<double, 24> step_state;
    for (int step = 0; step < 20; step++) {
        ss << "│ Step " << std::left << std::setw(3) << step << " │";
        ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, step, "x", step_state.data());
        // Print first 12 states (core states)
        for (int i = 0; i < 12; i++) {
            ss << std::right << std::setw(7) << std::fixed << std::setprecision(2) << step_state[i] << " │";
        }
        ss << "\n";
    }

    // Add bottom border to core states table
    ss << "╰───────────┴───────┴───────┴───────┴───────┴───────┴───────┴───────┴───────┴───────┴───────┴───────╯\n\n";

    // Add foot positions prediction table
    ss << "╭──────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────╮\n";
    ss << "│                                                Foot Positions Prediction                                             │\n";
    ss << "├──────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┤\n";
    ss << "│ Stage    │ FL-x   │ FL-y   │ FL-z   │ FR-x   │ FR-y   │ FR-z   │ RL-x   │ RL-y   │ RL-z   │ RR-x   │ RR-y   │ RR-z   │\n";
    ss << "├──────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┤\n";

    for (int step = 0; step < 20; step++) {
        ss << "│ Step " << std::left << std::setw(3) << step << " │";
        ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, step, "x", step_state.data());
        // Print foot positions (last 12 states)
        for (int i = 12; i < 24; i++) {
            ss << std::right << std::setw(7) << std::fixed << std::setprecision(2) << step_state[i] << " │";
        }
        ss << "\n";
    }

    // Add bottom border to foot positions table
    ss << "╰─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────╯\n\n";

    // Add control prediction table
    ss << "╭──────────┬────────────────────────┬────────────────────────┬────────────────────────┬────────────────────────╮\n";
    ss << "│                                         Control Prediction                                                   │\n";
    ss << "├──────────┬────────────────────────┬────────────────────────┬────────────────────────┬────────────────────────┤\n";
    ss << "│ Stage    │     FL Forces (xyz)    │    FR Forces (xyz)     │     RL Forces (xyz)    │    RR Forces (xyz)     │\n";
    ss << "├──────────┼────────────────────────┼────────────────────────┼────────────────────────┼────────────────────────┤\n";

    // Get control for each prediction step
    std::array<double, 12> step_control;
    for (int step = 0; step < 20; step++) {
        ss << "│ Step " << std::left << std::setw(3) << step << " │";
        ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, step, "u", step_control.data());
        
        // Print FL forces
        ss << std::right << std::setw(6) << std::fixed << std::setprecision(1) << step_control[0] << ", "
           << std::setw(6) << step_control[1] << ", "
           << std::setw(6) << step_control[2] << "  │";
           
        // Print FR forces
        ss << std::right << std::setw(6) << step_control[3] << ", "
           << std::setw(6) << step_control[4] << ", "
           << std::setw(6) << step_control[5] << "  │";
           
        // Print RL forces
        ss << std::right << std::setw(6) << step_control[6] << ", "
           << std::setw(6) << step_control[7] << ", "
           << std::setw(6) << step_control[8] << "  │";
           
        // Print RR forces
        ss << std::right << std::setw(6) << step_control[9] << ", "
           << std::setw(6) << step_control[10] << ", "
           << std::setw(6) << step_control[11] << "  │\n";
    }

    // Add bottom border to control prediction table
    ss << "╰──────────┴────────────────────────┴────────────────────────┴────────────────────────┴────────────────────────╯\n\n";

    // Add controller outputs table
    ss << "╭─────────────────────────────────────────────────────────────────────────────────╮\n";
    ss << "│                              Controller Outputs                                 │\n";
    ss << "├───────────────┬──────────────────────────────┬──────────────────────────────────┤\n";
    ss << "│ Leg           │ Foot Forces (X, Y, Z)        │ Joint Efforts (Hip, Knee)        │\n";
    ss << "├───────────────┼──────────────────────────────┼──────────────────────────────────┤\n";
    ss << "│ FL            │" << std::setw(8) << optimal_control_[0] << ", " 
                             << std::setw(8) << optimal_control_[1] << ", "
                             << std::setw(8) << optimal_control_[2] << "  │" 
                             << std::setw(8) << joint_efforts[0] << ", "
                             << std::setw(8) << joint_efforts[1] << "                │\n";
    ss << "│ FR            │" << std::setw(8) << optimal_control_[3] << ", " 
                             << std::setw(8) << optimal_control_[4] << ", "
                             << std::setw(8) << optimal_control_[5] << "  │"
                             << std::setw(8) << joint_efforts[2] << ", "
                             << std::setw(8) << joint_efforts[3] << "                │\n";
    ss << "│ RL            │" << std::setw(8) << optimal_control_[6] << ", " 
                             << std::setw(8) << optimal_control_[7] << ", "
                             << std::setw(8) << optimal_control_[8] << "  │"
                             << std::setw(8) << joint_efforts[4] << ", "
                             << std::setw(8) << joint_efforts[5] << "                │\n";
    ss << "│ RR            │" << std::setw(8) << optimal_control_[9] << ", " 
                             << std::setw(8) << optimal_control_[10] << ", "
                             << std::setw(8) << optimal_control_[11] << "  │"
                             << std::setw(8) << joint_efforts[6] << ", "
                             << std::setw(8) << joint_efforts[7] << "                │\n";
    ss << "├───────────────┼──────────────────────────────┼──────────────────────────────────┤\n";
    
    // Calculate total forces
    double total_fx = optimal_control_[0] + optimal_control_[3] + optimal_control_[6] + optimal_control_[9];
    double total_fy = optimal_control_[1] + optimal_control_[4] + optimal_control_[7] + optimal_control_[10];
    double total_fz = optimal_control_[2] + optimal_control_[5] + optimal_control_[8] + optimal_control_[11];
    
    ss << "│ Total Force   │" << std::setw(8) << total_fx << ", " 
                             << std::setw(8) << total_fy << ", "
                             << std::setw(8) << total_fz << "  │                                  │\n";
    ss << "╰───────────────┴──────────────────────────────┴──────────────────────────────────╯\n\n";

    // Log all tables to terminal
    RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());

    // Add update divider
    RCLCPP_INFO(get_node()->get_logger(), 
        "\n==============================================================================================\n"
        "                                           End of Step                                        \n"
        "==============================================================================================\n");

    // Store command result for hardware interface using computed joint efforts
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      // Set output small to keep the simulation stable
        auto result = command_interfaces_[i].set_value(joint_efforts[i]);
        //auto result = command_interfaces_[i].set_value(0);
        if (!result) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set command for joint %zu", i);
        }
    }

    throw std::runtime_error("Balance control test error");

    return controller_interface::return_type::OK;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in balance control: %s", e.what());
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
  desired_state_[2] = 0.18;  // Set desired height

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

}  // namespace quadruped_mpc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_mpc::BalanceController, controller_interface::ControllerInterface)
