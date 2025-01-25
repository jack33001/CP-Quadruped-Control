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

    // Update parameters in solver (foot positions and COM)
    double params[15];  // 3 coordinates × 5 points (4 feet + COM)
    std::copy(p1_.begin(), p1_.end(), params);      // FL foot (0-2)
    std::copy(p2_.begin(), p2_.end(), params + 3);  // FR foot (3-5)
    std::copy(p3_.begin(), p3_.end(), params + 6);  // RL foot (6-8)
    std::copy(p4_.begin(), p4_.end(), params + 9);  // RR foot (9-11)
    std::copy(com_.begin(), com_.end(), params + 12); // COM (12-14)

    // Set parameters for all nodes in the prediction horizon
    // This matches the Python solver's update_foot_positions behavior
    for (int stage = 0; stage <= solver_->nlp_solver_plan->N; stage++) {
        int status = quadruped_ode_acados_update_params(solver_, stage, params, 15);
        if (status != 0) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to update parameters at stage %d with status %d", stage, status);
            return controller_interface::return_type::ERROR;
        }
    }

    // Fill current state vector according to quadruped_model.py:
    // [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
    
    // Position (from center of mass position)
    current_state_[0] = info.state_.pc[0];  // x
    current_state_[1] = info.state_.pc[1];  // y
    current_state_[2] = info.state_.pc[2];  // z

    // Convert quaternion to euler angles once
    Eigen::Quaterniond q(
        info.state_.orientation_quat[0],  // w
        info.state_.orientation_quat[1],  // x
        info.state_.orientation_quat[2],  // y
        info.state_.orientation_quat[3]   // z
    );
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);  // Roll, Pitch, Yaw
    
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

    // Set current state in solver using the nlp interface
    ocp_nlp_out_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "x", current_state_.data());

    // Get solver state for comparison before printing tables
    std::array<double, 12> solver_state;
    ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "x", solver_state.data());

    /*
    // Print state vector table
    std::stringstream ss;
    ss << "\n╭─────────────────────────────────────────────────────╮\n";
    ss << "│                    State Vector                     │\n";
    ss << "├───────────────┬──────────────────┬──────────────────┤\n";
    ss << "│ Variable      │ Input State      │ Solver State     │\n";
    ss << "├───────────────┼──────────────────┼──────────────────┤\n";
    ss << "│ x             │" << std::setw(16) << std::fixed << std::setprecision(3) << current_state_[0] 
       << "  │" << std::setw(16) << solver_state[0] << "  │\n";
    // ... table content ...
    ss << "│ ang_vel_z     │" << std::setw(16) << current_state_[11] 
       << "  │" << std::setw(16) << solver_state[11] << "  │\n";
    ss << "╰───────────────┴──────────────────┴──────────────────╯\n\n";

    // Add parameter table showing what we're setting in the solver
    ss << "╭───────────────────────────────────────────────────────────────────────╮\n";
    ss << "│                         Parameter Values                              │\n";
    ss << "├───────────────┬──────────┬──────────┬─────────────┬─────────────────┤\n";
    ss << "│ Point         │    X     │    Y     │     Z       │  Stage Values   │\n";
    ss << "├───────────────┼──────────┼──────────┼─────────────┼─────────────────┤\n";
    // ... table content ...
    ss << "╰───────────────┴──────────┴──────────┴─────────────┴─────────────────╯\n\n";

    // Log both tables to terminal
    RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());

    // Add update divider
    RCLCPP_INFO(get_node()->get_logger(), 
        "\n====================================================================\n"
        "                          End of Update                               \n"
        "====================================================================\n");
    */

    // Solve OCP
    //RCLCPP_INFO(get_node()->get_logger(), "Solving optimization problem...");
    int solver_status = quadruped_ode_acados_solve(solver_);
    if (solver_status != 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "Solver failed with status %d", solver_status);
        return controller_interface::return_type::ERROR;
    }
    //RCLCPP_INFO(get_node()->get_logger(), "Solver completed successfully");
    
    // Get optimal control from nlp interface - these are FOOT FORCES, not joint efforts!
    ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "u", optimal_control_.data());

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

    // Store command result for hardware interface using computed joint efforts
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      // Set output small to keep the simulation stable
        //auto result = command_interfaces_[i].set_value(joint_efforts[i]);
        auto result = command_interfaces_[i].set_value(.01);
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
