#ifndef QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_
#define QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_

#include "quadruped_mpc/controller_interfaces/balance_controller.hpp"
#include <Eigen/Dense>  // Add this include for Eigen matrices

namespace quadruped_mpc
{

inline void BalanceController::update_state()
{
  static bool first_update = true;
  static int update_count = 0;
  update_count++;
  
  if (first_update) {
    RCLCPP_INFO(get_node()->get_logger(), "Balance controller received first update call");
    first_update = false;
  }

  // Check if we have received state data
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_state_) {
      if (update_count % 100 == 0) {
        RCLCPP_INFO(get_node()->get_logger(), "Waiting for state estimation data...");
      }
      throw std::runtime_error("No state estimation data received");
    }

    // Fill current state vector
    current_state_[0] = latest_state_->pc.x;
    current_state_[1] = latest_state_->pc.y;
    current_state_[2] = latest_state_->pc.z;

    // Orientation quaternion is already in the correct order in the message
    current_state_[3] = latest_state_->orientation.w;
    current_state_[4] = latest_state_->orientation.x;
    current_state_[5] = latest_state_->orientation.y;
    current_state_[6] = latest_state_->orientation.z;

    // Linear and angular velocities
    current_state_[7] = latest_state_->com_velocity.x;
    current_state_[8] = latest_state_->com_velocity.y;
    current_state_[9] = latest_state_->com_velocity.z;

    current_state_[10] = latest_state_->angular_velocity.x;
    current_state_[11] = latest_state_->angular_velocity.y;
    current_state_[12] = latest_state_->angular_velocity.z;

    // Foot positions
    current_state_[13] = latest_state_->p1.x;
    current_state_[14] = latest_state_->p1.y;
    current_state_[15] = latest_state_->p1.z;
    current_state_[16] = latest_state_->p2.x;
    current_state_[17] = latest_state_->p2.y;
    current_state_[18] = latest_state_->p2.z;
    current_state_[19] = latest_state_->p3.x;
    current_state_[20] = latest_state_->p3.y;
    current_state_[21] = latest_state_->p3.z;
    current_state_[22] = latest_state_->p4.x;
    current_state_[23] = latest_state_->p4.y;
    current_state_[24] = latest_state_->p4.z;
  }

  // Handle desired state updates from commands
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (new_cmd_received_ && latest_cmd_) {
      desired_state_[0] = latest_cmd_->position.x;
      desired_state_[1] = latest_cmd_->position.y;
      desired_state_[2] = latest_cmd_->position.z;
      desired_state_[3] = latest_cmd_->orientation.w;
      desired_state_[4] = latest_cmd_->orientation.x;
      desired_state_[5] = latest_cmd_->orientation.y;
      desired_state_[6] = latest_cmd_->orientation.z;
      new_cmd_received_ = false;
    }
  }
}

inline void BalanceController::update_control()
{
  for (int stage = 0; stage <= solver_->nlp_solver_plan->N; stage++) {
    ocp_nlp_cost_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, stage, "yref", desired_state_.data());
  }

  ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "lbx", current_state_.data());
  ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "ubx", current_state_.data());
  
  int solve_status = quadruped_ode_acados_solve(solver_);
  if (solve_status != 0) {
    throw std::runtime_error("ACADOS solver failed with status " + std::to_string(solve_status));
  }

  ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "u", optimal_control_.data());
}

inline void BalanceController::update_commands()
{
  std::array<double, 8> computed_efforts{};
  double scaler = 1;

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!latest_state_) {
    throw std::runtime_error("No state data available for computing efforts");
  }

  // Convert vector Jacobians back to Eigen matrices (3x2)
  // Note: Reconstructing in column-major order to match Pinocchio's output
  Eigen::Matrix<double, 3, 2> J1, J2, J3, J4;
  
  J1 << latest_state_->j1[0], latest_state_->j1[3],
        latest_state_->j1[1], latest_state_->j1[4],
        latest_state_->j1[2], latest_state_->j1[5];
  
  J2 << latest_state_->j2[0], latest_state_->j2[3],
        latest_state_->j2[1], latest_state_->j2[4],
        latest_state_->j2[2], latest_state_->j2[5];
  
  J3 << latest_state_->j3[0], latest_state_->j3[3],
        latest_state_->j3[1], latest_state_->j3[4],
        latest_state_->j3[2], latest_state_->j3[5];
  
  J4 << latest_state_->j4[0], latest_state_->j4[3],
        latest_state_->j4[1], latest_state_->j4[4],
        latest_state_->j4[2], latest_state_->j4[5];

  // Compute joint efforts using the Jacobians
  Eigen::Vector3d f1(optimal_control_[0], optimal_control_[1], optimal_control_[2]);
  Eigen::Vector2d tau1 = J1.transpose() * -f1 * scaler;
  computed_efforts[0] = tau1[0];
  computed_efforts[1] = tau1[1];

  Eigen::Vector3d f2(optimal_control_[3], optimal_control_[4], optimal_control_[5]);
  Eigen::Vector2d tau2 = J2.transpose() * -f2 * scaler;
  computed_efforts[2] = tau2[0];
  computed_efforts[3] = tau2[1];
  
  Eigen::Vector3d f3(optimal_control_[6], optimal_control_[7], optimal_control_[8]);
  Eigen::Vector2d tau3 = J3.transpose() * -f3 * scaler;
  computed_efforts[4] = tau3[0];
  computed_efforts[5] = tau3[1];
  
  Eigen::Vector3d f4(optimal_control_[9], optimal_control_[10], optimal_control_[11]);
  Eigen::Vector2d tau4 = J4.transpose() * -f4 * scaler;
  computed_efforts[6] = tau4[0];
  computed_efforts[7] = tau4[1];

  // Set computed efforts to command interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto result = command_interfaces_[i].set_value(computed_efforts[i]);
    if (!result) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set command for joint %zu", i);
    }
  }
}

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_
