#ifndef QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_
#define QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_

#include "quadruped_mpc/controller_interfaces/balance_controller.hpp"
#include "quadruped_mpc/utilities/shared_quadruped_info.hpp"

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

  // Read from shared quadruped info
  auto& info = SharedQuadrupedInfo::getInstance();
  std::lock_guard<std::mutex> lock(info.mutex_);

  if (!info.is_initialized_) {
    if (update_count % 100 == 0) {
      RCLCPP_INFO(get_node()->get_logger(), "Waiting for initialization...");
    }
    throw std::runtime_error("SharedQuadrupedInfo not initialized");
  }

  // Update foot positions for the solver
  p1_ = {info.state_.p1[0], info.state_.p1[1], info.state_.p1[2]};
  p2_ = {info.state_.p2[0], info.state_.p2[1], info.state_.p2[2]};
  p3_ = {info.state_.p3[0], info.state_.p3[1], info.state_.p3[2]};
  p4_ = {info.state_.p4[0], info.state_.p4[1], info.state_.p4[2]};
  com_ = {info.state_.pc[0], info.state_.pc[1], info.state_.pc[2]};

  // Fill current state vector
  current_state_[0] = info.state_.pc[0];
  current_state_[1] = info.state_.pc[1];
  current_state_[2] = info.state_.pc[2];

  Eigen::Quaterniond q(
      info.state_.orientation_quat[3],
      info.state_.orientation_quat[0],
      info.state_.orientation_quat[1],
      info.state_.orientation_quat[2]
  );
  
  current_state_[3] = q.coeffs()[3];
  current_state_[4] = q.coeffs()[0];
  current_state_[5] = q.coeffs()[1];
  current_state_[6] = q.coeffs()[2];

  current_state_[7] = info.state_.vc[0];
  current_state_[8] = info.state_.vc[1];
  current_state_[9] = info.state_.vc[2];

  current_state_[10] = info.state_.angular_velocity[0];
  current_state_[11] = info.state_.angular_velocity[1];
  current_state_[12] = info.state_.angular_velocity[2];

  for (int i = 0; i < 3; i++) {
    current_state_[13 + i] = info.state_.p1[i];
    current_state_[16 + i] = info.state_.p2[i];
    current_state_[19 + i] = info.state_.p3[i];
    current_state_[22 + i] = info.state_.p4[i];
  }

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
  auto& info = SharedQuadrupedInfo::getInstance();
  std::array<double, 8> computed_efforts{};
  double scaler = 1;

  Eigen::Vector3d f1(optimal_control_[0], optimal_control_[1], optimal_control_[2]);
  Eigen::Vector2d tau1 = info.state_.J1.transpose() * -f1 * scaler;
  computed_efforts[0] = tau1[0];
  computed_efforts[1] = tau1[1];

  Eigen::Vector3d f2(optimal_control_[3], optimal_control_[4], optimal_control_[5]);
  Eigen::Vector2d tau2 = info.state_.J2.transpose() * -f2 * scaler;
  computed_efforts[2] = tau2[0];
  computed_efforts[3] = tau2[1];
  
  Eigen::Vector3d f3(optimal_control_[6], optimal_control_[7], optimal_control_[8]);
  Eigen::Vector2d tau3 = info.state_.J3.transpose() * -f3 * scaler;
  computed_efforts[4] = tau3[0];
  computed_efforts[5] = tau3[1];
  
  Eigen::Vector3d f4(optimal_control_[9], optimal_control_[10], optimal_control_[11]);
  Eigen::Vector2d tau4 = info.state_.J4.transpose() * -f4 * scaler;
  computed_efforts[6] = tau4[0];
  computed_efforts[7] = tau4[1];

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto result = command_interfaces_[i].set_value(computed_efforts[i]);
    if (!result) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set command for joint %zu", i);
    }
  }
}

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_
