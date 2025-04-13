#ifndef QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_
#define QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_

#include "quadruped_mpc/controller_interfaces/balance_controller.hpp"
#include <Eigen/Dense>  // Add this include for Eigen matrices

namespace quadruped_mpc
{

inline bool BalanceController::update_state()
{
  try {
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
        return false;
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

    // Update gait pattern
    {
      std::lock_guard<std::mutex> lock(gait_mutex_);
      if (!latest_gait_) {
        if (update_count % 100 == 0) {
          RCLCPP_INFO(get_node()->get_logger(), "Waiting for gait pattern data...");
        }
        return false;
      }

      // Update contact flags in state vector
      current_state_[0] -= latest_gait_->com_position.x;
      current_state_[1] -= latest_gait_->com_position.y;
    }

    // Handle desired state updates from commands - updated for separate pose and twist
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      
      // Handle pose commands
      if (new_pose_cmd_received_ && latest_pose_cmd_) {
        desired_state_[0] = latest_pose_cmd_->position.x;
        desired_state_[1] = latest_pose_cmd_->position.y;
        desired_state_[2] = latest_pose_cmd_->position.z;
        desired_state_[3] = latest_pose_cmd_->orientation.w;
        desired_state_[4] = latest_pose_cmd_->orientation.x;
        desired_state_[5] = latest_pose_cmd_->orientation.y;
        desired_state_[6] = latest_pose_cmd_->orientation.z;
        new_pose_cmd_received_ = false;
      }
      
      // Handle twist commands
      if (new_twist_cmd_received_ && latest_twist_cmd_) {
        // Update linear velocity setpoints
        desired_state_[7] = latest_twist_cmd_->linear.x;
        desired_state_[8] = latest_twist_cmd_->linear.y;
        desired_state_[9] = latest_twist_cmd_->linear.z;
        
        // Update angular velocity setpoints
        desired_state_[10] = latest_twist_cmd_->angular.x;
        desired_state_[11] = latest_twist_cmd_->angular.y;
        desired_state_[12] = latest_twist_cmd_->angular.z;
        new_twist_cmd_received_ = false;
      }
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in update_state: %s", e.what());
    return false;
  }
}

inline bool BalanceController::update_control()
{
  try {
    for (int stage = 0; stage <= solver_->nlp_solver_plan->N; stage++) {
      ocp_nlp_cost_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, stage, "yref", desired_state_.data());
    }

    ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "lbx", current_state_.data());
    ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "ubx", current_state_.data());
    
    int solve_status = quadruped_ode_acados_solve(solver_);
    if (solve_status != 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "ACADOS solver failed with status %d", solve_status);
      return false;
    }

    ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "u", optimal_control_.data());
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in update_control: %s", e.what());
    return false;
  }
}

inline bool BalanceController::update_commands()
{
  try {
    // Publish foot forces - this is now the main function of update_commands
    if (foot_forces_publisher_ && foot_forces_publisher_->trylock()) {
      auto& msg = foot_forces_publisher_->msg_;
      
      // Set the foot forces from optimal control
      msg.foot1_force.x = optimal_control_[0];
      msg.foot1_force.y = optimal_control_[1];
      msg.foot1_force.z = optimal_control_[2];
      
      msg.foot2_force.x = optimal_control_[3];
      msg.foot2_force.y = optimal_control_[4];
      msg.foot2_force.z = optimal_control_[5];
      
      msg.foot3_force.x = optimal_control_[6];
      msg.foot3_force.y = optimal_control_[7];
      msg.foot3_force.z = optimal_control_[8];
      
      msg.foot4_force.x = optimal_control_[9];
      msg.foot4_force.y = optimal_control_[10];
      msg.foot4_force.z = optimal_control_[11];
      
      foot_forces_publisher_->unlockAndPublish();
      RCLCPP_DEBUG(get_node()->get_logger(), "Published foot forces");
    }

    // No longer calculate or send joint torques directly
    // The foot_controller will do this based on the published foot forces
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in update_commands: %s", e.what());
    return false;
  }
}

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_
