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

    // Update gait pattern and foot states
    {
      std::lock_guard<std::mutex> lock(gait_mutex_);
      if (!latest_gait_) {
        if (update_count % 100 == 0) {
          RCLCPP_INFO(get_node()->get_logger(), "Waiting for gait pattern data...");
        }
        return false;
      }

      // Get foot states from gait pattern
      foot1_state_ = latest_gait_->foot1_state;
      foot2_state_ = latest_gait_->foot2_state;
      foot3_state_ = latest_gait_->foot3_state;
      foot4_state_ = latest_gait_->foot4_state;

      // Update contact flags in state vector
      current_state_[0] -= latest_gait_->com_position.x;
      current_state_[1] -= latest_gait_->com_position.y;
      
      RCLCPP_DEBUG(get_node()->get_logger(), "Foot states: %d, %d, %d, %d", 
          foot1_state_, foot2_state_, foot3_state_, foot4_state_);
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
    // Set reference trajectory for all prediction steps
    for (int stage = 0; stage <= solver_->nlp_solver_plan->N; stage++) {
      ocp_nlp_cost_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, stage, "yref", desired_state_.data());
    }

    // Set state constraints
    ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "lbx", current_state_.data());
    ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "ubx", current_state_.data());
    
    // Calculate robot mass parameters
    double robot_mass = 15.0; // kg - replace with actual robot mass parameter
    double gravity = 9.81; // m/s²
    
    // Create force constraint vectors
    std::vector<double> min_force(12, 0.0);
    std::vector<double> max_force(12, 0.0);
    
    // Default force constraints - lateral friction constraints and max vertical force
    double friction_coef = 0.2; // friction coefficient
    double max_vertical_force = robot_mass * gravity; // Maximum force robot can apply
    
    // Set foot-specific constraints based on state
    // Foot 1 (indices 0,1,2 for x,y,z forces)
    if (foot1_state_ == 1) { // Swing - zero force in ALL directions (very small value to make solution feasible)
        min_force[0] = 0.01;
        max_force[0] = 0.01;
        min_force[1] = 0.01;
        max_force[1] = 0.01;
        min_force[2] = 0.01;
        max_force[2] = 0.01;
        RCLCPP_DEBUG(get_node()->get_logger(), "Foot 1 in swing - zero force in all directions");
    } else { // Stance - apply friction cone constraints
        min_force[0] = -max_vertical_force/4 * friction_coef;  // x min
        max_force[0] =  max_vertical_force/4 * friction_coef;  // x max
        min_force[1] = -max_vertical_force/4 * friction_coef;  // y min
        max_force[1] =  max_vertical_force/4 * friction_coef;  // y max
        min_force[2] = 0.0;                                   // z min (non-negative)
        max_force[2] = max_vertical_force;                     // z max
        RCLCPP_DEBUG(get_node()->get_logger(), "Foot 1 in stance - applying friction cone constraint");
    }
    
    // Foot 2 (indices 3,4,5 for x,y,z forces)
    if (foot2_state_ == 1) { // Swing - zero force in ALL directions (very small value to make solution feasible)
        min_force[3] = 0.01;
        max_force[3] = 0.011;
        min_force[4] = 0.01;
        max_force[4] = 0.01;
        min_force[5] = 0.01;
        max_force[5] = 0.01;
        RCLCPP_DEBUG(get_node()->get_logger(), "Foot 2 in swing - zero force in all directions");
    } else { // Stance
        min_force[3] = -max_vertical_force/4 * friction_coef;
        max_force[3] =  max_vertical_force/4 * friction_coef;
        min_force[4] = -max_vertical_force/4 * friction_coef;
        max_force[4] =  max_vertical_force/4 * friction_coef;
        min_force[5] = 0.0;
        max_force[5] = max_vertical_force;
        RCLCPP_DEBUG(get_node()->get_logger(), "Foot 2 in stance - applying friction cone constraint");
    }
    
    // Foot 3 (indices 6,7,8 for x,y,z forces)
    if (foot3_state_ == 1) { // Swing - zero force in ALL directions (very small value to make solution feasible)
        min_force[6] = 0.01;
        max_force[6] = 0.01;
        min_force[7] = 0.01;
        max_force[7] = 0.01;
        min_force[8] = 0.01;
        max_force[8] = 0.01;
        RCLCPP_DEBUG(get_node()->get_logger(), "Foot 3 in swing - zero force in all directions");
    } else { // Stance
        min_force[6] = -max_vertical_force/4 * friction_coef;
        max_force[6] =  max_vertical_force/4 * friction_coef;
        min_force[7] = -max_vertical_force/4 * friction_coef;
        max_force[7] =  max_vertical_force/4 * friction_coef;
        min_force[8] = 0.0;
        max_force[8] = max_vertical_force;
        RCLCPP_DEBUG(get_node()->get_logger(), "Foot 3 in stance - applying friction cone constraint");
    }
    
    // Foot 4 (indices 9,10,11 for x,y,z forces)
    if (foot4_state_ == 1) { // Swing - zero force in ALL directions (very small value to make solution feasible)
        min_force[9] = 0.01;
        max_force[9] = 0.01;
        min_force[10] = 0.01;
        max_force[10] = 0.01;
        min_force[11] = 0.01;
        max_force[11] = 0.01;
        RCLCPP_DEBUG(get_node()->get_logger(), "Foot 4 in swing - zero force in all directions");
    } else { // Stance
        min_force[9] = -max_vertical_force/4 * friction_coef;
        max_force[9] =  max_vertical_force/4 * friction_coef;
        min_force[10] = -max_vertical_force/4 * friction_coef;
        max_force[10] =  max_vertical_force/4 * friction_coef;
        min_force[11] = 0.0;
        max_force[11] = max_vertical_force;
        RCLCPP_DEBUG(get_node()->get_logger(), "Foot 4 in stance - applying friction cone constraint");
    }
    
    // Apply constraints to the ACADOS solver
    ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "lbu", min_force.data());
    ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "ubu", max_force.data());
    
    // Solve the optimal control problem
    int solve_status = quadruped_ode_acados_solve(solver_);
    if (solve_status != 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "ACADOS solver failed with status %d", solve_status);
      return false;
    }

    // Get the optimal control solution
    ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "u", optimal_control_.data());
    
    // Changed to DEBUG level to reduce terminal clutter
    RCLCPP_DEBUG(get_node()->get_logger(), 
      "Foot Forces (Optimal / Min / Max):");
    RCLCPP_DEBUG(get_node()->get_logger(), 
      "Foot1 [%s]: X: %6.2f / %6.2f / %6.2f | Y: %6.2f / %6.2f / %6.2f | Z: %6.2f / %6.2f / %6.2f",
      (foot1_state_ == 1) ? "SWING" : "STANCE",
      optimal_control_[0], min_force[0], max_force[0],
      optimal_control_[1], min_force[1], max_force[1],
      optimal_control_[2], min_force[2], max_force[2]);
    RCLCPP_DEBUG(get_node()->get_logger(), 
      "Foot2 [%s]: X: %6.2f / %6.2f / %6.2f | Y: %6.2f / %6.2f / %6.2f | Z: %6.2f / %6.2f / %6.2f",
      (foot2_state_ == 1) ? "SWING" : "STANCE",
      optimal_control_[3], min_force[3], max_force[3],
      optimal_control_[4], min_force[4], max_force[4],
      optimal_control_[5], min_force[5], max_force[5]);
    RCLCPP_DEBUG(get_node()->get_logger(), 
      "Foot3 [%s]: X: %6.2f / %6.2f / %6.2f | Y: %6.2f / %6.2f / %6.2f | Z: %6.2f / %6.2f / %6.2f",
      (foot3_state_ == 1) ? "SWING" : "STANCE",
      optimal_control_[6], min_force[6], max_force[6],
      optimal_control_[7], min_force[7], max_force[7],
      optimal_control_[8], min_force[8], max_force[8]);
    RCLCPP_DEBUG(get_node()->get_logger(), 
      "Foot4 [%s]: X: %6.2f / %6.2f / %6.2f | Y: %6.2f / %6.2f / %6.2f | Z: %6.2f / %6.2f / %6.2f",
      (foot4_state_ == 1) ? "SWING" : "STANCE",
      optimal_control_[9], min_force[9], max_force[9],
      optimal_control_[10], min_force[10], max_force[10],
      optimal_control_[11], min_force[11], max_force[11]);

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
      
      // Changed to DEBUG level to reduce terminal clutter
      RCLCPP_DEBUG(get_node()->get_logger(), 
      "Foot1: [%f, %f, %f], Foot2: [%f, %f, %f], Foot3: [%f, %f, %f], Foot4: [%f, %f, %f]",
      msg.foot1_force.x, msg.foot1_force.y, msg.foot1_force.z,
      msg.foot2_force.x, msg.foot2_force.y, msg.foot2_force.z,
      msg.foot3_force.x, msg.foot3_force.y, msg.foot3_force.z,
      msg.foot4_force.x, msg.foot4_force.y, msg.foot4_force.z);
      
      foot_forces_publisher_->unlockAndPublish();
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in update_commands: %s", e.what());
    return false;
  }
}

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_
