#ifndef QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_
#define QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_

#include "quadruped_mpc/controller_interfaces/balance_controller.hpp"
#include <Eigen/Dense>  // Add this include for Eigen matrices
#include <iomanip>      // Add this include for std::setw, std::setprecision

namespace quadruped_mpc
{

inline void BalanceController::print_state_vector_table()
{
  std::stringstream ss;
  
  // Get number of stages in the prediction horizon
  int prediction_horizon = solver_->nlp_solver_plan->N;
  std::vector<std::array<double, 25>> stage_states(prediction_horizon + 1);
  
  // Extract state vectors for each stage
  for (int stage = 0; stage <= prediction_horizon; stage++) {
    std::array<double, 25> stage_state;
    ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, stage, "x", stage_state.data());
    stage_states[stage] = stage_state;
  }
  
  // Print header line
  ss << "╔═════╦══════════════════════════╦══════════════════════════════════╦══════════════════════════╦══════════════════════════╗\n";
  ss << "║STAGE║     POSITION (XYZ)       ║     ORIENTATION (WXYZ)           ║    LINEAR VEL (XYZ)      ║    ANGULAR VEL (XYZ)     ║\n";
  ss << "╠═════╬══════════════════════════╬══════════════════════════════════╬══════════════════════════╬══════════════════════════╣\n";
  
  // Print rows for each stage
  for (int stage = 0; stage <= std::min(5, prediction_horizon); stage++) {
    ss << "║ " << std::setw(3) << stage << " ║ [";
    // Position
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][0] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][1] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][2] << "] ║ [";
    // Orientation (quaternion)
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][3] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][4] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][5] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][6] << "] ║ [";
    // Linear velocity
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][7] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][8] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][9] << "] ║ [";
    // Angular velocity
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][10] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][11] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[stage][12] << "] ║\n";
  }
  
  // If there are more stages, show an indication
  if (prediction_horizon > 5) {
    ss << "║ ... ║            ...           ║               ...                ║           ...            ║           ...            ║\n";
    
    // Show the last stage as well
    int last = prediction_horizon;
    ss << "║ " << std::setw(3) << last << " ║ [";
    // Position
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][0] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][1] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][2] << "] ║ [";
    // Orientation (quaternion)
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][3] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][4] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][5] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][6] << "] ║ [";
    // Linear velocity
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][7] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][8] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][9] << "] ║ [";
    // Angular velocity
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][10] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][11] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(3) << stage_states[last][12] << "] ║\n";
  }
  
  // Add a separator line before desired state
  ss << "╠═════╩══════════════════════════╩══════════════════════════════════╩══════════════════════════╩══════════════════════════╣\n";
  
  // Add the desired state row
  ss << "║ DESIRED STATE                                                                                                           ║\n";
  ss << "╠═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╣\n";
  
  // Position, orientation, linear and angular velocity
  ss << "║ Position: [";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[0] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[1] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[2] << "]  ";
  
  ss << "Orientation: [";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[3] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[4] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[5] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[6] << "]                                       ║\n";
  
  ss << "║ Lin Vel: [";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[7] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[8] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[9] << "]  ";
  
  ss << "Ang Vel: [";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[10] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[11] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << desired_state_[12] << "]                                                    ║\n";
  
  // Print footer line
  ss << "╚═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝\n\n";
  
  // Log the table
  RCLCPP_INFO(get_node()->get_logger(), "State Vector Prediction:\n%s", ss.str().c_str());
}

inline void BalanceController::print_controller_output_table()
{
  std::stringstream ss;
  
  // Get number of stages in the prediction horizon
  int prediction_horizon = solver_->nlp_solver_plan->N;
  std::vector<std::array<double, 12>> stage_controls(prediction_horizon + 1);
  
  // Extract optimal control for each stage
  for (int stage = 0; stage < prediction_horizon; stage++) {
    std::array<double, 12> stage_control;
    ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, stage, "u", stage_control.data());
    
    // Negate values as the solver returns ground reaction forces
    for (int i = 0; i < 12; i++) {
      stage_control[i] = -stage_control[i];
    }
    
    stage_controls[stage] = stage_control;
  }
  
  // Use the optimal_control_ for the first stage which is already populated
  stage_controls[0] = {
    optimal_control_[0], optimal_control_[1], optimal_control_[2],
    optimal_control_[3], optimal_control_[4], optimal_control_[5],
    optimal_control_[6], optimal_control_[7], optimal_control_[8],
    optimal_control_[9], optimal_control_[10], optimal_control_[11]
  };
  
  // Print header line
  ss << "╔═════╦══════════════════════════╦══════════════════════════╦══════════════════════════╦══════════════════════════╦═══════════════════════════╗\n";
  ss << "║STAGE║          FOOT 1          ║          FOOT 2          ║          FOOT 3          ║          FOOT 4          ║       TOTAL FORCE         ║\n";
  ss << "╠═════╬══════════════════════════╬══════════════════════════╬══════════════════════════╬══════════════════════════╬═══════════════════════════╣\n";
  
  // Print rows for each stage
  for (int stage = 0; stage < std::min(5, prediction_horizon); stage++) {  // Limit to first 5 stages for readability
    // Calculate total force for this stage
    double total_x = stage_controls[stage][0] + stage_controls[stage][3] + stage_controls[stage][6] + stage_controls[stage][9];
    double total_y = stage_controls[stage][1] + stage_controls[stage][4] + stage_controls[stage][7] + stage_controls[stage][10];
    double total_z = stage_controls[stage][2] + stage_controls[stage][5] + stage_controls[stage][8] + stage_controls[stage][11];
    
    ss << "║ " << std::setw(3) << stage << " ║ [";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[stage][0] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[stage][1] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[stage][2] << "] ║ [";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[stage][3] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[stage][4] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[stage][5] << "] ║ [";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[stage][6] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[stage][7] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[stage][8] << "] ║ [";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[stage][9] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[stage][10] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[stage][11] << "] ║ [";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << total_x << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << total_y << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << total_z << "] ║\n";
  }
  
  // If there are more stages, show an indication
  if (prediction_horizon > 5) {
    ss << "║ ... ║           ...            ║           ...            ║           ...            ║           ...            ║           ...             ║\n";
    
    // Show the last stage as well
    int last = prediction_horizon - 1;
    // Calculate total force for the last stage
    double last_total_x = stage_controls[last][0] + stage_controls[last][3] + stage_controls[last][6] + stage_controls[last][9];
    double last_total_y = stage_controls[last][1] + stage_controls[last][4] + stage_controls[last][7] + stage_controls[last][10];
    double last_total_z = stage_controls[last][2] + stage_controls[last][5] + stage_controls[last][8] + stage_controls[last][11];
    
    ss << "║ " << std::setw(3) << last << " ║ [";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[last][0] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[last][1] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[last][2] << "] ║ [";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[last][3] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[last][4] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[last][5] << "] ║ [";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[last][6] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[last][7] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[last][8] << "] ║ [";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[last][9] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[last][10] << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << stage_controls[last][11] << "] ║ [";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << last_total_x << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << last_total_y << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << last_total_z << "] ║\n";
  }
  
  // Print total force row - sum the forces from all feet in the first stage
  double total_x = stage_controls[0][0] + stage_controls[0][3] + stage_controls[0][6] + stage_controls[0][9];
  double total_y = stage_controls[0][1] + stage_controls[0][4] + stage_controls[0][7] + stage_controls[0][10];
  double total_z = stage_controls[0][2] + stage_controls[0][5] + stage_controls[0][8] + stage_controls[0][11];
  
  // Add a separator line before the totals
  ss << "╠═════╩══════════════════════════╩══════════════════════════╩══════════════════════════╩══════════════════════════╬═══════════════════════════╣\n";
  
  // Add the totals row for forces
  ss << "║ TOTAL FORCE (4 feet)                                                                                            ║ [";
  ss << std::setw(6) << std::fixed << std::setprecision(2) << total_x << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(2) << total_y << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(2) << total_z << "] ║\n";
  
  // Add a row for foot positions (only for current state)
  ss << "╠═════════════════════════════════════════════════════════════════════════════════════════════════════════════════╩═══════════════════════════╣\n";
  ss << "║ FOOT POSITIONS [" << (foot1_state_ == 1 ? "SWING" : "STANCE") << "|" << (foot2_state_ == 1 ? "SWING" : "STANCE") << "|" << (foot3_state_ == 1 ? "SWING" : "STANCE") << "|" << (foot4_state_ == 1 ? "SWING" : "STANCE") << "]                                                                                                ║\n";
  
  // Add lines for each foot position
  ss << "║ Foot1: [";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << current_state_[13] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << current_state_[14] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << current_state_[15] << "]  ";
  
  ss << "Foot2: [";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << current_state_[16] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << current_state_[17] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << current_state_[18] << "]  ";
  
  ss << "Foot3: [";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << current_state_[19] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << current_state_[20] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << current_state_[21] << "]  ";
  
  ss << "Foot4: [";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << current_state_[22] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << current_state_[23] << ", ";
  ss << std::setw(6) << std::fixed << std::setprecision(3) << current_state_[24] << "]          ║\n";
  
  // Print footer line
  ss << "╚═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝";
  
  // Log the table
  RCLCPP_INFO(get_node()->get_logger(), "Balance Controller Output (Foot Forces and Positions):\n%s", ss.str().c_str());
}

inline bool BalanceController::update_state()
{
  try {
    //RCLCPP_INFO(get_node()->get_logger(), "\n\n\n -------------------------------------------------- NEW STATE --------------------------------------------------\n\n");
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
    
    // Calculate robot mass parameters
    double robot_mass = 15.0; // kg - replace with actual robot mass parameter
    double gravity = 9.81; // m/s²
    
    // Create force constraint vectors
    std::vector<double> min_force(12, 0.0);
    std::vector<double> max_force(12, 0.0);
    
    // Default force constraints - lateral friction constraints and max vertical force
    double friction_coef = 0.2; // friction coefficient
    double max_vertical_force = robot_mass * gravity; // Maximum force robot can apply
    
    // Count the number of stance feet to distribute weight
    int num_stance_feet = 0;
    if (foot1_state_ == 0) num_stance_feet++;
    if (foot2_state_ == 0) num_stance_feet++;
    if (foot3_state_ == 0) num_stance_feet++;
    if (foot4_state_ == 0) num_stance_feet++;
    
    // Safety check to avoid division by zero
    if (num_stance_feet == 0) num_stance_feet = 1;
    
    // Calculate the vertical force per stance foot to support the robot's weight
    double vertical_force_per_foot = robot_mass * gravity / num_stance_feet;
    
    // Initialize control initial guess vector - will be populated based on foot state
    std::vector<double> initial_guess(12, 0.0);
    
    // Set foot-specific constraints based on current state
    // Foot 1 (indices 0,1,2 for x,y,z forces)
    if (foot1_state_ == 1) { // Swing - zero force in ALL directions
        min_force[0] = -0.01;
        max_force[0] = 0.01;
        min_force[1] = -0.01;
        max_force[1] = 0.01;
        min_force[2] = -0.01;
        max_force[2] = 0.01;
        // Initial guess for swing foot - zero force
        initial_guess[0] = 0.0;
        initial_guess[1] = 0.0;
        initial_guess[2] = 0.0;
    } else { // Stance - apply friction cone constraints
        min_force[0] = -max_vertical_force/4 * friction_coef;  // x min
        max_force[0] =  max_vertical_force/4 * friction_coef;  // x max
        min_force[1] = -max_vertical_force/4 * friction_coef;  // y min
        max_force[1] =  max_vertical_force/4 * friction_coef;  // y max
        min_force[2] = 0.0;                                    // z min (non-negative)
        max_force[2] = max_vertical_force;                     // z max
        // Initial guess for stance foot - share weight
        initial_guess[0] = 0.0; // no lateral force initially
        initial_guess[1] = 0.0; // no lateral force initially
        initial_guess[2] = vertical_force_per_foot; // negative because GRF is opposite to joint force
    }
    
    // Foot 2 (indices 3,4,5 for x,y,z forces)
    if (foot2_state_ == 1) { // Swing - zero force in ALL directions
        min_force[3] = -0.01;
        max_force[3] = 0.01;
        min_force[4] = -0.01;
        max_force[4] = 0.01;
        min_force[5] = -0.01;
        max_force[5] = 0.01;
        // Initial guess for swing foot - zero force
        initial_guess[3] = 0.0;
        initial_guess[4] = 0.0;
        initial_guess[5] = 0.0;
    } else { // Stance
        min_force[3] = -max_vertical_force/4 * friction_coef;
        max_force[3] =  max_vertical_force/4 * friction_coef;
        min_force[4] = -max_vertical_force/4 * friction_coef;
        max_force[4] =  max_vertical_force/4 * friction_coef;
        min_force[5] = 0.0;
        max_force[5] = max_vertical_force;
        // Initial guess for stance foot - share weight
        initial_guess[3] = 0.0; // no lateral force initially
        initial_guess[4] = 0.0; // no lateral force initially
        initial_guess[5] = vertical_force_per_foot; // negative because GRF is opposite to joint force
    }
    
    // Foot 3 (indices 6,7,8 for x,y,z forces)
    if (foot3_state_ == 1) { // Swing - zero force in ALL directions
        min_force[6] = -0.01;
        max_force[6] = 0.01;
        min_force[7] = -0.01;
        max_force[7] = 0.01;
        min_force[8] = -0.01;
        max_force[8] = 0.01;
        // Initial guess for swing foot - zero force
        initial_guess[6] = 0.0;
        initial_guess[7] = 0.0;
        initial_guess[8] = 0.0;
    } else { // Stance
        min_force[6] = -max_vertical_force/4 * friction_coef;
        max_force[6] =  max_vertical_force/4 * friction_coef;
        min_force[7] = -max_vertical_force/4 * friction_coef;
        max_force[7] =  max_vertical_force/4 * friction_coef;
        min_force[8] = 0.0;
        max_force[8] = max_vertical_force;
        // Initial guess for stance foot - share weight
        initial_guess[6] = 0.0; // no lateral force initially
        initial_guess[7] = 0.0; // no lateral force initially
        initial_guess[8] = vertical_force_per_foot; // negative because GRF is opposite to joint force
    }
    
    // Foot 4 (indices 9,10,11 for x,y,z forces)
    if (foot4_state_ == 1) { // Swing - zero force in ALL directions
        min_force[9] = -0.01;
        max_force[9] = 0.01;
        min_force[10] = -0.01;
        max_force[10] = 0.01;
        min_force[11] = -0.01;
        max_force[11] = 0.01;
        // Initial guess for swing foot - zero force
        initial_guess[9] = 0.0;
        initial_guess[10] = 0.0;
        initial_guess[11] = 0.0;
    } else { // Stance
        min_force[9] = -max_vertical_force/4 * friction_coef;
        max_force[9] =  max_vertical_force/4 * friction_coef;
        min_force[10] = -max_vertical_force/4 * friction_coef;
        max_force[10] =  max_vertical_force/4 * friction_coef;
        min_force[11] = 0.0;
        max_force[11] = max_vertical_force;
        // Initial guess for stance foot - share weight
        initial_guess[9] = 0.0; // no lateral force initially
        initial_guess[10] = 0.0; // no lateral force initially
        initial_guess[11] = vertical_force_per_foot; // negative because GRF is opposite to joint force
    }

    std::vector<double> lowest_state(12, 0.0);
    std::vector<double> highest_state(12, 0.0);

    // Set position constraints with reasonable bounds
    lowest_state[0] = -0.75;  // Set reasonable x and y bounds - the robot's body can't extend beyond the feet
    lowest_state[1] = -0.75;   
    lowest_state[2] = 0.05;    // Set z bounds - don't let the body get too close to the ground
    
    // Use non-constraining bounds for orientation and velocities
    lowest_state[3] = -1.0;    // Quaternion w (actual range is -1 to 1)
    lowest_state[4] = -1.0;    // Quaternion x
    lowest_state[5] = -1.0;    // Quaternion y
    lowest_state[6] = -1.0;    // Quaternion z
    
    // Set very loose bounds on velocities
    lowest_state[7] = -10.0;   // Linear velocity x
    lowest_state[8] = -10.0;   // Linear velocity y
    lowest_state[9] = -10.0;   // Linear velocity z
    lowest_state[10] = -10.0;  // Angular velocity x
    lowest_state[11] = -10.0;  // Angular velocity y
    lowest_state[12] = -10.0;  // Angular velocity z

    // Set position constraints with reasonable bounds
    highest_state[0] = 0.75;   // Upper bound for x position
    highest_state[1] = 0.75;   // Upper bound for y position
    highest_state[2] = 0.25;    // Upper bound for z position - don't let the body go too high
    
    // Use non-constraining bounds for orientation
    highest_state[3] = 1.0;    // Quaternion w (actual range is -1 to 1)
    highest_state[4] = 1.0;    // Quaternion x
    highest_state[5] = 1.0;    // Quaternion y
    highest_state[6] = 1.0;    // Quaternion z
    
    // Set very loose bounds on velocities
    highest_state[7] = 10.0;   // Linear velocity x
    highest_state[8] = 10.0;   // Linear velocity y
    highest_state[9] = 10.0;   // Linear velocity z
    highest_state[10] = 10.0;  // Angular velocity x
    highest_state[11] = 10.0;  // Angular velocity y
    highest_state[12] = 10.0;  // Angular velocity z

    // Apply constraints to ALL stages in the optimization horizon
    int horizon_length = solver_->nlp_solver_plan->N;
    for (int stage = 0; stage < horizon_length; stage++) {
        // Apply force constraints to the current stage in the ACADOS solver
        ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 
                                     stage, "lbu", min_force.data());
        ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 
                                     stage, "ubu", max_force.data());
        // Set the height constraints
        ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 
                                     stage, "lbx", lowest_state.data());
        ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 
                                     stage, "ubx", highest_state.data());
        // Set the initial guess for the control input
        //ocp_nlp_out_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 
        //                             stage, "u", initial_guess.data());   
    }

    // Set initial state constraints
    ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "lbx", current_state_.data());
    ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "ubx", current_state_.data());
    ocp_nlp_out_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "x", current_state_.data());
    
    // Solve the optimal control problem
    int solve_status = quadruped_ode_acados_solve(solver_);
    if (solve_status != 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "ACADOS solver failed with status %d", solve_status);
      return false;
    }

    // Get the optimal control solution
    ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "u", optimal_control_.data());
    
    // Negate the optimal control solution - the solver returns ground reaction forces,
    // but the legs need to exert the opposite forces
    for (int i = 0; i < optimal_control_.size(); i++) {
      optimal_control_[i] = -optimal_control_[i];
    }
    
    // Print nicely formatted tables with controller output and state vectors
    //print_controller_output_table();
    //print_state_vector_table();
    
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
