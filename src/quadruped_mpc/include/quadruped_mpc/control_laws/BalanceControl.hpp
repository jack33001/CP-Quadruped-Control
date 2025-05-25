#ifndef QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_
#define QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_

#include "quadruped_mpc/controller_interfaces/balance_controller.hpp"
#include <Eigen/Dense>
#include <iomanip>

namespace quadruped_mpc
{

  inline void BalanceController::print_state_vector_table()
  {
    std::stringstream ss;

    // Get number of stages in the prediction horizon
    int prediction_horizon = solver_->nlp_solver_plan->N;
    std::vector<std::array<double, 25>> stage_states(prediction_horizon + 1);

    // Extract state vectors for each stage
    for (int stage = 0; stage <= prediction_horizon; stage++)
    {
      std::array<double, 25> stage_state;
      ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, stage, "x", stage_state.data());
      stage_states[stage] = stage_state;
    }

    // Print header line
    ss << "╔═════╦══════════════════════════╦══════════════════════════════════╦══════════════════════════╦══════════════════════════╗\n";
    ss << "║STAGE║     POSITION (XYZ)       ║     ORIENTATION (WXYZ)           ║    LINEAR VEL (XYZ)      ║    ANGULAR VEL (XYZ)     ║\n";
    ss << "╠═════╬══════════════════════════╬══════════════════════════════════╬══════════════════════════╬══════════════════════════╣\n";

    // Print rows for each stage
    for (int stage = 0; stage <= std::min(5, prediction_horizon); stage++)
    {
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
    if (prediction_horizon > 5)
    {
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
    for (int stage = 0; stage < prediction_horizon; stage++)
    {
      std::array<double, 12> stage_control;
      ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, stage, "u", stage_control.data());

      // Negate values as the solver returns ground reaction forces
      for (int i = 0; i < 12; i++)
      {
        stage_control[i] = -stage_control[i];
      }

      stage_controls[stage] = stage_control;
    }

    // Use the optimal_control_ for the first stage which is already populated
    stage_controls[0] = {
        optimal_control_[0], optimal_control_[1], optimal_control_[2],
        optimal_control_[3], optimal_control_[4], optimal_control_[5],
        optimal_control_[6], optimal_control_[7], optimal_control_[8],
        optimal_control_[9], optimal_control_[10], optimal_control_[11]};

    // Print header line
    ss << "╔═════╦══════════════════════════╦══════════════════════════╦══════════════════════════╦══════════════════════════╦═══════════════════════════╗\n";
    ss << "║STAGE║          FOOT 1          ║          FOOT 2          ║          FOOT 3          ║          FOOT 4          ║       TOTAL FORCE         ║\n";
    ss << "╠═════╬══════════════════════════╬══════════════════════════╬══════════════════════════╬══════════════════════════╬═══════════════════════════╣\n";

    // Print rows for each stage
    for (int stage = 0; stage < std::min(5, prediction_horizon); stage++)
    {
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
    if (prediction_horizon > 5)
    {
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
    ss << "╠═════╩══════════════════════════╩══════════════════════════╦══════════════════════════╦══════════════════════════╬═══════════════════════════╣\n";

    // Add the totals row for forces
    ss << "║ TOTAL FORCE (4 feet)                                                                                            ║ [";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << total_x << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << total_y << ", ";
    ss << std::setw(6) << std::fixed << std::setprecision(2) << total_z << "] ║\n";

    // Add a row for foot positions (only for current state)
    ss << "╠═════════════════════════════════════════════════════════════════════════════════════════════════════════════════╩═══════════════════════════╣\n";

    // Use the new foot_states_ array for displaying foot states
    ss << "║ FOOT POSITIONS [";
    for (int i = 0; i < 4; i++)
    {
      ss << (foot_states_[i] == 1 ? "SWING" : "STANCE");
      if (i < 3)
        ss << "|";
    }
    ss << "]                                                                                                ║\n";

    // Add lines for each foot position using foot_positions_ array
    for (int foot = 0; foot < 4; foot++)
    {
      ss << "║ Foot" << (foot + 1) << ": [";
      ss << std::setw(6) << std::fixed << std::setprecision(3) << foot_positions_[foot][0] << ", ";
      ss << std::setw(6) << std::fixed << std::setprecision(3) << foot_positions_[foot][1] << ", ";
      ss << std::setw(6) << std::fixed << std::setprecision(3) << foot_positions_[foot][2] << "]  ";

      if (foot == 3)
      {
        ss << "         ║\n";
      }
    }

    // Print footer line
    ss << "╚═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝";

    // Log the table
    RCLCPP_INFO(get_node()->get_logger(), "Balance Controller Output (Foot Forces and Positions):\n%s", ss.str().c_str());
  }

  inline bool BalanceController::update_state()
  {
    try
    {
      static bool first_update = true;
      static int update_count = 0;
      update_count++;

      if (first_update)
      {
        RCLCPP_INFO(get_node()->get_logger(), "Balance controller received first update call");
        first_update = false;
      }

      // Check if we have received state data
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!latest_state_)
        {
          if (update_count % 100 == 0)
          {
            RCLCPP_INFO(get_node()->get_logger(), "Waiting for state estimation data...");
          }
          return false;
        }

        // Fill current state vector using more structured approach
        // Position and orientation
        current_state_[0] = latest_state_->pc.x;
        current_state_[1] = latest_state_->pc.y;
        current_state_[2] = latest_state_->pc.z;

        // Orientation quaternion
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

        // Map foot positions into arrays more efficiently
        geometry_msgs::msg::Point const *foot_msgs[] = {
            &latest_state_->p1,
            &latest_state_->p2,
            &latest_state_->p3,
            &latest_state_->p4};

        // Update foot positions in both arrays
        for (int foot = 0; foot < 4; foot++)
        {
          int state_idx = 13 + (foot * 3);

          // Update current_state_ array
          current_state_[state_idx] = foot_msgs[foot]->x;
          current_state_[state_idx + 1] = foot_msgs[foot]->y;
          current_state_[state_idx + 2] = foot_msgs[foot]->z;

          // Update foot_positions_ array
          foot_positions_[foot][0] = foot_msgs[foot]->x;
          foot_positions_[foot][1] = foot_msgs[foot]->y;
          foot_positions_[foot][2] = foot_msgs[foot]->z;
        }

        // Store COM position separately
        foot_positions_[4][0] = latest_state_->pc.x;
        foot_positions_[4][1] = latest_state_->pc.y;
        foot_positions_[4][2] = latest_state_->pc.z;
      }

      // Update gait pattern and foot states
      {
        std::lock_guard<std::mutex> lock(gait_mutex_);
        if (!latest_gait_)
        {
          if (update_count % 100 == 0)
          {
            RCLCPP_INFO(get_node()->get_logger(), "Waiting for gait pattern data...");
          }
          return false;
        }

        // Update foot states array with more compact code
        foot_states_[0] = latest_gait_->foot1_state;
        foot_states_[1] = latest_gait_->foot2_state;
        foot_states_[2] = latest_gait_->foot3_state;
        foot_states_[3] = latest_gait_->foot4_state;

        // Update COM offset from gait pattern
        current_state_[0] -= latest_gait_->com_position.x;
        current_state_[1] -= latest_gait_->com_position.y;

        // Unpack predicted states and phases into 2D arrays
        int num_stages = latest_gait_->prediction_stages;

        // Process predicted states (converts from flattened array to 2D)
        if (!latest_gait_->predicted_states.empty() &&
            latest_gait_->predicted_states.size() >= 4 * num_stages)
        {
          for (int stage = 0; stage < num_stages; stage++)
          {
            for (int foot = 0; foot < 4; foot++)
            {
              foot_states_prediction_[stage][foot] = latest_gait_->predicted_states[stage + foot * num_stages];
            }
          }
        }

        // Process predicted phases (converts from flattened array to 2D)
        if (!latest_gait_->predicted_phases.empty() &&
            latest_gait_->predicted_phases.size() >= 4 * num_stages)
        {
          for (int stage = 0; stage < num_stages; stage++)
          {
            for (int foot = 0; foot < 4; foot++)
            {
              foot_phases_prediction_[stage][foot] = latest_gait_->predicted_phases[stage + foot * num_stages];
            }
          }
        }

        // Use more concise logging for foot states
        RCLCPP_DEBUG(get_node()->get_logger(), "Foot states: [%d, %d, %d, %d]",
                     foot_states_[0], foot_states_[1], foot_states_[2], foot_states_[3]);
      }

      // Handle desired state updates from commands
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);

        // Handle pose commands with COM offset applied
        if (new_pose_cmd_received_ && latest_pose_cmd_)
        {
          desired_state_[0] = com_offset_[0];  // Apply X offset
          desired_state_[1] = com_offset_[1];  // Apply Y offset
          desired_state_[2] = latest_pose_cmd_->position.z + com_offset_[2];  // Apply Z offset to commanded height

          // Apply quaternion from command
          desired_state_[3] = latest_pose_cmd_->orientation.w;
          desired_state_[4] = latest_pose_cmd_->orientation.x;
          desired_state_[5] = latest_pose_cmd_->orientation.y;
          desired_state_[6] = latest_pose_cmd_->orientation.z;

          RCLCPP_DEBUG(get_node()->get_logger(), "Updated desired pose with COM offset: [%.3f, %.3f, %.3f]",
                       desired_state_[0], desired_state_[1], desired_state_[2]);
          new_pose_cmd_received_ = false;
        }

        // Handle twist commands
        if (new_twist_cmd_received_ && latest_twist_cmd_)
        {
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
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Error in update_state: %s", e.what());
      return false;
    }
  }

  inline bool BalanceController::update_control()
  {
    try
    {
      // Set reference trajectory for all prediction steps
      for (int stage = 0; stage <= solver_->nlp_solver_plan->N; stage++)
      {
        ocp_nlp_cost_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, stage, "yref", desired_state_.data());
      }

      // Calculate robot mass parameters
      double robot_mass = 15.0; // kg - replace with actual robot mass parameter
      double gravity = 9.81;    // m/s²
      double friction_coef = 0.2; // friction coefficient
      double max_vertical_force = robot_mass * gravity; // Maximum force robot can apply

      // Apply constraints to ALL stages in the optimization horizon
      int horizon_length = solver_->nlp_solver_plan->N;

      // Create and set state bounds vectors more efficiently
      std::vector<double> lowest_state(12, 0.0);
      std::vector<double> highest_state(12, 0.0);

      // Position bounds
      for (int i = 0; i < 2; i++)
      {
        lowest_state[i] = -0.75;
        highest_state[i] = 0.75;
      }
      lowest_state[2] = 0.05;
      highest_state[2] = 0.25;

      // Orientation bounds (quaternion)
      for (int i = 3; i <= 6; i++)
      {
        lowest_state[i] = -1.0;
        highest_state[i] = 1.0;
      }

      // Velocity bounds
      for (int i = 7; i <= 12; i++)
      {
        lowest_state[i] = -10.0;
        highest_state[i] = 10.0;
      }

      // For each stage in the optimization horizon, set constraints based on predicted foot states
      for (int stage = 0; stage < horizon_length; stage++)
      {
        // Create force constraint vectors for this stage
        std::vector<double> min_force(12, 0.0);
        std::vector<double> max_force(12, 0.0);
        std::vector<double> initial_guess(12, 0.0);

        // Count stance feet in this stage based on predicted states
        int num_stance_feet = 0;
        for (int foot = 0; foot < 4; foot++)
        {
          // Use the predicted foot state if available, otherwise fall back to current foot state
          int foot_state = (stage < static_cast<int>(foot_states_prediction_.size())) ? 
                            foot_states_prediction_[stage][foot] : foot_states_[foot];
          
          // States -2, -1, and 0 are stance states
          if (foot_state <= 0)
            num_stance_feet++;
        }

        // Safety check to avoid division by zero
        if (num_stance_feet == 0)
          num_stance_feet = 1;

        // Calculate the vertical force per stance foot to support the robot's weight
        double vertical_force_per_foot = robot_mass * gravity / num_stance_feet;

        // Set foot-specific constraints based on predicted state for this stage
        for (int foot = 0; foot < 4; foot++)
        {
          int base_idx = foot * 3; // Each foot has 3 force components (x,y,z)
          
          // Use the predicted foot state if available, otherwise fall back to current foot state
          int foot_state = foot_states_prediction_[stage][foot];

          // States <= 0 are stance states, > 0 are swing
          if (foot_state > 0)  
          { // Swing - zero force in ALL directions
            for (int axis = 0; axis < 3; axis++)
            {
              min_force[base_idx + axis] = -0.01;
              max_force[base_idx + axis] = 0.01;
              initial_guess[base_idx + axis] = 0.0;
            }
          }
          else
          { // Stance - apply friction cone constraints
            // X and Y bounds (limited by friction cone)
            min_force[base_idx] = -max_vertical_force / 4 * friction_coef;
            max_force[base_idx] = max_vertical_force / 4 * friction_coef;
            min_force[base_idx + 1] = -max_vertical_force / 4 * friction_coef;
            max_force[base_idx + 1] = max_vertical_force / 4 * friction_coef;

            // Z bounds (non-negative vertical force)
            min_force[base_idx + 2] = 0.0;
            max_force[base_idx + 2] = max_vertical_force;

            // Initial guess for stance foot - share weight
            initial_guess[base_idx] = 0.0;     // no lateral force initially
            initial_guess[base_idx + 1] = 0.0; // no lateral force initially
            initial_guess[base_idx + 2] = vertical_force_per_foot;
          }
        }

        // Apply force constraints to the current stage in the ACADOS solver
        ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in,
                                      stage, "lbu", min_force.data());
        ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in,
                                      stage, "ubu", max_force.data());
        
        // Set the state constraints
        ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in,
                                      stage, "lbx", lowest_state.data());
        ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in,
                                      stage, "ubx", highest_state.data());
      }

      // Set initial state constraints
      ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "lbx", current_state_.data());
      ocp_nlp_constraints_model_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_in, 0, "ubx", current_state_.data());
      ocp_nlp_out_set(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "x", current_state_.data());

      // Solve the optimal control problem
      int solve_status = quadruped_ode_acados_solve(solver_);
      if (solve_status != 0)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "ACADOS solver failed with status %d", solve_status);
        return false;
      }

      // Get the optimal control solution
      ocp_nlp_out_get(solver_->nlp_config, solver_->nlp_dims, solver_->nlp_out, 0, "u", optimal_control_.data());

      // Negate the optimal control solution - the solver returns ground reaction forces
      for (auto &force : optimal_control_)
      {
        force = -force;
      }

      // Uncomment these lines for detailed debug output when needed
      //print_state_vector_table();
      //print_controller_output_table();

      // Log debug info about foot forces more efficiently
      const char *foot_names[] = {"Foot1", "Foot2", "Foot3", "Foot4"};

      for (int foot = 0; foot < 4; foot++)
      {
        int base_idx = foot * 3;
        RCLCPP_DEBUG(get_node()->get_logger(),
                     "%s [%s]: X: %.2f, Y: %.2f, Z: %.2f",
                     foot_names[foot],
                     (foot_states_[foot] <= 0) ? "STANCE" : "SWING",
                     optimal_control_[base_idx],
                     optimal_control_[base_idx + 1],
                     optimal_control_[base_idx + 2]);
      }

      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Error in update_control: %s", e.what());
      return false;
    }
  }

  inline bool BalanceController::update_commands()
  {
    try
    {
      // Publish foot forces
      if (foot_forces_publisher_ && foot_forces_publisher_->trylock())
      {
        auto &msg = foot_forces_publisher_->msg_;

        // Set the foot forces more efficiently with a loop
        struct
        {
          double &x;
          double &y;
          double &z;
        } foot_forces[] = {
            {msg.foot1_force.x, msg.foot1_force.y, msg.foot1_force.z},
            {msg.foot2_force.x, msg.foot2_force.y, msg.foot2_force.z},
            {msg.foot3_force.x, msg.foot3_force.y, msg.foot3_force.z},
            {msg.foot4_force.x, msg.foot4_force.y, msg.foot4_force.z}};

        // Update all foot forces in a single loop
        for (int foot = 0; foot < 4; foot++)
        {
          int base_idx = foot * 3;
          foot_forces[foot].x = optimal_control_[base_idx];
          foot_forces[foot].y = optimal_control_[base_idx + 1];
          foot_forces[foot].z = optimal_control_[base_idx + 2];
        }

        msg.total_force.x = -(foot_forces[0].x + foot_forces[1].x + foot_forces[2].x + foot_forces[3].x);
        msg.total_force.y = -(foot_forces[0].y + foot_forces[1].y + foot_forces[2].y + foot_forces[3].y);
        msg.total_force.z = -(foot_forces[0].z + foot_forces[1].z + foot_forces[2].z + foot_forces[3].z);

        foot_forces_publisher_->unlockAndPublish();
      }

      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Error in update_commands: %s", e.what());
      return false;
    }
  }

} // namespace quadruped_mpc

#endif // QUADRUPED_MPC_CONTROL_LAWS_BALANCE_CONTROL_HPP_