#ifndef QUADRUPED_MPC_CONTROL_LAWS_FOOT_CONTROL_HPP_
#define QUADRUPED_MPC_CONTROL_LAWS_FOOT_CONTROL_HPP_

#include "quadruped_mpc/controller_interfaces/foot_controller.hpp"
#include <Eigen/Dense>
#include <sstream>
#include <iomanip>
#include <array>

namespace quadruped_mpc
{
  // Forward declarations for helper functions
  inline Eigen::Vector3d bezier_quadratic(
      const Eigen::Vector3d &p0,
      const Eigen::Vector3d &p1,
      const Eigen::Vector3d &p2,
      double t);

  inline std::vector<Eigen::Vector3d> generate_bezier_path(
      const Eigen::Vector3d &p0,
      const Eigen::Vector3d &p1,
      const Eigen::Vector3d &p2,
      int num_points);

  inline bool read_topics(FootController &controller)
  {
    try
    {
      // Get latest state and gait pattern data from the realtime buffers
      auto state_msg = controller.state_buffer_->readFromRT();
      auto gait_msg = controller.gait_buffer_->readFromRT();

      // Update state data - properly handle the shared pointer
      if (state_msg)
      {
        std::lock_guard<std::mutex> lock(controller.state_mutex_);
        controller.latest_state_ = *state_msg;

        // Only log state sizes if they've changed or on initialization
        static size_t prev_j1_size = 0, prev_j2_size = 0, prev_j3_size = 0, prev_j4_size = 0;
        
        // Create arrays for more concise code
        const std::array<size_t, 4> current_sizes = {
            controller.latest_state_.j1.size(),
            controller.latest_state_.j2.size(),
            controller.latest_state_.j3.size(),
            controller.latest_state_.j4.size()
        };
        
        std::array<size_t*, 4> prev_sizes = {&prev_j1_size, &prev_j2_size, &prev_j3_size, &prev_j4_size};
        bool sizes_changed = false;
        
        // Check if any sizes have changed
        for (size_t i = 0; i < 4; i++) {
            if (current_sizes[i] != *prev_sizes[i]) {
                *prev_sizes[i] = current_sizes[i];
                sizes_changed = true;
            }
        }
        
        if (sizes_changed)
        {
          RCLCPP_DEBUG(controller.get_node()->get_logger(),
                       "Updated state data - J1 size: %zu, J2 size: %zu, J3 size: %zu, J4 size: %zu",
                       current_sizes[0], current_sizes[1], current_sizes[2], current_sizes[3]);
        }
      }

      // Update gait pattern - properly handle the shared pointer
      if (gait_msg)
      {
        std::lock_guard<std::mutex> lock(controller.gait_mutex_);
        controller.latest_gait_ = *gait_msg;
        // Only log once in a while or on significant changes
        static int gait_counter = 0;
        if ((gait_counter++ % 500) == 0)
        {
          RCLCPP_DEBUG(controller.get_node()->get_logger(), "Updated gait pattern data");
        }
      }

      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Error in read_topics: %s", e.what());
      return false;
    }
  }

  inline bool determine_forces(FootController &controller)
  {
    try
    {
      // Get latest foot forces from the realtime buffer
      auto foot_forces_msg = controller.foot_forces_buffer_->readFromRT();

      // Initialize forces structure
      controller.foot_forces_.has_data = false;
      
      // Initialize all foot forces to zero using an array-based approach
      geometry_msgs::msg::Vector3* foot_forces[4] = {
          &controller.foot_forces_.foot1,
          &controller.foot_forces_.foot2,
          &controller.foot_forces_.foot3,
          &controller.foot_forces_.foot4
      };
      
      for (auto& force : foot_forces) {
          force->x = force->y = force->z = 0.0;
      }

      // Acquire lock for accessing gait data
      std::lock_guard<std::mutex> gait_lock(controller.gait_mutex_);

      // If we have forces data from balance controller, update our local foot forces
      if (foot_forces_msg)
      {
        // Set flag that we have data
        controller.foot_forces_.has_data = true;
        
        // Get pointers to the message forces
        const geometry_msgs::msg::Vector3* msg_forces[4] = {
            &foot_forces_msg->foot1_force,
            &foot_forces_msg->foot2_force,
            &foot_forces_msg->foot3_force,
            &foot_forces_msg->foot4_force
        };
        
        // Copy all forces in one loop
        for (int i = 0; i < 4; i++) {
            *foot_forces[i] = *msg_forces[i];
        }
      }
      else
      {
        static int no_data_counter = 0;
        if ((no_data_counter++ % 1000) == 0)
        {
          RCLCPP_DEBUG(controller.get_node()->get_logger(), "No force commands received");
        }
        return false; // No forces to apply
      }

      // Get foot states and swing forces
      int* foot_states[4] = {
          &controller.latest_gait_.foot1_state,
          &controller.latest_gait_.foot2_state,
          &controller.latest_gait_.foot3_state,
          &controller.latest_gait_.foot4_state
      };
      
      geometry_msgs::msg::Vector3* swing_forces[4] = {
          &controller.swing_forces_.foot1,
          &controller.swing_forces_.foot2,
          &controller.swing_forces_.foot3,
          &controller.swing_forces_.foot4
      };
      
      // Log all relevant condition values in debug level
      RCLCPP_DEBUG(controller.get_node()->get_logger(),
                  "Gait and swing conditions: swing_forces.has_data=%d, "
                  "foot1_state=%d, foot2_state=%d, foot3_state=%d, foot4_state=%d",
                  controller.swing_forces_.has_data,
                  *foot_states[0], *foot_states[1], *foot_states[2], *foot_states[3]);

      // Process each foot for swing forces
      const char* foot_names[4] = {"Foot 1", "Foot 2", "Foot 3", "Foot 4"};
      
      for (int i = 0; i < 4; i++)
      {
        if (*foot_states[i] == 1 && controller.swing_forces_.has_data)
        {
          // Apply swing force
          *foot_forces[i] = *swing_forces[i];
          RCLCPP_DEBUG(controller.get_node()->get_logger(),
                      "%s using swing force: [%.2f, %.2f, %.2f]",
                      foot_names[i],
                      swing_forces[i]->x,
                      swing_forces[i]->y,
                      swing_forces[i]->z);
        }
        else
        {
          RCLCPP_DEBUG(controller.get_node()->get_logger(),
                      "%s not using swing force: state=%d, has_data=%d",
                      foot_names[i], *foot_states[i], controller.swing_forces_.has_data);
        }
      }
      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Error in determine_forces: %s", e.what());
      return false;
    }
  }

  inline bool swing_trajectory(FootController &controller, const rclcpp::Time &current_time)
  {
    try
    {
      // Lock mutexes to access state and gait data safely
      std::lock_guard<std::mutex> gait_lock(controller.gait_mutex_);
      std::lock_guard<std::mutex> state_lock(controller.state_mutex_);

      // Get step height from gait pattern message
      double step_height = 0.05; // Default 5cm step height as fallback
      if (controller.latest_gait_.step_height > 0.0) {
        step_height = controller.latest_gait_.step_height;
        // Log when we use step height from message
        static double last_logged_step_height = 0.0;
        if (std::abs(step_height - last_logged_step_height) > 0.001) {
          RCLCPP_INFO(controller.get_node()->get_logger(),
                     "Using step height from gait pattern: %.3f m", step_height);
          last_logged_step_height = step_height;
        }
      } else {
        // Log when using default value
        static bool logged_default = false;
        if (!logged_default) {
          RCLCPP_INFO(controller.get_node()->get_logger(),
                     "Using default step height: %.3f m", step_height);
          logged_default = true;
        }
      }

      const double kp = 500.0;         // Position gain
      const double kd = 10.0;          // Velocity damping gain

      // Structure to store foot-specific data for processing
      struct FootData
      {
        int state;                                 // Foot state (0=stance, 1=swing)
        double phase;                              // Phase in swing cycle (0.0 to 1.0)
        Eigen::Vector3d current_pos;               // Current foot position
        Eigen::Vector3d current_vel;               // Current foot velocity
        Eigen::Vector3d hip_pos;                   // Hip position
        Eigen::Vector3d target_pos;                // Target position for step
        geometry_msgs::msg::Vector3 *trajectories; // Trajectory array pointer
        geometry_msgs::msg::Vector3 *swing_force;  // Where to store resulting force
        const char *name;                          // Name for logging
      };

      // Initialize swing_forces has_data flag to false initially
      controller.swing_forces_.has_data = false;

      // Create combined arrays for state data and gait data to reduce repetitive code
      std::array<geometry_msgs::msg::Vector3 *, 4> swing_forces = {
          &controller.swing_forces_.foot1,
          &controller.swing_forces_.foot2,
          &controller.swing_forces_.foot3,
          &controller.swing_forces_.foot4};

      std::array<geometry_msgs::msg::Vector3 *, 4> traj_arrays = {
          controller.foot_trajectories_.foot1.data(),
          controller.foot_trajectories_.foot2.data(),
          controller.foot_trajectories_.foot3.data(),
          controller.foot_trajectories_.foot4.data()};

      std::array<const char *, 4> foot_names = {
          "Foot 1 (Front left)",
          "Foot 2 (Front right)",
          "Foot 3 (Rear left)",
          "Foot 4 (Rear right)"};

      // Array to hold data for all feet
      std::array<FootData, 4> feet = {{{controller.latest_gait_.foot1_state,
                                        controller.latest_gait_.foot1_phase,
                                        {controller.latest_state_.p1.x, controller.latest_state_.p1.y, controller.latest_state_.p1.z},
                                        {controller.latest_state_.v1.x, controller.latest_state_.v1.y, controller.latest_state_.v1.z},
                                        {controller.latest_state_.h1.x, controller.latest_state_.h1.y, controller.latest_state_.h1.z},
                                        {controller.latest_gait_.foot1_step_position.x, controller.latest_gait_.foot1_step_position.y, controller.latest_gait_.foot1_step_position.z},
                                        traj_arrays[0],
                                        swing_forces[0],
                                        foot_names[0]},
                                       {controller.latest_gait_.foot2_state,
                                        controller.latest_gait_.foot2_phase,
                                        {controller.latest_state_.p2.x, controller.latest_state_.p2.y, controller.latest_state_.p2.z},
                                        {controller.latest_state_.v2.x, controller.latest_state_.v2.y, controller.latest_state_.v2.z},
                                        {controller.latest_state_.h2.x, controller.latest_state_.h2.y, controller.latest_state_.h2.z},
                                        {controller.latest_gait_.foot2_step_position.x, controller.latest_gait_.foot2_step_position.y, controller.latest_gait_.foot2_step_position.z},
                                        traj_arrays[1],
                                        swing_forces[1],
                                        foot_names[1]},
                                       {controller.latest_gait_.foot3_state,
                                        controller.latest_gait_.foot3_phase,
                                        {controller.latest_state_.p3.x, controller.latest_state_.p3.y, controller.latest_state_.p3.z},
                                        {controller.latest_state_.v3.x, controller.latest_state_.v3.y, controller.latest_state_.v3.z},
                                        {controller.latest_state_.h3.x, controller.latest_state_.h3.y, controller.latest_state_.h3.z},
                                        {controller.latest_gait_.foot3_step_position.x, controller.latest_gait_.foot3_step_position.y, controller.latest_gait_.foot3_step_position.z},
                                        traj_arrays[2],
                                        swing_forces[2],
                                        foot_names[2]},
                                       {controller.latest_gait_.foot4_state,
                                        controller.latest_gait_.foot4_phase,
                                        {controller.latest_state_.p4.x, controller.latest_state_.p4.y, controller.latest_state_.p4.z},
                                        {controller.latest_state_.v4.x, controller.latest_state_.v4.y, controller.latest_state_.v4.z},
                                        {controller.latest_state_.h4.x, controller.latest_state_.h4.y, controller.latest_state_.h4.z},
                                        {controller.latest_gait_.foot4_step_position.x, controller.latest_gait_.foot4_step_position.y, controller.latest_gait_.foot4_step_position.z},
                                        traj_arrays[3],
                                        swing_forces[3],
                                        foot_names[3]}}};

      // Helper function to generate and store trajectory
      auto generate_trajectory = [&](const FootData &foot, int foot_idx)
      {
        Eigen::Vector3d p0 = foot.current_pos;
        // Use the step_height parameter for trajectory generation
        Eigen::Vector3d p1(foot.hip_pos.x(), foot.hip_pos.y(), foot.hip_pos.z() + step_height);
        auto trajectory = generate_bezier_path(p0, p1, foot.target_pos, 50);

        // Store the trajectory
        for (size_t i = 0; i < trajectory.size() && i < 50; ++i)
        {
          foot.trajectories[i].x = trajectory[i][0];
          foot.trajectories[i].y = trajectory[i][1];
          foot.trajectories[i].z = trajectory[i][2];
        }

        // Add INFO level logging for control points with foot number
        RCLCPP_INFO(controller.get_node()->get_logger(),
                   "Foot %d (%s) trajectory control points: start=[%.3f, %.3f, %.3f], mid=[%.3f, %.3f, %.3f], end=[%.3f, %.3f, %.3f]",
                   foot_idx + 1, foot.name, 
                   p0.x(), p0.y(), p0.z(),
                   p1.x(), p1.y(), p1.z(),
                   foot.target_pos.x(), foot.target_pos.y(), foot.target_pos.z());

        // Log trajectory details at debug level
        RCLCPP_DEBUG(controller.get_node()->get_logger(),
                     "Generated trajectory for %s: from [%.3f, %.3f, %.3f] through [%.3f, %.3f, %.3f] to [%.3f, %.3f, %.3f]",
                     foot.name, p0.x(), p0.y(), p0.z(), p1.x(), p1.y(), p1.z(),
                     foot.target_pos.x(), foot.target_pos.y(), foot.target_pos.z());

        if (RCUTILS_LOG_SEVERITY_DEBUG >= rcutils_logging_get_logger_level(
                                              controller.get_node()->get_logger().get_name()))
        {
          std::stringstream traj_ss;
          traj_ss << foot.name << " trajectory points:";
          traj_ss << "\n  Hip Position: [" << foot.hip_pos.x() << ", "
                  << foot.hip_pos.y() << ", " << foot.hip_pos.z() << "]";
          traj_ss << "\n  Control Points:";
          traj_ss << "\n    p0 (start): [" << p0.x() << ", " << p0.y() << ", " << p0.z() << "]";
          traj_ss << "\n    p1 (mid)  : [" << p1.x() << ", " << p1.y() << ", " << p1.z() << "]";
          traj_ss << "\n    p2 (end)  : [" << foot.target_pos.x() << ", "
                  << foot.target_pos.y() << ", " << foot.target_pos.z() << "]";
          traj_ss << "\n  Trajectory (first few points):";
          for (size_t i = 0; i < std::min(trajectory.size(), static_cast<size_t>(5)); ++i)
          {
            traj_ss << "\n    [" << i << "]: ["
                    << foot.trajectories[i].x << ", "
                    << foot.trajectories[i].y << ", "
                    << foot.trajectories[i].z << "]";
          }
          RCLCPP_DEBUG(controller.get_node()->get_logger(), "%s", traj_ss.str().c_str());
        }
      };

      // Helper function to apply PD control for a swinging foot
      auto apply_pd_control = [&](const FootData &foot, int trajectory_idx) -> std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
      {
        Eigen::Vector3d target_pos(
            foot.trajectories[trajectory_idx].x,
            foot.trajectories[trajectory_idx].y,
            foot.trajectories[trajectory_idx].z);

        Eigen::Vector3d pos_error = target_pos - foot.current_pos;
        Eigen::Vector3d pd_force = kp * pos_error - kd * foot.current_vel;

        foot.swing_force->x = pd_force(0);
        foot.swing_force->y = pd_force(1);
        foot.swing_force->z = pd_force(2);

        return {target_pos, pos_error, pd_force};
      };

      // Process each foot
      for (int i = 0; i < 4; ++i)
      {
        // Skip inactive feet (not in swing phase)
        if (feet[i].state != 1)
          continue;

        RCLCPP_DEBUG(controller.get_node()->get_logger(), "Entered %s pd control", feet[i].name);

        // Generate trajectory at the beginning of swing phase
        if (feet[i].phase == 0.0)
        {
          RCLCPP_DEBUG(controller.get_node()->get_logger(), "%s generating new trajectory", feet[i].name);
          generate_trajectory(feet[i], i);
        }

        // Use phase to determine trajectory index (phase goes from 0.0 to 1.0)
        int trajectory_idx = static_cast<int>(feet[i].phase * 49);  // Map 0.0-1.0 to 0-49
        trajectory_idx = std::min(49, std::max(0, trajectory_idx)); // Clamp to valid range

        // Apply PD control
        apply_pd_control(feet[i], trajectory_idx);

        // Mark that we have valid swing force data
        controller.swing_forces_.has_data = true;
      }

      // Periodic overall debug
      static int swing_counter = 0;
      if ((swing_counter++ % 500) == 0)
      {
        RCLCPP_DEBUG(controller.get_node()->get_logger(),
                     "Swing trajectory states: F1=%d(%.2f), F2=%d(%.2f), F3=%d(%.2f), F4=%d(%.2f)",
                     feet[0].state, feet[0].phase,
                     feet[1].state, feet[1].phase,
                     feet[2].state, feet[2].phase,
                     feet[3].state, feet[3].phase);
      }

      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Error in swing_trajectory: %s", e.what());
      return false;
    }
  }

  inline bool apply_jacobians(FootController &controller)
  {
    try
    {
      // Check if we have foot force data
      if (!controller.foot_forces_.has_data)
      {
        RCLCPP_DEBUG(controller.get_node()->get_logger(), "No foot force data available");
        for (auto &interface : controller.joint_command_interfaces_)
        {
          interface.set_value(0.0);
        }
        return true;
      }

      std::lock_guard<std::mutex> lock(controller.state_mutex_);

      // Define arrays for jacobian access
      const std::vector<double>* jacobian_arrays[4] = {
          &controller.latest_state_.j1,
          &controller.latest_state_.j2,
          &controller.latest_state_.j3,
          &controller.latest_state_.j4
      };
      
      // Check for valid Jacobian data
      bool jacobians_empty = false;
      bool jacobians_wrong_size = false;
      
      for (const auto* jacobian : jacobian_arrays) {
          if (jacobian->empty()) {
              jacobians_empty = true;
              break;
          }
          if (jacobian->size() < 6) {
              jacobians_wrong_size = true;
              break;
          }
      }
      
      if (jacobians_empty || jacobians_wrong_size)
      {
        static int jacobian_error_counter = 0;
        if ((jacobian_error_counter++ % 100) == 0)
        {
          if (jacobians_empty)
          {
            RCLCPP_WARN(controller.get_node()->get_logger(),
                        "Jacobian data incomplete: j1=%s, j2=%s, j3=%s, j4=%s",
                        jacobian_arrays[0]->empty() ? "empty" : "has data",
                        jacobian_arrays[1]->empty() ? "empty" : "has data",
                        jacobian_arrays[2]->empty() ? "empty" : "has data",
                        jacobian_arrays[3]->empty() ? "empty" : "has data");
          }

          if (jacobians_wrong_size)
          {
            RCLCPP_WARN(controller.get_node()->get_logger(),
                        "Jacobian arrays wrong size: j1=%zu, j2=%zu, j3=%zu, j4=%zu (expected 6 each)",
                        jacobian_arrays[0]->size(), jacobian_arrays[1]->size(),
                        jacobian_arrays[2]->size(), jacobian_arrays[3]->size());
          }
        }

        for (auto &interface : controller.joint_command_interfaces_)
        {
          interface.set_value(0.0);
        }
        return true;
      }

      // Check if we have enough interfaces
      if (controller.joint_command_interfaces_.size() < 8)
      {
        RCLCPP_WARN(controller.get_node()->get_logger(),
                    "Not enough command interfaces available: %zu (expected 8)",
                    controller.joint_command_interfaces_.size());

        for (auto &interface : controller.joint_command_interfaces_)
        {
          interface.set_value(0.0);
        }
        return true;
      }

      // Create arrays for forces for more compact code
      const geometry_msgs::msg::Vector3* force_msgs[4] = {
          &controller.foot_forces_.foot1,
          &controller.foot_forces_.foot2,
          &controller.foot_forces_.foot3,
          &controller.foot_forces_.foot4
      };
      
      std::array<Eigen::Vector3d, 4> forces;
      std::array<Eigen::Matrix<double, 3, 2>, 4> jacobians;
      
      // Fill the forces and jacobians in loops
      for (int i = 0; i < 4; i++) {
          forces[i] = Eigen::Vector3d(force_msgs[i]->x, force_msgs[i]->y, force_msgs[i]->z);
          
          // Create the jacobian matrix
          const auto& j = *jacobian_arrays[i];
          jacobians[i] << j[0], j[3],
                          j[1], j[4],
                          j[2], j[5];
      }

      // Calculate total vertical force
      double total_vertical_force = 0.0;
      for (const auto& force : forces) {
          total_vertical_force += force.z();
      }

      // Compute torques
      std::array<Eigen::Vector2d, 4> torques;
      std::array<const char*, 4> leg_names = {"FL", "FR", "RL", "RR"};

      // Calculate power
      double total_power = 0.0;
      
      for (int i = 0; i < 4; i++)
      {
        // Get joint velocities (if available from the state message) 
        double v1 = 0.0, v2 = 0.0;
        
        // Calculate indices for this leg's joints from the joint_states array
        int joint_idx1 = i * 2;
        int joint_idx2 = i * 2 + 1;
        
        // Extract velocities if they exist in the state message
        if (controller.latest_state_.joint_velocities.size() > joint_idx2) {
          v1 = controller.latest_state_.joint_velocities[joint_idx1];
          v2 = controller.latest_state_.joint_velocities[joint_idx2];
        }

        // Calculate torques using the jacobian transpose
        torques[i] = jacobians[i].transpose() * forces[i];
        
        // Calculate power (torque * angular velocity)
        double power = torques[i](0) * v1 + torques[i](1) * v2;
        total_power += power;
        
        // Set torque values to joint interfaces
        controller.joint_command_interfaces_[i * 2].set_value(torques[i][0]);
        controller.joint_command_interfaces_[i * 2 + 1].set_value(torques[i][1]);
        
        // Check torques against limits
        const double torque_limit = 8.0; // 8 Newton-meters
        if (std::abs(torques[i](0)) >= torque_limit || std::abs(torques[i](1)) >= torque_limit)
        {
          RCLCPP_WARN(controller.get_node()->get_logger(),
                      "%s torque exceeds limit: [%.2f, %.2f] Nm (limit: %.2f Nm)",
                      leg_names[i], torques[i](0), torques[i](1), torque_limit);
        }
      }

      // Log foot forces and states in a formatted table with Unicode borders on every cycle
      {
        // Get foot states for the table
        std::array<int, 4> foot_states = {
          controller.latest_gait_.foot1_state,
          controller.latest_gait_.foot2_state,
          controller.latest_gait_.foot3_state,
          controller.latest_gait_.foot4_state
        };

        // Create table header and separator with Unicode borders
        std::stringstream ss;
        ss << "\nFoot Forces and States:\n";
        ss << "┌──────┬─────────────┬─────────────┬─────────────┬───────┐\n";
        ss << "│ Foot │   Force X   │   Force Y   │   Force Z   │ State │\n";
        ss << "├──────┼─────────────┼─────────────┼─────────────┼───────┤\n";

        // Calculate sums for summary row
        double sum_x = 0.0, sum_y = 0.0;
        
        // Add each foot's data
        for (int i = 0; i < 4; i++) {
          ss << "│ " << std::setw(4) << std::left << leg_names[i] << " │ ";
          ss << std::setw(11) << std::right << std::fixed << std::setprecision(3) << forces[i].x() << " │ ";
          ss << std::setw(11) << std::right << std::fixed << std::setprecision(3) << forces[i].y() << " │ ";
          ss << std::setw(11) << std::right << std::fixed << std::setprecision(3) << forces[i].z() << " │ ";
          
          // State: 0=stance, 1=swing
          std::string state_str = (foot_states[i] == 1) ? "SWING" : "STANCE";
          ss << std::setw(5) << std::left << state_str << " │";
          ss << "\n";
          
          // Add to sums
          sum_x += forces[i].x();
          sum_y += forces[i].y();
        }

        // Add table footer with summary
        ss << "├──────┼─────────────┼─────────────┼─────────────┼───────┤\n";
        ss << "│ SUM  │ " << std::setw(11) << std::right << std::fixed << std::setprecision(3) << sum_x << " │ ";
        ss << std::setw(11) << std::right << std::fixed << std::setprecision(3) << sum_y << " │ ";
        ss << std::setw(11) << std::right << std::fixed << std::setprecision(3)
           << total_vertical_force << " │       │\n";
        ss << "└──────┴─────────────┴─────────────┴─────────────┴───────┘\n";
        
        // Add power information
        ss << "Total Mechanical Power: " << std::fixed << std::setprecision(3) << total_power << " W\n";

        // Log the entire table at once on every cycle
        //RCLCPP_INFO(controller.get_node()->get_logger(), "%s", ss.str().c_str());
      }

      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Exception in apply_jacobians: %s", e.what());

      try
      {
        for (auto &interface : controller.joint_command_interfaces_)
        {
          interface.set_value(0.0);
        }
      }
      catch (...)
      {
        RCLCPP_ERROR(controller.get_node()->get_logger(), "Failed to apply zero torques after exception");
      }

      return true;
    }
  }

  inline Eigen::Vector3d bezier_quadratic(
      const Eigen::Vector3d &p0,
      const Eigen::Vector3d &p1,
      const Eigen::Vector3d &p2,
      double t)
  {
    t = std::max(0.0, std::min(1.0, t));
    double t1 = 1.0 - t;
    return t1 * t1 * p0 + 2.0 * t1 * t * p1 + t * t * p2;
  }

  inline std::vector<Eigen::Vector3d> generate_bezier_path(
      const Eigen::Vector3d &p0,
      const Eigen::Vector3d &p1,
      const Eigen::Vector3d &p2,
      int num_points)
  {
    std::vector<Eigen::Vector3d> path;
    path.reserve(num_points);
    for (int i = 0; i < num_points; i++)
    {
      double t = static_cast<double>(i) / (num_points - 1);
      path.push_back(bezier_quadratic(p0, p1, p2, t));
    }
    return path;
  }

} // namespace quadruped_mpc

#endif // QUADRUPED_MPC_CONTROL_LAWS_FOOT_CONTROL_HPP_
