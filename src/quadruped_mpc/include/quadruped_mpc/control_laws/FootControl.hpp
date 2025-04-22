#ifndef QUADRUPED_MPC_CONTROL_LAWS_FOOT_CONTROL_HPP_
#define QUADRUPED_MPC_CONTROL_LAWS_FOOT_CONTROL_HPP_

#include "quadruped_mpc/controller_interfaces/foot_controller.hpp"
#include <Eigen/Dense>
#include <sstream>
#include <iomanip>

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

  // Helper to log vector values
  inline std::string vec3_to_string(const Eigen::Vector3d &vec)
  {
    std::stringstream ss;
    ss << "[" << vec[0] << ", " << vec[1] << ", " << vec[2] << "]";
    return ss.str();
  }

  // Helper to log vector2d values
  inline std::string vec2_to_string(const Eigen::Vector2d &vec)
  {
    std::stringstream ss;
    ss << "[" << vec[0] << ", " << vec[1] << "]";
    return ss.str();
  }
  
  // Helper to visualize a mapping between forces and torques for debugging
  inline std::string visualize_force_torque_mapping(
      const Eigen::Vector3d& force,
      const Eigen::Vector2d& torque,
      const Eigen::Matrix<double, 3, 2>& jacobian)
  {
    std::stringstream ss;
    ss << "\n    Force: " << vec3_to_string(force);
    ss << "\n    Jacobian:\n";
    ss << "      [ " << std::fixed << std::setprecision(4) << jacobian(0, 0) << ", " << jacobian(0, 1) << " ]\n";
    ss << "      [ " << std::fixed << std::setprecision(4) << jacobian(1, 0) << ", " << jacobian(1, 1) << " ]\n";
    ss << "      [ " << std::fixed << std::setprecision(4) << jacobian(2, 0) << ", " << jacobian(2, 1) << " ]\n";
    ss << "    Torque: " << vec2_to_string(torque);
    return ss.str();
  }

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
        if (controller.latest_state_.j1.size() != prev_j1_size ||
            controller.latest_state_.j2.size() != prev_j2_size ||
            controller.latest_state_.j3.size() != prev_j3_size ||
            controller.latest_state_.j4.size() != prev_j4_size)
        {
          prev_j1_size = controller.latest_state_.j1.size();
          prev_j2_size = controller.latest_state_.j2.size();
          prev_j3_size = controller.latest_state_.j3.size();
          prev_j4_size = controller.latest_state_.j4.size();

          RCLCPP_DEBUG(controller.get_node()->get_logger(),
                       "Updated state data - J1 size: %zu, J2 size: %zu, J3 size: %zu, J4 size: %zu",
                       controller.latest_state_.j1.size(),
                       controller.latest_state_.j2.size(),
                       controller.latest_state_.j3.size(),
                       controller.latest_state_.j4.size());
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

      // Initialize final forces with zeros
      controller.foot_forces_.foot1.x = 0.0;
      controller.foot_forces_.foot1.y = 0.0;
      controller.foot_forces_.foot1.z = 0.0;
      controller.foot_forces_.foot2.x = 0.0;
      controller.foot_forces_.foot2.y = 0.0;
      controller.foot_forces_.foot2.z = 0.0;
      controller.foot_forces_.foot3.x = 0.0;
      controller.foot_forces_.foot3.y = 0.0;
      controller.foot_forces_.foot3.z = 0.0;
      controller.foot_forces_.foot4.x = 0.0;
      controller.foot_forces_.foot4.y = 0.0;
      controller.foot_forces_.foot4.z = 0.0;

      // By default, we don't have valid data
      controller.foot_forces_.has_data = false;

      // Acquire locks for accessing state and gait data
      std::lock_guard<std::mutex> gait_lock(controller.gait_mutex_);

      // If we have forces data from balance controller, update our local foot forces
      if (foot_forces_msg)
      {
        // Set flag that we have data
        controller.foot_forces_.has_data = true;

        // Start with the forces from the balance controller
        controller.foot_forces_.foot1 = foot_forces_msg->foot1_force;
        controller.foot_forces_.foot2 = foot_forces_msg->foot2_force;
        controller.foot_forces_.foot3 = foot_forces_msg->foot3_force;
        controller.foot_forces_.foot4 = foot_forces_msg->foot4_force;
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

      // Log all relevant condition values in debug level instead of info
      RCLCPP_DEBUG(controller.get_node()->get_logger(), 
         "Gait and swing conditions: swing_forces.has_data=%d, "
         "foot1_state=%d, foot2_state=%d, foot3_state=%d, foot4_state=%d",
         controller.swing_forces_.has_data,
         controller.latest_gait_.foot1_state,
         controller.latest_gait_.foot2_state, 
         controller.latest_gait_.foot3_state,
         controller.latest_gait_.foot4_state);

      // For feet in swing state (state == 1), use the forces from swing trajectory
      if (controller.latest_gait_.foot1_state == 1 && controller.swing_forces_.has_data)
      {
        controller.foot_forces_.foot1 = controller.swing_forces_.foot1;
        RCLCPP_DEBUG(controller.get_node()->get_logger(), 
          "Foot 1 using swing force: [%.2f, %.2f, %.2f]",
          controller.swing_forces_.foot1.x,
          controller.swing_forces_.foot1.y,
          controller.swing_forces_.foot1.z);
      }
      else
      {
        RCLCPP_DEBUG(controller.get_node()->get_logger(),
           "Foot 1 not using swing force: state=%d, has_data=%d",
           controller.latest_gait_.foot1_state, controller.swing_forces_.has_data);
      }

      if (controller.latest_gait_.foot2_state == 1 && controller.swing_forces_.has_data)
      {
        controller.foot_forces_.foot2 = controller.swing_forces_.foot2;
        RCLCPP_DEBUG(controller.get_node()->get_logger(), 
          "Foot 2 using swing force: [%.2f, %.2f, %.2f]",
          controller.swing_forces_.foot2.x,
          controller.swing_forces_.foot2.y,
          controller.swing_forces_.foot2.z);
      }
      else
      {
        RCLCPP_DEBUG(controller.get_node()->get_logger(),
           "Foot 2 not using swing force: state=%d, has_data=%d",
           controller.latest_gait_.foot2_state, controller.swing_forces_.has_data);
      }

      if (controller.latest_gait_.foot3_state == 1 && controller.swing_forces_.has_data)
      {
        controller.foot_forces_.foot3 = controller.swing_forces_.foot3;
        RCLCPP_DEBUG(controller.get_node()->get_logger(), 
          "Foot 3 using swing force: [%.2f, %.2f, %.2f]",
          controller.swing_forces_.foot3.x,
          controller.swing_forces_.foot3.y,
          controller.swing_forces_.foot3.z);
      }
      else
      {
        RCLCPP_DEBUG(controller.get_node()->get_logger(),
           "Foot 3 not using swing force: state=%d, has_data=%d",
           controller.latest_gait_.foot3_state, controller.swing_forces_.has_data);
      }

      if (controller.latest_gait_.foot4_state == 1 && controller.swing_forces_.has_data)
      {
        controller.foot_forces_.foot4 = controller.swing_forces_.foot4;
        RCLCPP_DEBUG(controller.get_node()->get_logger(), 
          "Foot 4 using swing force: [%.2f, %.2f, %.2f]",
          controller.swing_forces_.foot4.x,
          controller.swing_forces_.foot4.y,
          controller.swing_forces_.foot4.z);
      }
      else
      {
        RCLCPP_DEBUG(controller.get_node()->get_logger(),
           "Foot 4 not using swing force: state=%d, has_data=%d",
           controller.latest_gait_.foot4_state, controller.swing_forces_.has_data);
      }

      // Generate consolidated force table with all feet
      std::lock_guard<std::mutex> state_lock(controller.state_mutex_);
      std::stringstream table;
      table << "\n╔═════╦══════╦═══════╦══════════════════════════╦══════════════════════════╦══════════════════════════╦══════════════════════════╗";
      table << "\n║FOOT ║STATE ║ PHASE ║          TARGET          ║        CURRENT           ║      TRAJECTORY ENDPT    ║           FORCE          ║";
      table << "\n╠═════╬══════╬═══════╬══════════════════════════╬══════════════════════════╬══════════════════════════╬══════════════════════════╣";
      
      // Array of references to make the code more compact
      const std::array<std::string, 4> foot_names = {"FL", "FR", "RL", "RR"};
      const std::array<int, 4> foot_states = {
          controller.latest_gait_.foot1_state,
          controller.latest_gait_.foot2_state,
          controller.latest_gait_.foot3_state,
          controller.latest_gait_.foot4_state
      };
      const std::array<double, 4> foot_phases = {
          controller.latest_gait_.foot1_phase,
          controller.latest_gait_.foot2_phase,
          controller.latest_gait_.foot3_phase,
          controller.latest_gait_.foot4_phase
      };
      
      // Access current positions
      const std::array<Eigen::Vector3d, 4> current_positions = {
          Eigen::Vector3d(controller.latest_state_.p1.x, controller.latest_state_.p1.y, controller.latest_state_.p1.z),
          Eigen::Vector3d(controller.latest_state_.p2.x, controller.latest_state_.p2.y, controller.latest_state_.p2.z),
          Eigen::Vector3d(controller.latest_state_.p3.x, controller.latest_state_.p3.y, controller.latest_state_.p3.z),
          Eigen::Vector3d(controller.latest_state_.p4.x, controller.latest_state_.p4.y, controller.latest_state_.p4.z)
      };
      
      // Access target positions for feet in swing phase
      std::array<Eigen::Vector3d, 4> target_positions;
      std::array<bool, 4> has_target_position = {false, false, false, false};
      // Keep track of trajectory endpoints for each foot
      std::array<Eigen::Vector3d, 4> trajectory_endpoints;
      for (int i = 0; i < 4; ++i) {
        if (foot_states[i] == 1 && foot_phases[i] >= 0.0 && foot_phases[i] <= 1.0) {
          int trajectory_idx = static_cast<int>(foot_phases[i] * 49);
          trajectory_idx = std::min(49, std::max(0, trajectory_idx));
          
          // Store the final trajectory point (endpoint) based on foot number
          switch (i) {
            case 0: // FL
              target_positions[i] = Eigen::Vector3d(
                controller.foot_trajectories_.foot1[trajectory_idx].x,
                controller.foot_trajectories_.foot1[trajectory_idx].y,
                controller.foot_trajectories_.foot1[trajectory_idx].z);
              trajectory_endpoints[i] = Eigen::Vector3d(
                controller.foot_trajectories_.foot1[49].x,
                controller.foot_trajectories_.foot1[49].y,
                controller.foot_trajectories_.foot1[49].z);
              has_target_position[i] = true;
              break;
            case 1: // FR
              target_positions[i] = Eigen::Vector3d(
                controller.foot_trajectories_.foot2[trajectory_idx].x,
                controller.foot_trajectories_.foot2[trajectory_idx].y,
                controller.foot_trajectories_.foot2[trajectory_idx].z);
              trajectory_endpoints[i] = Eigen::Vector3d(
                controller.foot_trajectories_.foot2[49].x,
                controller.foot_trajectories_.foot2[49].y,
                controller.foot_trajectories_.foot2[49].z);
              has_target_position[i] = true;
              break;
            case 2: // RL
              target_positions[i] = Eigen::Vector3d(
                controller.foot_trajectories_.foot3[trajectory_idx].x,
                controller.foot_trajectories_.foot3[trajectory_idx].y,
                controller.foot_trajectories_.foot3[trajectory_idx].z);
              trajectory_endpoints[i] = Eigen::Vector3d(
                controller.foot_trajectories_.foot3[49].x,
                controller.foot_trajectories_.foot3[49].y,
                controller.foot_trajectories_.foot3[49].z);
              has_target_position[i] = true;
              break;
            case 3: // RR
              target_positions[i] = Eigen::Vector3d(
                controller.foot_trajectories_.foot4[trajectory_idx].x,
                controller.foot_trajectories_.foot4[trajectory_idx].y,
                controller.foot_trajectories_.foot4[trajectory_idx].z);
              trajectory_endpoints[i] = Eigen::Vector3d(
                controller.foot_trajectories_.foot4[49].x,
                controller.foot_trajectories_.foot4[49].y,
                controller.foot_trajectories_.foot4[49].z);
              has_target_position[i] = true;
              break;
          }
        } else {
          // For feet in stance phase, just initialize with zeros
          trajectory_endpoints[i] = Eigen::Vector3d::Zero();
        }
      }
      
      // Access forces
      const std::array<Eigen::Vector3d, 4> forces = {
          Eigen::Vector3d(controller.foot_forces_.foot1.x, controller.foot_forces_.foot1.y, controller.foot_forces_.foot1.z),
          Eigen::Vector3d(controller.foot_forces_.foot2.x, controller.foot_forces_.foot2.y, controller.foot_forces_.foot2.z),
          Eigen::Vector3d(controller.foot_forces_.foot3.x, controller.foot_forces_.foot3.y, controller.foot_forces_.foot3.z),
          Eigen::Vector3d(controller.foot_forces_.foot4.x, controller.foot_forces_.foot4.y, controller.foot_forces_.foot4.z)
      };

      // Format and add each foot's data to the table
      for (int i = 0; i < 4; ++i) {
        std::string state_str = (foot_states[i] == 1) ? "SWING " : "STANCE";
        
        // Format position vectors with consistent width
        std::stringstream target_ss, current_ss, endpoint_ss, force_ss;
        
        // For target position, ensure consistent formatting regardless of stance/swing
        target_ss << std::fixed << std::setprecision(3);
        if (has_target_position[i]) {
          target_ss << "["
                  << std::setw(7) << target_positions[i].x() << ","
                  << std::setw(7) << target_positions[i].y() << ","
                  << std::setw(7) << target_positions[i].z() << "]";
        } else {
          // Format N/A with exact same width as the vector representation
          target_ss << "[      N/A              ]";
        }
        
        // Always show current position with consistent formatting
        current_ss << std::fixed << std::setprecision(3);
        current_ss << "["
                << std::setw(7) << current_positions[i].x() << ","
                << std::setw(7) << current_positions[i].y() << ","
                << std::setw(7) << current_positions[i].z() << "]";

        // Show trajectory endpoint data if target position exists
        endpoint_ss << std::fixed << std::setprecision(3);
        if (has_target_position[i]) {
          // Display the trajectory endpoint instead of the error
          endpoint_ss << "["
                 << std::setw(7) << trajectory_endpoints[i].x() << ","
                 << std::setw(7) << trajectory_endpoints[i].y() << ","
                 << std::setw(7) << trajectory_endpoints[i].z() << "]";
        } else {
          // Format N/A with exact same width as the vector representation
          endpoint_ss << "[      N/A              ]";
        }
        
        // Always show forces with consistent formatting - adjust precision and width for better alignment
        force_ss << std::fixed << std::setprecision(2);  // Increased precision for better formatting
        force_ss << "["
               << std::setw(7) << forces[i].x() << ","   // Increased width from 6 to 7 to match other columns
               << std::setw(7) << forces[i].y() << ","   // Increased width from 6 to 7 to match other columns
               << std::setw(7) << forces[i].z() << "]";  // Increased width from 6 to 7 to match other columns
        
        // Format the table row with consistent width columns
        table << "\n║ " << std::left << std::setw(3) << foot_names[i] << " ║"
              << std::setw(6) << state_str << "║" 
              << std::fixed << std::setprecision(2) << std::right << std::setw(7) << foot_phases[i] << "║"
              << std::setw(22) << target_ss.str() << " ║" 
              << std::setw(22) << current_ss.str() << " ║" 
              << std::setw(22) << endpoint_ss.str() << " ║" 
              << std::setw(22) << force_ss.str() << " ║";
      }
      
      // Calculate total stance forces
      Eigen::Vector3d total_stance_force = Eigen::Vector3d::Zero();
      int stance_foot_count = 0;
      
      for (int i = 0; i < 4; ++i) {
        if (foot_states[i] != 1) {  // If foot is in stance phase
          total_stance_force += forces[i];
          stance_foot_count++;
        }
      }
      
      // Add separator line and total stance force row
      table << "\n╠═════╩══════╩═══════╩══════════════════════════╩══════════════════════════╩══════════════════════════╬══════════════════════════╣";
      
      // Format the total stance force with consistent styling
      std::stringstream total_force_ss;
      total_force_ss << std::fixed << std::setprecision(2);
      total_force_ss << "["
                   << std::setw(7) << total_stance_force.x() << ","
                   << std::setw(7) << total_stance_force.y() << ","
                   << std::setw(7) << total_stance_force.z() << "]";
      
      // Add total stance force row (spanning multiple columns)
      table << "\n║ TOTAL STANCE FORCE (" << stance_foot_count << " feet)                                                                            ║ "
            << std::setw(22) << total_force_ss.str() << " ║";
      
      table << "\n╚═════════════════════════════════════════════════════════════════════════════════════════════════════╩══════════════════════════╝";
      RCLCPP_INFO(controller.get_node()->get_logger(), "Foot Forces and Trajectories: %s", table.str().c_str());

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

      // Default step height (this could be a parameter)
      const double step_height = 0.05; // 5cm step height

      // PD control gains - these could be parameters
      const double kp = 500.0; // Position gain
      const double kd = 10.0;  // Velocity damping gain

      // Generate trajectories if needed and apply PD control for swing feet

      // Initialize a simple tracking structure for state info
      struct TrajectoryInfo {
        bool active = false;
        double phase = 0.0;
        int trajectory_idx = 0;
        Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d current_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d pos_error = Eigen::Vector3d::Zero();
        Eigen::Vector3d pd_force = Eigen::Vector3d::Zero();
      };
      TrajectoryInfo foot_info[4]; // Data for all 4 feet

      // Foot 1 (Front left)
      if (controller.latest_gait_.foot1_state == 1)
      {
        RCLCPP_DEBUG(controller.get_node()->get_logger(),
              "Entered foot 1 pd control if statement");
        // Generate trajectory at the beginning of swing phase
        if (controller.latest_gait_.foot1_phase == 0.0)
        {
          RCLCPP_DEBUG(controller.get_node()->get_logger(), "Foot 1 (Front left) generating new trajectory");

          // Extract current position (p0) from the state message
          Eigen::Vector3d p0(
          controller.latest_state_.p1.x,
          controller.latest_state_.p1.y,
          controller.latest_state_.p1.z);

          // Create middle control point (p1): same x,y as hip but raised by step_height
          Eigen::Vector3d p1(
          controller.latest_state_.h1.x,
          controller.latest_state_.h1.y,
          controller.latest_state_.h1.z + step_height);

          // Target position (p2) from gait pattern
          Eigen::Vector3d p2(
          controller.latest_gait_.foot1_step_position.x,
          controller.latest_gait_.foot1_step_position.y,
          controller.latest_gait_.foot1_step_position.z);

          // Generate the trajectory using our helper function
          auto trajectory = generate_bezier_path(p0, p1, p2, 50);

          // Store the trajectory
          for (size_t i = 0; i < trajectory.size() && i < 50; ++i)
          {
        controller.foot_trajectories_.foot1[i].x = trajectory[i][0];
        controller.foot_trajectories_.foot1[i].y = trajectory[i][1];
        controller.foot_trajectories_.foot1[i].z = trajectory[i][2];
          }

          // Log the entire trajectory at debug level
          std::stringstream traj_ss;
          traj_ss << "Foot 1 trajectory points:";
          traj_ss << "\n  Hip Position: [" 
             << controller.latest_state_.h1.x << ", "
             << controller.latest_state_.h1.y << ", "
             << controller.latest_state_.h1.z << "]";
          traj_ss << "\n  Control Points:";
          traj_ss << "\n    p0 (start): [" << p0.x() << ", " << p0.y() << ", " << p0.z() << "]";
          traj_ss << "\n    p1 (mid)  : [" << p1.x() << ", " << p1.y() << ", " << p1.z() << "]";
          traj_ss << "\n    p2 (end)  : [" << p2.x() << ", " << p2.y() << ", " << p2.z() << "]";
          traj_ss << "\n  Trajectory:";
          for (size_t i = 0; i < std::min(trajectory.size(), static_cast<size_t>(50)); ++i) {
        traj_ss << "\n    [" << i << "]: [" 
           << controller.foot_trajectories_.foot1[i].x << ", "
           << controller.foot_trajectories_.foot1[i].y << ", "
           << controller.foot_trajectories_.foot1[i].z << "]";
          }
          RCLCPP_DEBUG(controller.get_node()->get_logger(), "%s\n\n", traj_ss.str().c_str());

          RCLCPP_DEBUG(controller.get_node()->get_logger(),
              "Generated trajectory for foot1: from %s through %s to %s",
              vec3_to_string(p0).c_str(), vec3_to_string(p1).c_str(), vec3_to_string(p2).c_str());
        }

        // Use phase to determine trajectory index (phase goes from 0.0 to 1.0)
        double phase = controller.latest_gait_.foot1_phase;
        int trajectory_idx = static_cast<int>(phase * 49);          // Map 0.0-1.0 to 0-49
        trajectory_idx = std::min(49, std::max(0, trajectory_idx)); // Clamp to valid range

        // Get current and desired positions
        Eigen::Vector3d current_pos(
        controller.latest_state_.p1.x,
        controller.latest_state_.p1.y,
        controller.latest_state_.p1.z);

        Eigen::Vector3d target_pos(
        controller.foot_trajectories_.foot1[trajectory_idx].x,
        controller.foot_trajectories_.foot1[trajectory_idx].y,
        controller.foot_trajectories_.foot1[trajectory_idx].z);

        // Get current velocity (zeros if not available)
        Eigen::Vector3d current_vel = Eigen::Vector3d::Zero();
        if (controller.latest_state_.v1.x != 0.0 || 
        controller.latest_state_.v1.y != 0.0 || 
        controller.latest_state_.v1.z != 0.0) {
          // Only use velocity feedback if it's non-zero (available)
          current_vel = Eigen::Vector3d(
          controller.latest_state_.v1.x,
          controller.latest_state_.v1.y,
          controller.latest_state_.v1.z);
        }

        // Calculate position error and PD control force
        Eigen::Vector3d pos_error = target_pos - current_pos;
        Eigen::Vector3d pd_force = kp * pos_error - kd * current_vel;

        // Store the calculated swing force for this foot
        controller.swing_forces_.foot1.x = pd_force(0);
        controller.swing_forces_.foot1.y = pd_force(1);
        controller.swing_forces_.foot1.z = pd_force(2);
        controller.swing_forces_.has_data = true;

        // Store info for reference (formerly for tabular logging)
        foot_info[0].active = true;
        foot_info[0].phase = phase;
        foot_info[0].trajectory_idx = trajectory_idx;
        foot_info[0].target_pos = target_pos;
        foot_info[0].current_pos = current_pos;
        foot_info[0].pos_error = pos_error;
        foot_info[0].pd_force = pd_force;
      }

      // Foot 2 (Front right)
      if (controller.latest_gait_.foot2_state == 1)
      {
        RCLCPP_DEBUG(controller.get_node()->get_logger(),
              "Entered foot 2 pd control if statement");
        // Generate trajectory at the beginning of swing phase
        if (controller.latest_gait_.foot2_phase == 0.0)
        {
          RCLCPP_DEBUG(controller.get_node()->get_logger(), "Foot 2 (Front right) generating new trajectory");

          // Extract current position (p0) from the state message
          Eigen::Vector3d p0(
          controller.latest_state_.p2.x,
          controller.latest_state_.p2.y,
          controller.latest_state_.p2.z);

          // Create middle control point (p1): same x,y as hip but raised by step_height
          Eigen::Vector3d p1(
          controller.latest_state_.h2.x,
          controller.latest_state_.h2.y,
          controller.latest_state_.h2.z + step_height);

          // Target position (p2) from gait pattern
          Eigen::Vector3d p2(
          controller.latest_gait_.foot2_step_position.x,
          controller.latest_gait_.foot2_step_position.y,
          controller.latest_gait_.foot2_step_position.z);

          // Generate the trajectory using the helper function
          auto trajectory = generate_bezier_path(p0, p1, p2, 50);

          // Store the trajectory
          for (size_t i = 0; i < trajectory.size() && i < 50; ++i)
          {
        controller.foot_trajectories_.foot2[i].x = trajectory[i][0];
        controller.foot_trajectories_.foot2[i].y = trajectory[i][1];
        controller.foot_trajectories_.foot2[i].z = trajectory[i][2];
          }
          
          // Log the entire trajectory at debug level
          std::stringstream traj_ss;
          traj_ss << "Foot 2 trajectory points:";
          traj_ss << "\n  Hip Position: [" 
             << controller.latest_state_.h2.x << ", "
             << controller.latest_state_.h2.y << ", "
             << controller.latest_state_.h2.z << "]";
          traj_ss << "\n  Control Points:";
          traj_ss << "\n    p0 (start): [" << p0.x() << ", " << p0.y() << ", " << p0.z() << "]";
          traj_ss << "\n    p1 (mid)  : [" << p1.x() << ", " << p1.y() << ", " << p1.z() << "]";
          traj_ss << "\n    p2 (end)  : [" << p2.x() << ", " << p2.y() << ", " << p2.z() << "]";
          traj_ss << "\n  Trajectory:";
          for (size_t i = 0; i < std::min(trajectory.size(), static_cast<size_t>(50)); ++i) {
        traj_ss << "\n    [" << i << "]: [" 
           << controller.foot_trajectories_.foot2[i].x << ", "
           << controller.foot_trajectories_.foot2[i].y << ", "
           << controller.foot_trajectories_.foot2[i].z << "]";
          }
          RCLCPP_DEBUG(controller.get_node()->get_logger(), "%s\n\n", traj_ss.str().c_str());

          RCLCPP_DEBUG(controller.get_node()->get_logger(),
              "Generated trajectory for foot2: from %s through %s to %s",
              vec3_to_string(p0).c_str(), vec3_to_string(p1).c_str(), vec3_to_string(p2).c_str());
        }

        // Use phase to determine trajectory index (phase goes from 0.0 to 1.0)
        double phase = controller.latest_gait_.foot2_phase;
        int trajectory_idx = static_cast<int>(phase * 49);          // Map 0.0-1.0 to 0-49
        trajectory_idx = std::min(49, std::max(0, trajectory_idx)); // Clamp to valid range

        // Get current and desired positions
        Eigen::Vector3d current_pos(
        controller.latest_state_.p2.x,
        controller.latest_state_.p2.y,
        controller.latest_state_.p2.z);

        Eigen::Vector3d target_pos(
        controller.foot_trajectories_.foot2[trajectory_idx].x,
        controller.foot_trajectories_.foot2[trajectory_idx].y,
        controller.foot_trajectories_.foot2[trajectory_idx].z);

        // Get current velocity (zeros if not available)
        Eigen::Vector3d current_vel = Eigen::Vector3d::Zero();
        if (controller.latest_state_.v2.x != 0.0 || 
        controller.latest_state_.v2.y != 0.0 || 
        controller.latest_state_.v2.z != 0.0) {
          // Only use velocity feedback if it's non-zero (available)
          current_vel = Eigen::Vector3d(
          controller.latest_state_.v2.x,
          controller.latest_state_.v2.y,
          controller.latest_state_.v2.z);
        }

        // Calculate position error and PD control force
        Eigen::Vector3d pos_error = target_pos - current_pos;
        Eigen::Vector3d pd_force = kp * pos_error - kd * current_vel;

        // Store the calculated swing force for this foot
        controller.swing_forces_.foot2.x = pd_force(0);
        controller.swing_forces_.foot2.y = pd_force(1);
        controller.swing_forces_.foot2.z = pd_force(2);
        controller.swing_forces_.has_data = true;

        // Store info for reference (formerly for tabular logging)
        foot_info[1].active = true;
        foot_info[1].phase = phase;
        foot_info[1].trajectory_idx = trajectory_idx;
        foot_info[1].target_pos = target_pos;
        foot_info[1].current_pos = current_pos;
        foot_info[1].pos_error = pos_error;
        foot_info[1].pd_force = pd_force;
      }

      // Foot 3 (Rear left)
      if (controller.latest_gait_.foot3_state == 1)
      {
        RCLCPP_DEBUG(controller.get_node()->get_logger(),
              "Entered foot 3 pd control if statement");
        // Generate trajectory at the beginning of swing phase
        if (controller.latest_gait_.foot3_phase == 0.0)
        {
          RCLCPP_DEBUG(controller.get_node()->get_logger(), "Foot 3 (Rear left) generating new trajectory");

          // Extract current position (p0) from the state message
          Eigen::Vector3d p0(
          controller.latest_state_.p3.x,
          controller.latest_state_.p3.y,
          controller.latest_state_.p3.z);

          // Create middle control point (p1): same x,y as hip but raised by step_height
          Eigen::Vector3d p1(
          controller.latest_state_.h3.x,
          controller.latest_state_.h3.y,
          controller.latest_state_.h3.z + step_height);

          // Target position (p2) from gait pattern
          Eigen::Vector3d p2(
          controller.latest_gait_.foot3_step_position.x,
          controller.latest_gait_.foot3_step_position.y,
          controller.latest_gait_.foot3_step_position.z);

          // Generate the trajectory using the helper function
          auto trajectory = generate_bezier_path(p0, p1, p2, 50);

          // Store the trajectory
          for (size_t i = 0; i < trajectory.size() && i < 50; ++i)
          {
        controller.foot_trajectories_.foot3[i].x = trajectory[i][0];
        controller.foot_trajectories_.foot3[i].y = trajectory[i][1];
        controller.foot_trajectories_.foot3[i].z = trajectory[i][2];
          }
          
          // Log the entire trajectory at debug level
          std::stringstream traj_ss;
          traj_ss << "Foot 3 trajectory points:";
          traj_ss << "\n  Hip Position: [" 
             << controller.latest_state_.h3.x << ", "
             << controller.latest_state_.h3.y << ", "
             << controller.latest_state_.h3.z << "]";
          traj_ss << "\n  Control Points:";
          traj_ss << "\n    p0 (start): [" << p0.x() << ", " << p0.y() << ", " << p0.z() << "]";
          traj_ss << "\n    p1 (mid)  : [" << p1.x() << ", " << p1.y() << ", " << p1.z() << "]";
          traj_ss << "\n    p2 (end)  : [" << p2.x() << ", " << p2.y() << ", " << p2.z() << "]";
          traj_ss << "\n  Trajectory:";
          for (size_t i = 0; i < std::min(trajectory.size(), static_cast<size_t>(50)); ++i) {
        traj_ss << "\n    [" << i << "]: [" 
           << controller.foot_trajectories_.foot3[i].x << ", "
           << controller.foot_trajectories_.foot3[i].y << ", "
           << controller.foot_trajectories_.foot3[i].z << "]";
          }
          RCLCPP_DEBUG(controller.get_node()->get_logger(), "%s\n\n", traj_ss.str().c_str());

          RCLCPP_DEBUG(controller.get_node()->get_logger(),
              "Generated trajectory for foot3: from %s through %s to %s",
              vec3_to_string(p0).c_str(), vec3_to_string(p1).c_str(), vec3_to_string(p2).c_str());
        }

        // Use phase to determine trajectory index (phase goes from 0.0 to 1.0)
        double phase = controller.latest_gait_.foot3_phase;
        int trajectory_idx = static_cast<int>(phase * 49);          // Map 0.0-1.0 to 0-49
        trajectory_idx = std::min(49, std::max(0, trajectory_idx)); // Clamp to valid range

        // Get current and desired positions
        Eigen::Vector3d current_pos(
        controller.latest_state_.p3.x,
        controller.latest_state_.p3.y,
        controller.latest_state_.p3.z);

        Eigen::Vector3d target_pos(
        controller.foot_trajectories_.foot3[trajectory_idx].x,
        controller.foot_trajectories_.foot3[trajectory_idx].y,
        controller.foot_trajectories_.foot3[trajectory_idx].z);

        // Get current velocity (zeros if not available)
        Eigen::Vector3d current_vel = Eigen::Vector3d::Zero();
        if (controller.latest_state_.v3.x != 0.0 || 
        controller.latest_state_.v3.y != 0.0 || 
        controller.latest_state_.v3.z != 0.0) {
          // Only use velocity feedback if it's non-zero (available)
          current_vel = Eigen::Vector3d(
          controller.latest_state_.v3.x,
          controller.latest_state_.v3.y,
          controller.latest_state_.v3.z);
        }

        // Calculate position error and PD control force
        Eigen::Vector3d pos_error = target_pos - current_pos;
        Eigen::Vector3d pd_force = kp * pos_error - kd * current_vel;

        // Store the calculated swing force for this foot
        controller.swing_forces_.foot3.x = pd_force(0);
        controller.swing_forces_.foot3.y = pd_force(1);
        controller.swing_forces_.foot3.z = pd_force(2);
        controller.swing_forces_.has_data = true;

        // Store info for reference (formerly for tabular logging)
        foot_info[2].active = true;
        foot_info[2].phase = phase;
        foot_info[2].trajectory_idx = trajectory_idx;
        foot_info[2].target_pos = target_pos;
        foot_info[2].current_pos = current_pos;
        foot_info[2].pos_error = pos_error;
        foot_info[2].pd_force = pd_force;
      }

      // Foot 4 (Rear right)
      if (controller.latest_gait_.foot4_state == 1)
      {
        RCLCPP_DEBUG(controller.get_node()->get_logger(),
              "Entered foot 4 pd control if statement");
        // Generate trajectory at the beginning of swing phase
        if (controller.latest_gait_.foot4_phase == 0.0)
        {
          RCLCPP_DEBUG(controller.get_node()->get_logger(), "Foot 4 (Rear right) generating new trajectory");

          // Extract current position (p0) from the state message
          Eigen::Vector3d p0(
          controller.latest_state_.p4.x,
          controller.latest_state_.p4.y,
          controller.latest_state_.p4.z);

          // Create middle control point (p1): same x,y as hip but raised by step_height
          Eigen::Vector3d p1(
          controller.latest_state_.h4.x,
          controller.latest_state_.h4.y,
          controller.latest_state_.h4.z + step_height);

          // Target position (p2) from gait pattern
          Eigen::Vector3d p2(
          controller.latest_gait_.foot4_step_position.x,
          controller.latest_gait_.foot4_step_position.y,
          controller.latest_gait_.foot4_step_position.z);

          // Generate the trajectory using the helper function
          auto trajectory = generate_bezier_path(p0, p1, p2, 50);

          // Store the trajectory
          for (size_t i = 0; i < trajectory.size() && i < 50; ++i)
          {
        controller.foot_trajectories_.foot4[i].x = trajectory[i][0];
        controller.foot_trajectories_.foot4[i].y = trajectory[i][1];
        controller.foot_trajectories_.foot4[i].z = trajectory[i][2];
          }
          
          // Log the entire trajectory at debug level
          std::stringstream traj_ss;
          traj_ss << "Foot 4 trajectory points:";
          traj_ss << "\n  Hip Position: [" 
             << controller.latest_state_.h4.x << ", "
             << controller.latest_state_.h4.y << ", "
             << controller.latest_state_.h4.z << "]";
          traj_ss << "\n  Control Points:";
          traj_ss << "\n    p0 (start): [" << p0.x() << ", " << p0.y() << ", " << p0.z() << "]";
          traj_ss << "\n    p1 (mid)  : [" << p1.x() << ", " << p1.y() << ", " << p1.z() << "]";
          traj_ss << "\n    p2 (end)  : [" << p2.x() << ", " << p2.y() << ", " << p2.z() << "]";
          traj_ss << "\n  Trajectory:";
          for (size_t i = 0; i < std::min(trajectory.size(), static_cast<size_t>(50)); ++i) {
        traj_ss << "\n    [" << i << "]: [" 
           << controller.foot_trajectories_.foot4[i].x << ", "
           << controller.foot_trajectories_.foot4[i].y << ", "
           << controller.foot_trajectories_.foot4[i].z << "]";
          }
          RCLCPP_DEBUG(controller.get_node()->get_logger(), "%s\n\n", traj_ss.str().c_str());

          RCLCPP_DEBUG(controller.get_node()->get_logger(),
              "Generated trajectory for foot4: from %s through %s to %s",
              vec3_to_string(p0).c_str(), vec3_to_string(p1).c_str(), vec3_to_string(p2).c_str());
        }

        // Use phase to determine trajectory index (phase goes from 0.0 to 1.0)
        double phase = controller.latest_gait_.foot4_phase;
        int trajectory_idx = static_cast<int>(phase * 49);          // Map 0.0-1.0 to 0-49
        trajectory_idx = std::min(49, std::max(0, trajectory_idx)); // Clamp to valid range

        // Get current and desired positions
        Eigen::Vector3d current_pos(
        controller.latest_state_.p4.x,
        controller.latest_state_.p4.y,
        controller.latest_state_.p4.z);

        Eigen::Vector3d target_pos(
        controller.foot_trajectories_.foot4[trajectory_idx].x,
        controller.foot_trajectories_.foot4[trajectory_idx].y,
        controller.foot_trajectories_.foot4[trajectory_idx].z);

        // Get current velocity (zeros if not available)
        Eigen::Vector3d current_vel = Eigen::Vector3d::Zero();
        if (controller.latest_state_.v4.x != 0.0 || 
        controller.latest_state_.v4.y != 0.0 || 
        controller.latest_state_.v4.z != 0.0) {
          // Only use velocity feedback if it's non-zero (available)
          current_vel = Eigen::Vector3d(
          controller.latest_state_.v4.x,
          controller.latest_state_.v4.y,
          controller.latest_state_.v4.z);
        }

        // Calculate position error and PD control force
        Eigen::Vector3d pos_error = target_pos - current_pos;
        Eigen::Vector3d pd_force = kp * pos_error - kd * current_vel;

        // Store the calculated swing force for this foot
        controller.swing_forces_.foot4.x = pd_force(0);
        controller.swing_forces_.foot4.y = pd_force(1);
        controller.swing_forces_.foot4.z = pd_force(2);
        controller.swing_forces_.has_data = true;

        // Store info for reference (formerly for tabular logging)
        foot_info[3].active = true;
        foot_info[3].phase = phase;
        foot_info[3].trajectory_idx = trajectory_idx;
        foot_info[3].target_pos = target_pos;
        foot_info[3].current_pos = current_pos;
        foot_info[3].pos_error = pos_error;
        foot_info[3].pd_force = pd_force;
      }
      
      // Periodic overall debug
      static int swing_counter = 0;
      if ((swing_counter++ % 500) == 0)
      {
        RCLCPP_DEBUG(controller.get_node()->get_logger(),
             "Swing trajectory states: F1=%d(%.2f), F2=%d(%.2f), F3=%d(%.2f), F4=%d(%.2f)",
             controller.latest_gait_.foot1_state, controller.latest_gait_.foot1_phase,
             controller.latest_gait_.foot2_state, controller.latest_gait_.foot2_phase,
             controller.latest_gait_.foot3_state, controller.latest_gait_.foot3_phase,
             controller.latest_gait_.foot4_state, controller.latest_gait_.foot4_phase);
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

      bool jacobians_empty = controller.latest_state_.j1.empty() ||
                             controller.latest_state_.j2.empty() ||
                             controller.latest_state_.j3.empty() ||
                             controller.latest_state_.j4.empty();

      bool jacobians_wrong_size = controller.latest_state_.j1.size() < 6 ||
                                  controller.latest_state_.j2.size() < 6 ||
                                  controller.latest_state_.j3.size() < 6 ||
                                  controller.latest_state_.j4.size() < 6;

      if (jacobians_empty || jacobians_wrong_size)
      {
        static int jacobian_error_counter = 0;
        if ((jacobian_error_counter++ % 100) == 0)
        {
          if (jacobians_empty)
          {
            RCLCPP_WARN(controller.get_node()->get_logger(),
                        "Jacobian data incomplete: j1=%s, j2=%s, j3=%s, j4=%s",
                        controller.latest_state_.j1.empty() ? "empty" : "has data",
                        controller.latest_state_.j2.empty() ? "empty" : "has data",
                        controller.latest_state_.j3.empty() ? "empty" : "has data",
                        controller.latest_state_.j4.empty() ? "empty" : "has data");
          }

          if (jacobians_wrong_size)
          {
            RCLCPP_WARN(controller.get_node()->get_logger(),
                        "Jacobian arrays wrong size: j1=%zu, j2=%zu, j3=%zu, j4=%zu (expected 6 each)",
                        controller.latest_state_.j1.size(), controller.latest_state_.j2.size(),
                        controller.latest_state_.j3.size(), controller.latest_state_.j4.size());
          }
        }

        for (auto &interface : controller.joint_command_interfaces_)
        {
          interface.set_value(0.0);
        }
        return true;
      }

      Eigen::Vector3d force1(
          controller.foot_forces_.foot1.x,
          controller.foot_forces_.foot1.y,
          controller.foot_forces_.foot1.z);

      Eigen::Vector3d force2(
          controller.foot_forces_.foot2.x,
          controller.foot_forces_.foot2.y,
          controller.foot_forces_.foot2.z);

      Eigen::Vector3d force3(
          controller.foot_forces_.foot3.x,
          controller.foot_forces_.foot3.y,
          controller.foot_forces_.foot3.z);

      Eigen::Vector3d force4(
          controller.foot_forces_.foot4.x,
          controller.foot_forces_.foot4.y,
          controller.foot_forces_.foot4.z);

      // Create a static counter for less frequent logging (every 10 iterations like balance controller)
      static int log_counter = 0;
      bool detailed_log = ((log_counter++ % 10) == 0);
      
      // Log forces at the same frequency as balance controller
      if (detailed_log) {
        RCLCPP_DEBUG(controller.get_node()->get_logger(), 
                    "FOOT FORCES: FL=[%.2f, %.2f, %.2f] FR=[%.2f, %.2f, %.2f] RL=[%.2f, %.2f, %.2f] RR=[%.2f, %.2f, %.2f]",
                    force1(0), force1(1), force1(2),
                    force2(0), force2(1), force2(2),
                    force3(0), force3(1), force3(2),
                    force4(0), force4(1), force4(2));

        // Explicitly log each Jacobian element for validation, but only at the controlled rate
        RCLCPP_DEBUG(controller.get_node()->get_logger(), 
                    "FL Jacobian [raw]: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    controller.latest_state_.j1[0], controller.latest_state_.j1[1], controller.latest_state_.j1[2],
                    controller.latest_state_.j1[3], controller.latest_state_.j1[4], controller.latest_state_.j1[5]);
        RCLCPP_DEBUG(controller.get_node()->get_logger(), 
                    "FR Jacobian [raw]: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    controller.latest_state_.j2[0], controller.latest_state_.j2[1], controller.latest_state_.j2[2],
                    controller.latest_state_.j2[3], controller.latest_state_.j2[4], controller.latest_state_.j2[5]);
        RCLCPP_DEBUG(controller.get_node()->get_logger(), 
                    "RL Jacobian [raw]: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    controller.latest_state_.j3[0], controller.latest_state_.j3[1], controller.latest_state_.j3[2],
                    controller.latest_state_.j3[3], controller.latest_state_.j3[4], controller.latest_state_.j3[5]);
        RCLCPP_DEBUG(controller.get_node()->get_logger(), 
                    "RR Jacobian [raw]: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    controller.latest_state_.j4[0], controller.latest_state_.j4[1], controller.latest_state_.j4[2],
                    controller.latest_state_.j4[3], controller.latest_state_.j4[4], controller.latest_state_.j4[5]);
      }

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

      // Construct the Jacobian matrices
      Eigen::Matrix<double, 3, 2> J1, J2, J3, J4;

      J1 << controller.latest_state_.j1[0], controller.latest_state_.j1[3],
          controller.latest_state_.j1[1], controller.latest_state_.j1[4],
          controller.latest_state_.j1[2], controller.latest_state_.j1[5];

      J2 << controller.latest_state_.j2[0], controller.latest_state_.j2[3],
          controller.latest_state_.j2[1], controller.latest_state_.j2[4],
          controller.latest_state_.j2[2], controller.latest_state_.j2[5];

      J3 << controller.latest_state_.j3[0], controller.latest_state_.j3[3],
          controller.latest_state_.j3[1], controller.latest_state_.j3[4],
          controller.latest_state_.j3[2], controller.latest_state_.j3[5];

      J4 << controller.latest_state_.j4[0], controller.latest_state_.j4[3],
          controller.latest_state_.j4[1], controller.latest_state_.j4[4],
          controller.latest_state_.j4[2], controller.latest_state_.j4[5];

      // Calculate torques using the negative of forces (action-reaction principle)
      Eigen::Vector2d torque1 = J1.transpose() * force1;
      Eigen::Vector2d torque2 = J2.transpose() * force2;
      Eigen::Vector2d torque3 = J3.transpose() * force3;
      Eigen::Vector2d torque4 = J4.transpose() * force4;

      // Create a table to log the current state
      if (detailed_log) {
        std::stringstream desired_state_table;
        desired_state_table << "\nCurrent Body State:";
        desired_state_table << "\n╔═════╦══════════════════════════╦══════════════════════════════════╦══════════════════════════╦══════════════════════════╗";
        desired_state_table << "\n║STAGE║     POSITION (XYZ)       ║     ORIENTATION (WXYZ)           ║    COM VEL (XYZ)         ║    ANGULAR VEL (XYZ)     ║";
        desired_state_table << "\n╠═════╬══════════════════════════╬══════════════════════════════════╬══════════════════════════╬══════════════════════════╣";
        
        // Get current state from latest state using fields that exist in the message
        Eigen::Vector3d position(
            controller.latest_state_.pc.x,
            controller.latest_state_.pc.y,
            controller.latest_state_.pc.z);
        
        Eigen::Vector4d orientation(
            controller.latest_state_.orientation.w,
            controller.latest_state_.orientation.x,
            controller.latest_state_.orientation.y,
            controller.latest_state_.orientation.z);
        
        // Use COM velocity (center of mass velocity)
        Eigen::Vector3d com_velocity(
            controller.latest_state_.com_velocity.x,
            controller.latest_state_.com_velocity.y,
            controller.latest_state_.com_velocity.z);
        
        // Use IMU angular velocity
        Eigen::Vector3d angular_velocity(
            controller.latest_state_.angular_velocity.x,
            controller.latest_state_.angular_velocity.y,
            controller.latest_state_.angular_velocity.z);
        
        // Format the state data for the table
        std::stringstream pos_ss, quat_ss, lin_vel_ss, ang_vel_ss;
        
        pos_ss << std::fixed << std::setprecision(3);
        pos_ss << "[" << std::setw(6) << position.x() 
               << "," << std::setw(6) << position.y() 
               << "," << std::setw(6) << position.z() << "]";
        
        quat_ss << std::fixed << std::setprecision(3);
        quat_ss << "[" << std::setw(6) << orientation.w() 
                << "," << std::setw(6) << orientation.x() 
                << "," << std::setw(6) << orientation.y() 
                << "," << std::setw(6) << orientation.z() << "]";
        
        lin_vel_ss << std::fixed << std::setprecision(3);
        lin_vel_ss << "[" << std::setw(6) << com_velocity.x() 
                  << "," << std::setw(6) << com_velocity.y() 
                  << "," << std::setw(6) << com_velocity.z() << "]";
        
        ang_vel_ss << std::fixed << std::setprecision(3);
        ang_vel_ss << "[" << std::setw(6) << angular_velocity.x() 
                  << "," << std::setw(6) << angular_velocity.y() 
                  << "," << std::setw(6) << angular_velocity.z() << "]";
        
        // Add a single row with current state
        desired_state_table << "\n║   0 ║ " << std::setw(26) << pos_ss.str() 
                           << " ║ " << std::setw(30) << quat_ss.str() 
                           << " ║ " << std::setw(26) << lin_vel_ss.str() 
                           << " ║ " << std::setw(26) << ang_vel_ss.str() << " ║";
        
        // Add closing line for the table
        desired_state_table << "\n╚═════╩══════════════════════════╩══════════════════════════════════╩══════════════════════════╩══════════════════════════╝";
        
        RCLCPP_INFO(controller.get_node()->get_logger(), "Current Body State: %s", desired_state_table.str().c_str());
      }

      // Check if any torque exceeds the limit (8 Nm)
      const double torque_limit = 8.0;  // 8 Newton-meters
      
      // Check and warn for torque1
      if (std::abs(torque1(0)) >= torque_limit || std::abs(torque1(1)) >= torque_limit) {
        RCLCPP_WARN(controller.get_node()->get_logger(), 
                   "FL torque exceeds limit: [%.2f, %.2f] Nm (limit: %.2f Nm)",
                   torque1(0), torque1(1), torque_limit);
      }
      
      // Check and warn for torque2
      if (std::abs(torque2(0)) >= torque_limit || std::abs(torque2(1)) >= torque_limit) {
        RCLCPP_WARN(controller.get_node()->get_logger(), 
                   "FR torque exceeds limit: [%.2f, %.2f] Nm (limit: %.2f Nm)",
                   torque2(0), torque2(1), torque_limit);
      }
      
      // Check and warn for torque3
      if (std::abs(torque3(0)) >= torque_limit || std::abs(torque3(1)) >= torque_limit) {
        RCLCPP_WARN(controller.get_node()->get_logger(), 
                   "RL torque exceeds limit: [%.2f, %.2f] Nm (limit: %.2f Nm)",
                   torque3(0), torque3(1), torque_limit);
      }
      
      // Check and warn for torque4
      if (std::abs(torque4(0)) >= torque_limit || std::abs(torque4(1)) >= torque_limit) {
        RCLCPP_WARN(controller.get_node()->get_logger(), 
                   "RR torque exceeds limit: [%.2f, %.2f] Nm (limit: %.2f Nm)",
                   torque4(0), torque4(1), torque_limit);
      }

      // Log the computed torques to verify correct calculation
      if (detailed_log) {
        RCLCPP_DEBUG(controller.get_node()->get_logger(), 
                    "COMPUTED TORQUES: FL=[%.2f, %.2f] FR=[%.2f, %.2f] RL=[%.2f, %.2f] RR=[%.2f, %.2f]",
                    torque1(0), torque1(1),
                    torque2(0), torque2(1),
                    torque3(0), torque3(1),
                    torque4(0), torque4(1));
      }

      // Set torque values to joint interfaces
      controller.joint_command_interfaces_[0].set_value(torque1[0]);
      controller.joint_command_interfaces_[1].set_value(torque1[1]);

      controller.joint_command_interfaces_[2].set_value(torque2[0]);
      controller.joint_command_interfaces_[3].set_value(torque2[1]);

      controller.joint_command_interfaces_[4].set_value(torque3[0]);
      controller.joint_command_interfaces_[5].set_value(torque3[1]);

      controller.joint_command_interfaces_[6].set_value(torque4[0]);
      controller.joint_command_interfaces_[7].set_value(torque4[1]);

      // Log the actual interface values after they've been set
      if (detailed_log) {
        RCLCPP_DEBUG(controller.get_node()->get_logger(), 
                    "INTERFACE VALUES: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    controller.joint_command_interfaces_[0].get_value(),
                    controller.joint_command_interfaces_[1].get_value(),
                    controller.joint_command_interfaces_[2].get_value(),
                    controller.joint_command_interfaces_[3].get_value(),
                    controller.joint_command_interfaces_[4].get_value(),
                    controller.joint_command_interfaces_[5].get_value(),
                    controller.joint_command_interfaces_[6].get_value(),
                    controller.joint_command_interfaces_[7].get_value());

        // Log interface names to confirm the correct ordering
        std::stringstream ss;
        ss << "\nInterface to Joint Mapping:";
        for (size_t i = 0; i < controller.joint_command_interfaces_.size(); ++i) {
          ss << "\n  [" << i << "] " << controller.joint_command_interfaces_[i].get_name() 
             << " = " << controller.joint_command_interfaces_[i].get_value();
        }
        RCLCPP_DEBUG(controller.get_node()->get_logger(), "%s", ss.str().c_str());

        ss.str("");
        ss << "\nDetailed Jacobian Transformations:";
        ss << "\n  Foot 1 (Front Left - fl_hip, fl_knee):" << visualize_force_torque_mapping(force1, torque1, J1);
        ss << "\n  Foot 2 (Front Right - fr_hip, fr_knee):" << visualize_force_torque_mapping(force2, torque2, J2);
        ss << "\n  Foot 3 (Rear Left - rl_hip, rl_knee):" << visualize_force_torque_mapping(force3, torque3, J3);
        ss << "\n  Foot 4 (Rear Right - rr_hip, rr_knee):" << visualize_force_torque_mapping(force4, torque4, J4);
        RCLCPP_DEBUG(controller.get_node()->get_logger(), "%s", ss.str().c_str());
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
