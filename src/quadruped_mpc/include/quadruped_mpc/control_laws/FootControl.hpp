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
      const Eigen::Vector3d &force,
      const Eigen::Vector2d &torque,
      const Eigen::Matrix<double, 3, 2> &jacobian)
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
                       prev_j1_size, prev_j2_size, prev_j3_size, prev_j4_size);
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
      controller.foot_forces_.foot1 = geometry_msgs::msg::Vector3();
      controller.foot_forces_.foot2 = geometry_msgs::msg::Vector3();
      controller.foot_forces_.foot3 = geometry_msgs::msg::Vector3();
      controller.foot_forces_.foot4 = geometry_msgs::msg::Vector3();
      controller.foot_forces_.has_data = false;

      // Acquire lock for accessing gait data
      std::lock_guard<std::mutex> gait_lock(controller.gait_mutex_);

      // If we have forces data from balance controller, update our local foot forces
      if (foot_forces_msg)
      {
        // Set flag that we have data and copy forces
        controller.foot_forces_.has_data = true;
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

      // Log all relevant condition values in debug level
      RCLCPP_DEBUG(controller.get_node()->get_logger(),
                   "Gait and swing conditions: swing_forces.has_data=%d, "
                   "foot1_state=%d, foot2_state=%d, foot3_state=%d, foot4_state=%d",
                   controller.swing_forces_.has_data,
                   controller.latest_gait_.foot1_state,
                   controller.latest_gait_.foot2_state,
                   controller.latest_gait_.foot3_state,
                   controller.latest_gait_.foot4_state);

      // Array to organize foot data access
      struct FootGaitInfo
      {
        int *state;
        geometry_msgs::msg::Vector3 *swing_force;
        geometry_msgs::msg::Vector3 *foot_force;
        const char *name;
      };

      // Map foot data for easier access
      FootGaitInfo feet[4] = {
          {&controller.latest_gait_.foot1_state, &controller.swing_forces_.foot1, &controller.foot_forces_.foot1, "Foot 1"},
          {&controller.latest_gait_.foot2_state, &controller.swing_forces_.foot2, &controller.foot_forces_.foot2, "Foot 2"},
          {&controller.latest_gait_.foot3_state, &controller.swing_forces_.foot3, &controller.foot_forces_.foot3, "Foot 3"},
          {&controller.latest_gait_.foot4_state, &controller.swing_forces_.foot4, &controller.foot_forces_.foot4, "Foot 4"}};

      // Process each foot for swing forces
      for (int i = 0; i < 4; i++)
      {
        if (*feet[i].state == 1 && controller.swing_forces_.has_data)
        {
          // Apply swing force
          *feet[i].foot_force = *feet[i].swing_force;
          RCLCPP_DEBUG(controller.get_node()->get_logger(),
                       "%s using swing force: [%.2f, %.2f, %.2f]",
                       feet[i].name,
                       feet[i].swing_force->x,
                       feet[i].swing_force->y,
                       feet[i].swing_force->z);
        }
        else
        {
          RCLCPP_DEBUG(controller.get_node()->get_logger(),
                       "%s not using swing force: state=%d, has_data=%d",
                       feet[i].name, *feet[i].state, controller.swing_forces_.has_data);
        }
      }

      // Generate consolidated force table with all feet (keeping original table format)
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
          controller.latest_gait_.foot4_state};
      const std::array<double, 4> foot_phases = {
          controller.latest_gait_.foot1_phase,
          controller.latest_gait_.foot2_phase,
          controller.latest_gait_.foot3_phase,
          controller.latest_gait_.foot4_phase};

      // Access current positions
      const std::array<Eigen::Vector3d, 4> current_positions = {
          Eigen::Vector3d(controller.latest_state_.p1.x, controller.latest_state_.p1.y, controller.latest_state_.p1.z),
          Eigen::Vector3d(controller.latest_state_.p2.x, controller.latest_state_.p2.y, controller.latest_state_.p2.z),
          Eigen::Vector3d(controller.latest_state_.p3.x, controller.latest_state_.p3.y, controller.latest_state_.p3.z),
          Eigen::Vector3d(controller.latest_state_.p4.x, controller.latest_state_.p4.y, controller.latest_state_.p4.z)};

      // Function to get trajectory data for a specific foot
      auto get_foot_trajectory = [&controller](int idx, int foot_num)
      {
        idx = std::min(49, std::max(0, idx));
        const auto &traj = (foot_num == 0) ? controller.foot_trajectories_.foot1 : (foot_num == 1) ? controller.foot_trajectories_.foot2
                                                                               : (foot_num == 2)   ? controller.foot_trajectories_.foot3
                                                                                                   : controller.foot_trajectories_.foot4;
        return Eigen::Vector3d(traj[idx].x, traj[idx].y, traj[idx].z);
      };

      // Process trajectory data and generate the table rows
      std::array<Eigen::Vector3d, 4> forces_eigen = {
          Eigen::Vector3d(controller.foot_forces_.foot1.x, controller.foot_forces_.foot1.y, controller.foot_forces_.foot1.z),
          Eigen::Vector3d(controller.foot_forces_.foot2.x, controller.foot_forces_.foot2.y, controller.foot_forces_.foot2.z),
          Eigen::Vector3d(controller.foot_forces_.foot3.x, controller.foot_forces_.foot3.y, controller.foot_forces_.foot3.z),
          Eigen::Vector3d(controller.foot_forces_.foot4.x, controller.foot_forces_.foot4.y, controller.foot_forces_.foot4.z)};

      // Calculate total stance force and generate table
      Eigen::Vector3d total_stance_force = Eigen::Vector3d::Zero();
      int stance_foot_count = 0;

      for (int i = 0; i < 4; ++i)
      {
        // Determine if foot is in swing phase and has valid trajectory
        bool is_swing = foot_states[i] == 1 && foot_phases[i] >= 0.0 && foot_phases[i] <= 1.0;

        // Get target/trajectory data
        Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d end_pos = Eigen::Vector3d::Zero();
        if (is_swing)
        {
          int trajectory_idx = static_cast<int>(foot_phases[i] * 49);
          target_pos = get_foot_trajectory(trajectory_idx, i);
          end_pos = get_foot_trajectory(49, i); // Endpoint
        }
        else
        {
          // Track stance feet for total force
          total_stance_force += forces_eigen[i];
          stance_foot_count++;
        }

        // Format all position data for display
        std::string state_str = is_swing ? "SWING " : "STANCE";

        // Format position vectors with consistent width
        std::stringstream target_ss, current_ss, endpoint_ss, force_ss;

        // Format target position
        target_ss << std::fixed << std::setprecision(3);
        if (is_swing)
        {
          target_ss << "["
                    << std::setw(7) << target_pos.x() << ","
                    << std::setw(7) << target_pos.y() << ","
                    << std::setw(7) << target_pos.z() << "]";
        }
        else
        {
          target_ss << "[      N/A              ]";
        }

        // Format current position
        current_ss << std::fixed << std::setprecision(3);
        current_ss << "["
                   << std::setw(7) << current_positions[i].x() << ","
                   << std::setw(7) << current_positions[i].y() << ","
                   << std::setw(7) << current_positions[i].z() << "]";

        // Format trajectory endpoint
        endpoint_ss << std::fixed << std::setprecision(3);
        if (is_swing)
        {
          endpoint_ss << "["
                      << std::setw(7) << end_pos.x() << ","
                      << std::setw(7) << end_pos.y() << ","
                      << std::setw(7) << end_pos.z() << "]";
        }
        else
        {
          endpoint_ss << "[      N/A              ]";
        }

        // Format force data
        force_ss << std::fixed << std::setprecision(2);
        force_ss << "["
                 << std::setw(7) << forces_eigen[i].x() << ","
                 << std::setw(7) << forces_eigen[i].y() << ","
                 << std::setw(7) << forces_eigen[i].z() << "]";

        // Add the formatted row to the table
        table << "\n║ " << std::left << std::setw(3) << foot_names[i] << " ║"
              << std::setw(6) << state_str << "║"
              << std::fixed << std::setprecision(2) << std::right << std::setw(7) << foot_phases[i] << "║"
              << std::setw(22) << target_ss.str() << " ║"
              << std::setw(22) << current_ss.str() << " ║"
              << std::setw(22) << endpoint_ss.str() << " ║"
              << std::setw(22) << force_ss.str() << " ║";
      }

      // Add separator line and total stance force row
      table << "\n╠═════╩══════╩═══════╩══════════════════════════╩══════════════════════════╩══════════════════════════╬══════════════════════════╣";

      // Format the total stance force
      std::stringstream total_force_ss;
      total_force_ss << std::fixed << std::setprecision(2);
      total_force_ss << "["
                     << std::setw(7) << total_stance_force.x() << ","
                     << std::setw(7) << total_stance_force.y() << ","
                     << std::setw(7) << total_stance_force.z() << "]";

      // Add total stance force row
      table << "\n║ TOTAL STANCE FORCE (" << stance_foot_count << " feet)                                                                         ║ "
            << std::setw(22) << total_force_ss.str() << "║";

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

      // Default step height and PD control gains
      const double step_height = 0.05; // 5cm step height
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
      auto generate_trajectory = [&](const FootData &foot)
      {
        Eigen::Vector3d p0 = foot.current_pos;
        Eigen::Vector3d p1(foot.hip_pos.x(), foot.hip_pos.y(), foot.hip_pos.z() + step_height);
        auto trajectory = generate_bezier_path(p0, p1, foot.target_pos, 50);

        // Store the trajectory
        for (size_t i = 0; i < trajectory.size() && i < 50; ++i)
        {
          foot.trajectories[i].x = trajectory[i][0];
          foot.trajectories[i].y = trajectory[i][1];
          foot.trajectories[i].z = trajectory[i][2];
        }

        // Log trajectory details at debug level
        RCLCPP_DEBUG(controller.get_node()->get_logger(),
                     "Generated trajectory for %s: from %s through %s to %s",
                     foot.name, vec3_to_string(p0).c_str(), vec3_to_string(p1).c_str(),
                     vec3_to_string(foot.target_pos).c_str());

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
          generate_trajectory(feet[i]);
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

      // Check for valid Jacobian data
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

      // Create arrays for forces and matrices for more compact code
      std::array<Eigen::Vector3d, 4> forces = {
          Eigen::Vector3d(controller.foot_forces_.foot1.x, controller.foot_forces_.foot1.y, controller.foot_forces_.foot1.z),
          Eigen::Vector3d(controller.foot_forces_.foot2.x, controller.foot_forces_.foot2.y, controller.foot_forces_.foot2.z),
          Eigen::Vector3d(controller.foot_forces_.foot3.x, controller.foot_forces_.foot3.y, controller.foot_forces_.foot3.z),
          Eigen::Vector3d(controller.foot_forces_.foot4.x, controller.foot_forces_.foot4.y, controller.foot_forces_.foot4.z)};

      std::array<Eigen::Matrix<double, 3, 2>, 4> jacobians;

      // Create the jacobian matrices
      jacobians[0] << controller.latest_state_.j1[0], controller.latest_state_.j1[3],
          controller.latest_state_.j1[1], controller.latest_state_.j1[4],
          controller.latest_state_.j1[2], controller.latest_state_.j1[5];

      jacobians[1] << controller.latest_state_.j2[0], controller.latest_state_.j2[3],
          controller.latest_state_.j2[1], controller.latest_state_.j2[4],
          controller.latest_state_.j2[2], controller.latest_state_.j2[5];

      jacobians[2] << controller.latest_state_.j3[0], controller.latest_state_.j3[3],
          controller.latest_state_.j3[1], controller.latest_state_.j3[4],
          controller.latest_state_.j3[2], controller.latest_state_.j3[5];

      jacobians[3] << controller.latest_state_.j4[0], controller.latest_state_.j4[3],
          controller.latest_state_.j4[1], controller.latest_state_.j4[4],
          controller.latest_state_.j4[2], controller.latest_state_.j4[5];

      // Calculate total vertical force before any transformations
      double total_vertical_force = 0.0;
      for (int i = 0; i < 4; i++) {
        total_vertical_force += forces[i].z();
      }

      // Create a log string for detailed force and torque analysis
      std::stringstream force_debug;
      force_debug << "\n===== FORCE TO TORQUE ANALYSIS =====";
      force_debug << "\nTOTAL VERTICAL FORCE: " << total_vertical_force << " N";
      force_debug << "\nExpected force for weight balance: " << (14.65 * 9.81) << " N"; // Robot mass * gravity

      // Compute torques and log the calculation
      std::array<Eigen::Vector2d, 4> torques;
      std::array<const char*, 4> leg_names = {"FL", "FR", "RL", "RR"};
      
      // Also calculate power to see if energy conservation is violated
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
        
        // Log the Jacobian matrix details for this leg
        force_debug << "\n\n" << leg_names[i] << " LEG:";
        force_debug << "\n  Applied Force: [" << forces[i].x() << ", " << forces[i].y() << ", " << forces[i].z() << "] N";
        force_debug << "\n  Jacobian Matrix:";
        force_debug << "\n    [" << std::fixed << std::setprecision(4) << jacobians[i](0,0) << ", " << jacobians[i](0,1) << "]";
        force_debug << "\n    [" << std::fixed << std::setprecision(4) << jacobians[i](1,0) << ", " << jacobians[i](1,1) << "]";
        force_debug << "\n    [" << std::fixed << std::setprecision(4) << jacobians[i](2,0) << ", " << jacobians[i](2,1) << "]";
        
        // Calculate and log the force-torque relationship
        Eigen::Vector2d tau_j = jacobians[i].transpose() * forces[i];
        force_debug << "\n  Resulting Torques: [" << tau_j(0) << ", " << tau_j(1) << "] Nm";
        force_debug << "\n  Joint Velocities: [" << v1 << ", " << v2 << "] rad/s";
        force_debug << "\n  Power: " << power << " W";
        
        // Also log condition number of the Jacobian to check for numerical issues
        // A high condition number could indicate the Jacobian is nearly singular
        Eigen::JacobiSVD<Eigen::Matrix<double, 3, 2>> svd(jacobians[i]);
        double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
        force_debug << "\n  Jacobian Condition Number: " << cond;
        
        // Calculate expected foot velocity from joint velocities
        Eigen::Vector3d expected_foot_vel = jacobians[i] * Eigen::Vector2d(v1, v2);
        force_debug << "\n  Expected Foot Velocity: [" << expected_foot_vel.x() << ", " 
                    << expected_foot_vel.y() << ", " << expected_foot_vel.z() << "] m/s";
        
        // If available, compare with actual foot velocity
        Eigen::Vector3d actual_foot_vel;
        if (i == 0) actual_foot_vel = Eigen::Vector3d(controller.latest_state_.v1.x, controller.latest_state_.v1.y, controller.latest_state_.v1.z);
        else if (i == 1) actual_foot_vel = Eigen::Vector3d(controller.latest_state_.v2.x, controller.latest_state_.v2.y, controller.latest_state_.v2.z);
        else if (i == 2) actual_foot_vel = Eigen::Vector3d(controller.latest_state_.v3.x, controller.latest_state_.v3.y, controller.latest_state_.v3.z);
        else actual_foot_vel = Eigen::Vector3d(controller.latest_state_.v4.x, controller.latest_state_.v4.y, controller.latest_state_.v4.z);
        
        force_debug << "\n  Actual Foot Velocity: [" << actual_foot_vel.x() << ", " 
                    << actual_foot_vel.y() << ", " << actual_foot_vel.z() << "] m/s";
        
        // Calculate virtual work - this should be conserved (F·v = τ·ω)
        double virtual_work_foot = forces[i].dot(actual_foot_vel);
        double virtual_work_joint = torques[i](0) * v1 + torques[i](1) * v2;
        force_debug << "\n  Virtual Work (F·v): " << virtual_work_foot << " W";
        force_debug << "\n  Virtual Work (τ·ω): " << virtual_work_joint << " W";
        force_debug << "\n  Work Difference: " << (virtual_work_foot - virtual_work_joint) << " W";
      }
      
      force_debug << "\n\nTOTAL POWER: " << total_power << " W";
      
      // Check work conservation across all legs
      force_debug << "\n\nVERIFICATION OF TOTAL SYSTEM:";
      
      // Compare normal forces vs weight
      double robot_weight = 14.65 * 9.81; // mass * gravity
      double weight_ratio = total_vertical_force / robot_weight;
      force_debug << "\n  Total Vertical Force: " << total_vertical_force << " N";
      force_debug << "\n  Robot Weight: " << robot_weight << " N";
      force_debug << "\n  Force/Weight Ratio: " << weight_ratio;
      
      if (std::abs(weight_ratio - 1.0) > 0.2) { // More than 20% difference
        force_debug << "\n  WARNING: Force/weight ratio significantly different from 1.0";
      }
      
      // Get joint command interfaces (post-transformation)
      std::vector<double> applied_torques;
      for (auto &interface : controller.joint_command_interfaces_) {
        applied_torques.push_back(interface.get_value());
      }
      
      force_debug << "\n\nJOINT TORQUE ANALYSIS:";
      for (int i = 0; i < std::min(8, static_cast<int>(applied_torques.size())); i++) {
        int leg_idx = i / 2;
        int joint_idx = i % 2;
        force_debug << "\n  " << leg_names[leg_idx] << " Joint " << joint_idx 
                   << ": Computed=" << torques[leg_idx][joint_idx] 
                   << ", Applied=" << applied_torques[i];
      }
      
      // Log the detailed force analysis on every cycle (removed counter-based logging)
      RCLCPP_INFO(controller.get_node()->get_logger(), "%s", force_debug.str().c_str());

      // Check all torques against limits
      const double torque_limit = 8.0; // 8 Newton-meters
      for (int i = 0; i < 4; i++)
      {
        if (std::abs(torques[i](0)) >= torque_limit || std::abs(torques[i](1)) >= torque_limit)
        {
          RCLCPP_WARN(controller.get_node()->get_logger(),
                      "%s torque exceeds limit: [%.2f, %.2f] Nm (limit: %.2f Nm)",
                      leg_names[i], torques[i](0), torques[i](1), torque_limit);
        }
      }

      // Set torque values to joint interfaces - use flat indexing for compactness
      for (int i = 0; i < 4; i++)
      {
        controller.joint_command_interfaces_[i * 2].set_value(torques[i][0]);
        controller.joint_command_interfaces_[i * 2 + 1].set_value(torques[i][1]);
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
