#ifndef QUADRUPED_MPC_CONTROL_LAWS_STATE_ESTIMATION_HPP_
#define QUADRUPED_MPC_CONTROL_LAWS_STATE_ESTIMATION_HPP_

#include "quadruped_mpc/controller_interfaces/state_estimator.hpp"

namespace quadruped_mpc
{
inline bool StateEstimator::read_state_interfaces()
{
  try {
    static int call_count = 0;
    call_count++;
    bool should_log = (call_count % 10 == 0); // Changed from 50 to 10 to match other controllers
    
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "StateEstimator::read_state_interfaces() - Call #%d", call_count
    );

    // Resize joint states vector if needed
    if (joint_states_.size() != joint_names_.size()) {
      joint_states_.resize(joint_names_.size());
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Resized joint_states_ vector to size %zu", joint_states_.size()
      );
    }

    // Read all interfaces for each joint
    const size_t interfaces_per_joint = state_interface_types_.size();
    
    if (should_log) {
      RCLCPP_DEBUG(get_node()->get_logger(), 
                 "Reading %zu joints with %zu interfaces per joint", 
                 joint_names_.size(), interfaces_per_joint);
    }
    
    std::stringstream joint_data_log;
    joint_data_log << "\n\n\n----------------------------------- NEW TIMESTEP -----------------------------------\n\n--- JOINT STATE READINGS ---\n";
    joint_data_log << "Joint | Position | Velocity | Effort\n";
    joint_data_log << "------+----------+----------+--------\n";
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      size_t base_idx = i * interfaces_per_joint;
      // Update local storage only
      joint_states_[i].position = state_interfaces_[base_idx].get_value();      // position
      joint_states_[i].velocity = state_interfaces_[base_idx + 1].get_value();  // velocity
      joint_states_[i].effort = state_interfaces_[base_idx + 2].get_value();    // effort

      joint_data_log << std::setw(5) << joint_names_[i] << " | " 
                    << std::fixed << std::setprecision(4) << std::setw(8) << joint_states_[i].position << " | "
                    << std::setw(8) << joint_states_[i].velocity << " | "
                    << std::setw(8) << joint_states_[i].effort << std::endl;
    }
    
    if (should_log) {
      RCLCPP_DEBUG(get_node()->get_logger(), "%s", joint_data_log.str().c_str());
    }

    // Read IMU data - assuming they're after all joint interfaces
    const size_t imu_start_idx = joint_names_.size() * interfaces_per_joint;
    // Store quaternion in Pinocchio's order [x,y,z,w]
    imu_orientation_ = Eigen::Vector4d(
      state_interfaces_[imu_start_idx].get_value(),      // x
      state_interfaces_[imu_start_idx + 1].get_value(),  // y
      state_interfaces_[imu_start_idx + 2].get_value(),  // z
      state_interfaces_[imu_start_idx + 3].get_value()   // w
    );

    // Read and store angular velocities directly
    imu_angular_velocity_ = Eigen::Vector3d(
      state_interfaces_[imu_start_idx + 4].get_value(),  // wx
      state_interfaces_[imu_start_idx + 5].get_value(),  // wy
      state_interfaces_[imu_start_idx + 6].get_value()   // wz
    );

    // Read and store linear accelerations
    imu_linear_acceleration_ = Eigen::Vector3d(
      state_interfaces_[imu_start_idx + 7].get_value(),  // ax
      state_interfaces_[imu_start_idx + 8].get_value(),  // ay
      state_interfaces_[imu_start_idx + 9].get_value()   // az
    );

    if (should_log) {
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "\n--- IMU READINGS ---\n"
        "Orientation [x,y,z,w]: [%.4f, %.4f, %.4f, %.4f]\n"
        "Angular Velocity [x,y,z]: [%.4f, %.4f, %.4f]\n"
        "Linear Acceleration [x,y,z]: [%.4f, %.4f, %.4f]",
        imu_orientation_[0], imu_orientation_[1], imu_orientation_[2], imu_orientation_[3],
        imu_angular_velocity_[0], imu_angular_velocity_[1], imu_angular_velocity_[2],
        imu_linear_acceleration_[0], imu_linear_acceleration_[1], imu_linear_acceleration_[2]
      );
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error reading state interfaces: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::update_model()
{
  try {
    static int call_count = 0;
    call_count++;
    bool should_log = (call_count % 10 == 0); // Changed from 50 to 10
    
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "StateEstimator::update_model() - Call #%d", call_count
    );

    // Clear position and velocity vectors
    current_positions_.setZero();
    current_velocities_.setZero();

    std::stringstream mapping_log;
    mapping_log << "\n--- JOINT TO PINOCCHIO STATE MAPPING ---\n";
    mapping_log << "Joint | Pinocchio Model Position | Pinocchio Model Velocity\n";
    mapping_log << "------+-------------------------+-------------------------\n";

    // Map joint states to Pinocchio model states
    for (size_t i = 0; i < joint_mappings_.size(); ++i) {
      const auto& mapping = joint_mappings_[i];
      const auto& joint = model_.joints[mapping.pinocchio_idx];
      // Get joint state
      const auto& state = joint_states_[i];
      // Map to Pinocchio state vectors
      if (joint.nq() == 2) {  // Revolute joints in Pinocchio use sin/cos
        current_positions_[joint.idx_q()] = std::cos(state.position);
        current_positions_[joint.idx_q() + 1] = std::sin(state.position);
        
        mapping_log << std::setw(5) << joint_names_[i] << " | "
                    << "q[" << joint.idx_q() << "] = cos(" << state.position << ") = " << std::cos(state.position) << ", "
                    << "q[" << joint.idx_q()+1 << "] = sin(" << state.position << ") = " << std::sin(state.position) << " | "
                    << "v[" << joint.idx_v() << "] = " << state.velocity << std::endl;
      } else {
        current_positions_[joint.idx_q()] = state.position;
        
        mapping_log << std::setw(5) << joint_names_[i] << " | "
                    << "q[" << joint.idx_q() << "] = " << state.position << " | "
                    << "v[" << joint.idx_v() << "] = " << state.velocity << std::endl;
      }
      current_velocities_[joint.idx_v()] = state.velocity;
    }
    
    if (should_log) {
      RCLCPP_DEBUG(get_node()->get_logger(), "%s", mapping_log.str().c_str());
      
      // Log the full state vectors for debugging
      std::stringstream state_log;
      state_log << "\n--- FULL STATE VECTORS AFTER MAPPING ---\n";
      state_log << "Positions (q): [";
      for (int i = 0; i < current_positions_.size(); i++) {
        state_log << current_positions_[i];
        if (i < current_positions_.size() - 1) state_log << ", ";
      }
      state_log << "]\n";
      
      state_log << "Velocities (v): [";
      for (int i = 0; i < current_velocities_.size(); i++) {
        state_log << current_velocities_[i];
        if (i < current_velocities_.size() - 1) state_log << ", ";
      }
      state_log << "]\n";
      
      RCLCPP_DEBUG(get_node()->get_logger(), "%s", state_log.str().c_str());
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error updating model: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::foot_positions()
{
  try {
    static int call_count = 0;
    call_count++;
    bool should_log = (call_count % 10 == 0); // Changed from 50 to 10
    
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "StateEstimator::foot_positions() - Call #%d", call_count
    );

    // Update foot states with positions relative to center of mass
    std::stringstream foot_log;
    foot_log << "\n--- FOOT POSITIONS RELATIVE TO COM ---\n";
    foot_log << "Foot | Position (X, Y, Z)\n";
    foot_log << "-----+-------------------\n";
    
    for (size_t i = 0; i < 4; ++i) {
      Eigen::Vector3d raw_position = data_->oMf[foot_frame_ids_[i]].translation();
      Eigen::Vector3d com_offset(current_positions_[0], current_positions_[1], current_positions_[2]);
      foot_states_[i].position = raw_position + com_offset;
      
      foot_log << "  " << i << "  | (" 
              << std::fixed << std::setprecision(4) << foot_states_[i].position.x() << ", "
              << std::fixed << std::setprecision(4) << foot_states_[i].position.y() << ", "
              << std::fixed << std::setprecision(4) << foot_states_[i].position.z() << ")\n";
    }
    
    if (should_log) {
      RCLCPP_DEBUG(get_node()->get_logger(), "%s", foot_log.str().c_str());
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in foot position calculation: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::detect_contact()
{
  try {
    static int call_count = 0;
    call_count++;
    bool should_log = (call_count % 10 == 0); // Changed from 50 to 10
    
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "StateEstimator::detect_contact() - Call #%d", call_count
    );

    // Log the input gait pattern data
    std::stringstream gait_log;
    gait_log << "\n--- GAIT PATTERN DATA ---\n";
    if (gait_pattern_.foot1_state >= 0) {
      gait_log << "Foot1: " << (gait_pattern_.foot1_state == 1 ? "SWING" : "STANCE") << " (" << gait_pattern_.foot1_state << ")\n"
              << "Foot2: " << (gait_pattern_.foot2_state == 1 ? "SWING" : "STANCE") << " (" << gait_pattern_.foot2_state << ")\n"
              << "Foot3: " << (gait_pattern_.foot3_state == 1 ? "SWING" : "STANCE") << " (" << gait_pattern_.foot3_state << ")\n"
              << "Foot4: " << (gait_pattern_.foot4_state == 1 ? "SWING" : "STANCE") << " (" << gait_pattern_.foot4_state << ")\n";
    } else {
      gait_log << "No valid gait pattern data available (all values < 0)\n";
    }
    
    // Use gait pattern information directly from member variable
    // Check if we have valid gait pattern data (foot1_state is at least 0)
    if (gait_pattern_.foot1_state >= 0) {
      // Foot states: 0 = stance/contact, 1 = flight/no contact
      foot_states_[0].in_contact = (gait_pattern_.foot1_state != 1);
      foot_states_[1].in_contact = (gait_pattern_.foot2_state != 1);
      foot_states_[2].in_contact = (gait_pattern_.foot3_state != 1);
      foot_states_[3].in_contact = (gait_pattern_.foot4_state != 1);
      
      gait_log << "\n--- RESULTING FOOT CONTACT STATES ---\n";
      gait_log << "Foot1: " << (foot_states_[0].in_contact ? "CONTACT" : "NO CONTACT") << "\n"
              << "Foot2: " << (foot_states_[1].in_contact ? "CONTACT" : "NO CONTACT") << "\n" 
              << "Foot3: " << (foot_states_[2].in_contact ? "CONTACT" : "NO CONTACT") << "\n"
              << "Foot4: " << (foot_states_[3].in_contact ? "CONTACT" : "NO CONTACT") << "\n";
    } else {
      // Default to all contacts true if no gait pattern is available
      for (auto& foot : foot_states_) {
        foot.in_contact = true;
      }
      gait_log << "\n--- RESULTING FOOT CONTACT STATES ---\n";
      gait_log << "All feet in CONTACT (default due to no gait pattern)\n";
    }
    
    if (should_log) {
      RCLCPP_DEBUG(get_node()->get_logger(), "%s", gait_log.str().c_str());
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in contact detection: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::pin_kinematics()
{
  try {
    static int call_count = 0;
    call_count++;
    bool should_log = (call_count % 10 == 0); // Changed from 50 to 10
    
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "StateEstimator::pin_kinematics() - Call #%d", call_count
    );

    RCLCPP_DEBUG(get_node()->get_logger(), "Running forward kinematics with positions size: %ld, velocities size: %ld", 
                current_positions_.size(), current_velocities_.size());
                
    // Forward kinematics
    pinocchio::forwardKinematics(model_, *data_, current_positions_, current_velocities_);
    pinocchio::updateFramePlacements(model_, *data_);
    
    if (should_log) {
      RCLCPP_DEBUG(get_node()->get_logger(), "Forward kinematics computed successfully");
    }
    
    // Update foot positions and set default contact states
    std::stringstream foot_states_log;
    foot_states_log << "\n--- FOOT STATES AFTER KINEMATICS ---\n";
    foot_states_log << "Foot | Position (X, Y, Z)        | Velocity (X, Y, Z)       | Contact\n";
    foot_states_log << "-----+--------------------------+-------------------------+---------\n";
    
    for (size_t i = 0; i < 4; ++i) {
      foot_states_[i].position = data_->oMf[foot_frame_ids_[i]].translation();
      foot_states_[i].in_contact = true;  // Default to true for now

      // Calculate and store foot frame velocity
      auto frame_velocity = pinocchio::getFrameVelocity(model_, *data_, foot_frame_ids_[i], 
                               pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
      foot_states_[i].velocity = frame_velocity.linear();

      foot_states_log << "  " << i << "  | (" 
                     << std::fixed << std::setprecision(4) << foot_states_[i].position.x() << ", "
                     << std::fixed << std::setprecision(4) << foot_states_[i].position.y() << ", "
                     << std::fixed << std::setprecision(4) << foot_states_[i].position.z() << ") | ("
                     << std::fixed << std::setprecision(4) << foot_states_[i].velocity.x() << ", "
                     << std::fixed << std::setprecision(4) << foot_states_[i].velocity.y() << ", "
                     << std::fixed << std::setprecision(4) << foot_states_[i].velocity.z() << ") | "
                     << (foot_states_[i].in_contact ? "TRUE" : "FALSE") << "\n";

      // Log velocity in different frames for debugging
      auto local_velocity = pinocchio::getFrameVelocity(model_, *data_, foot_frame_ids_[i], 
                  pinocchio::ReferenceFrame::LOCAL);
      auto world_velocity = pinocchio::getFrameVelocity(model_, *data_, foot_frame_ids_[i], 
                  pinocchio::ReferenceFrame::WORLD);

      if (should_log) {
        // Detailed velocity logging for each foot when should_log is true
        RCLCPP_DEBUG(get_node()->get_logger(), 
                  "Foot %zu velocities (detailed):\n"
                  "  LOCAL: [%.4f, %.4f, %.4f]\n"
                  "  LOCAL_WORLD_ALIGNED: [%.4f, %.4f, %.4f]\n"
                  "  WORLD: [%.4f, %.4f, %.4f]",
                  i,
                  local_velocity.linear()[0], local_velocity.linear()[1], local_velocity.linear()[2],
                  frame_velocity.linear()[0], frame_velocity.linear()[1], frame_velocity.linear()[2],
                  world_velocity.linear()[0], world_velocity.linear()[1], world_velocity.linear()[2]);
      }
    }
    
    if (should_log) {
      RCLCPP_DEBUG(get_node()->get_logger(), "%s", foot_states_log.str().c_str());
    }

    // Compute full Jacobians in world frame
    Eigen::MatrixXd J_temp(6, model_.nv);
    // FL
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_,
                                   foot_frame_ids_[0], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                   J_temp);
    leg_jacobians_.J1 = J_temp.block<3,2>(0, model_.joints[joint_mappings_[0].pinocchio_idx].idx_v());
    // FR
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_,
                                   foot_frame_ids_[1], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                   J_temp);
    leg_jacobians_.J2 = J_temp.block<3,2>(0, model_.joints[joint_mappings_[2].pinocchio_idx].idx_v());
    // RL
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_,
                                   foot_frame_ids_[2], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                   J_temp);
    leg_jacobians_.J3 = J_temp.block<3,2>(0, model_.joints[joint_mappings_[4].pinocchio_idx].idx_v());
    // RR
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_,
                                   foot_frame_ids_[3], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                   J_temp);
    leg_jacobians_.J4 = J_temp.block<3,2>(0, model_.joints[joint_mappings_[6].pinocchio_idx].idx_v());

    if (should_log) {
      // Jacobian logging
      std::stringstream jacobian_log;
      jacobian_log << "\n--- LEG JACOBIANS ---\n";
      
      auto print_jacobian = [&jacobian_log](const std::string& name, const Eigen::Matrix<double, 3, 2>& J) {
        jacobian_log << name << ":\n";
        for (int i = 0; i < J.rows(); i++) {
          for (int j = 0; j < J.cols(); j++) {
            jacobian_log << std::fixed << std::setprecision(4) << std::setw(8) << J(i, j);
            if (j < J.cols() - 1) jacobian_log << " ";
          }
          jacobian_log << "\n";
        }
      };
      
      print_jacobian("J1 (FL)", leg_jacobians_.J1);
      print_jacobian("J2 (FR)", leg_jacobians_.J2);
      print_jacobian("J3 (RL)", leg_jacobians_.J3);
      print_jacobian("J4 (RR)", leg_jacobians_.J4);
      
      RCLCPP_DEBUG(get_node()->get_logger(), "%s", jacobian_log.str().c_str());
    }

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in pin kinematics: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::estimate_base_position()
{
  try {
    static int call_count = 0;
    call_count++;
    bool should_log = (call_count % 10 == 0); // Changed from 50 to 10
    
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "StateEstimator::estimate_base_position() - Call #%d", call_count
    );
    
    std::stringstream base_pos_log;
    base_pos_log << "\n--- BASE POSITION ESTIMATION ---\n";

    if (latest_odom_) {
      auto odom = latest_odom_;
      auto position = odom->pose.pose.position;
      auto orientation = odom->pose.pose.orientation;
      auto velocity = odom->twist.twist.linear;
      auto angular_velocity = odom->twist.twist.angular;
      
      base_pos_log << "Ground Truth Odometry:\n";
      base_pos_log << "  Position: (" << position.x << ", " << position.y << ", " << position.z << ")\n";
      base_pos_log << "  Orientation: [" << orientation.x << ", " << orientation.y << ", " 
                  << orientation.z << ", " << orientation.w << "]\n";
      base_pos_log << "  Linear Velocity: (" << velocity.x << ", " << velocity.y << ", " << velocity.z << ")\n";
      base_pos_log << "  Angular Velocity: (" << angular_velocity.x << ", " << angular_velocity.y << ", " 
                  << angular_velocity.z << ")\n";
    
      // Define foot sphere radius
      constexpr double foot_radius = 0.015;  // meters
      
      // Count the contacts
      int contact_count = 0;
      for (size_t i = 0; i < foot_states_.size(); i++) {
        if (foot_states_[i].in_contact) {
          contact_count++;
        }
      }
      
      base_pos_log << "\nContacting feet count: " << contact_count << "\n";

      // We'll solve Ax = b for the COM velocity
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3*contact_count, 3);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(3*contact_count);

      base_pos_log << "Current COM Position before update: [" << current_positions_[0] << ", " 
                  << current_positions_[1] << ", " << current_positions_[2] << "]\n";

      Eigen::Vector3d body_velocity = Eigen::Vector3d(velocity.x, velocity.y, velocity.z);
      Eigen::Vector3d body_angular_velocity = Eigen::Vector3d(angular_velocity.x, angular_velocity.y, angular_velocity.z);
      
      // For each foot in contact
      int contacting_foot_number = 0;
      base_pos_log << "\nFoot contact data for velocity estimation:\n";
      base_pos_log << "Foot | In Contact | Position               | Velocity              | Angular Cross-Term\n";
      base_pos_log << "-----+-----------+-----------------------+----------------------+------------------\n";

      for (size_t i = 0; i < foot_states_.size(); i++) {
        if (foot_states_[i].in_contact) {
          Eigen::Vector3d cross_term = -body_angular_velocity.cross(foot_states_[i].position);
          
          base_pos_log << "  " << i << "  | " 
                      << (foot_states_[i].in_contact ? "YES" : "NO ") << "       | ("
                      << std::fixed << std::setprecision(3) << foot_states_[i].position.x() << ", "
                      << std::fixed << std::setprecision(3) << foot_states_[i].position.y() << ", "
                      << std::fixed << std::setprecision(3) << foot_states_[i].position.z() << ") | ("
                      << std::fixed << std::setprecision(3) << foot_states_[i].velocity.x() << ", "
                      << std::fixed << std::setprecision(3) << foot_states_[i].velocity.y() << ", "
                      << std::fixed << std::setprecision(3) << foot_states_[i].velocity.z() << ") | ("
                      << std::fixed << std::setprecision(3) << cross_term.x() << ", "
                      << std::fixed << std::setprecision(3) << cross_term.y() << ", "
                      << std::fixed << std::setprecision(3) << cross_term.z() << ")\n";
          
          // Fill one 3x3 block of the A matrix with identity
          A.block<3,3>(3*contacting_foot_number, 0) = Eigen::Matrix3d::Identity();
          
          // Fill the corresponding section of b with negative foot velocity
          b.segment<3>(3*contacting_foot_number) = -foot_states_[i].velocity - body_angular_velocity.cross(foot_states_[i].position);
          
          contacting_foot_number++;
        } else {
          base_pos_log << "  " << i << "  | NO        | (--skipped non-contact foot--)\n";
        }
      }

      // Solve for COM velocity if we have contacts
      Eigen::Vector3d v_com = Eigen::Vector3d::Zero();
      if (contact_count > 0) {
        // Solve the system A*v_com = b using least squares
        v_com = A.colPivHouseholderQr().solve(b);
        
        base_pos_log << "\nLinear velocity estimation:\n";
        base_pos_log << "  Estimated v_com: (" << v_com[0] << ", " << v_com[1] << ", " << v_com[2] << ")\n";
        base_pos_log << "  Odom velocity: (" << velocity.x << ", " << velocity.y << ", " << velocity.z << ")\n";
        base_pos_log << "  Difference: (" << v_com[0]-velocity.x << ", " << v_com[1]-velocity.y << ", " << v_com[2]-velocity.z << ")\n";
        
        // Update velocity in current state
        current_velocities_[0] = velocity.x;  // Using odom velocity instead of estimated
        current_velocities_[1] = velocity.y;  // Using odom velocity instead of estimated
        current_velocities_[2] = velocity.z;  // Using odom velocity instead of estimated
        
        base_pos_log << "  Using odom velocities for state update\n";
      } else {
        base_pos_log << "\nNo feet in contact with ground - cannot estimate velocity\n";
      }
      
      // World z position - still compute from foot positions
      double world_z = 0.0;
      if (contact_count > 0) {
        for (const auto& foot : foot_states_) {
          if (foot.in_contact) {
            world_z -= foot.position.z();
          }
        }
        world_z /= contact_count;
        world_z += foot_radius; // Adjust by foot radius
        
        base_pos_log << "\nZ position calculation from contacts:\n";
        base_pos_log << "  Average negative foot z: " << -world_z << " + foot radius " << foot_radius << " = " << world_z << "\n";
      } else {
        world_z = position.z;  // Use odom if no contacts
        base_pos_log << "\nNo feet in contact, using odom Z: " << world_z << "\n";
      }
      
      // Update position
      base_pos_log << "\nPosition update:\n";
      base_pos_log << "  Before: (" << current_positions_[0] << ", " << current_positions_[1] << ", " << current_positions_[2] << ")\n";
      
      current_positions_[0] += 0;  // No update to X
      current_positions_[1] += 0;  // No update to Y
      current_positions_[2] = world_z; // Direct Z from contact
      
      base_pos_log << "  After: (" << current_positions_[0] << ", " << current_positions_[1] << ", " << current_positions_[2] << ")\n";
      
    } else {
      base_pos_log << "No odometry data available\n";
    }
    
    if (should_log) {
      RCLCPP_DEBUG(get_node()->get_logger(), "%s", base_pos_log.str().c_str());
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in base position estimation: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::estimate_orientation()
{
  try {
    static int call_count = 0;
    call_count++;
    bool should_log = (call_count % 10 == 0); // Changed from 50 to 10
    
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "StateEstimator::estimate_orientation() - Call #%d", call_count
    );

    std::stringstream orient_log;
    orient_log << "\n--- ORIENTATION ESTIMATION ---\n";
    orient_log << "IMU quaternion [x,y,z,w]: [" 
               << imu_orientation_[0] << ", " 
               << imu_orientation_[1] << ", " 
               << imu_orientation_[2] << ", " 
               << imu_orientation_[3] << "]\n";
    
    orient_log << "Angular velocity [x,y,z]: [" 
               << imu_angular_velocity_[0] << ", " 
               << imu_angular_velocity_[1] << ", " 
               << imu_angular_velocity_[2] << "]\n";

    orient_log << "Previous state quaternion [x,y,z,w]: [" 
               << current_positions_[3] << ", " 
               << current_positions_[4] << ", " 
               << current_positions_[5] << ", " 
               << current_positions_[6] << "]\n";
               
    // Copy IMU quaternion to state - note the different ordering in imu_orientation_
    current_positions_[3] = imu_orientation_[0];  // x
    current_positions_[4] = imu_orientation_[1];  // y
    current_positions_[5] = imu_orientation_[2];  // z
    current_positions_[6] = imu_orientation_[3];  // w

    // Update the body's angular velocities
    current_velocities_[3] = imu_angular_velocity_[0];
    current_velocities_[4] = imu_angular_velocity_[1];
    current_velocities_[5] = imu_angular_velocity_[2];
    
    orient_log << "Updated state quaternion [x,y,z,w]: [" 
               << current_positions_[3] << ", " 
               << current_positions_[4] << ", " 
               << current_positions_[5] << ", " 
               << current_positions_[6] << "]\n";
               
    orient_log << "Updated angular velocity [x,y,z]: [" 
               << current_velocities_[3] << ", " 
               << current_velocities_[4] << ", " 
               << current_velocities_[5] << "]\n";

    if (should_log) {
      RCLCPP_DEBUG(get_node()->get_logger(), "%s", orient_log.str().c_str());
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in orientation estimation: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::update_odometry()
{
  try {
    static int call_count = 0;
    call_count++;
    bool should_log = (call_count % 10 == 0); // Changed from "more frequently than other methods" to match other controllers
    
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "StateEstimator::update_odometry() - Call #%d", call_count
    );

    std::stringstream odom_log;
    odom_log << "\n--- ODOMETRY UPDATE ---\n";

    // Log ground truth data if available
    if (latest_odom_) {
      auto position = latest_odom_->pose.pose.position;
      auto orientation = latest_odom_->pose.pose.orientation;
      auto linear_vel = latest_odom_->twist.twist.linear;
      auto angular_vel = latest_odom_->twist.twist.angular;
      
      odom_log << "GROUND TRUTH ODOMETRY:\n";
      odom_log << "  Position: (" << position.x << ", " << position.y << ", " << position.z << ")\n";
      odom_log << "  Orientation: [" << orientation.x << ", " << orientation.y << ", " 
              << orientation.z << ", " << orientation.w << "]\n"; 
      odom_log << "  Linear Velocity: (" << linear_vel.x << ", " << linear_vel.y << ", " << linear_vel.z << ")\n";
      odom_log << "  Angular Velocity: (" << angular_vel.x << ", " << angular_vel.y << ", " << angular_vel.z << ")\n\n";
    } else {
      odom_log << "GROUND TRUTH: No data available\n\n";
    }

    // Publish tf data for RViz
    auto time_now = get_node()->get_clock()->now();
    geometry_msgs::msg::TransformStamped transform;
    nav_msgs::msg::Odometry odom;

    transform.header.stamp = time_now;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    
    odom.header = transform.header;
    odom.child_frame_id = transform.child_frame_id;

    Eigen::Vector3d pc(current_positions_[0], current_positions_[1], current_positions_[2]);
    Eigen::Vector3d angular_velocity = imu_angular_velocity_;
    const auto& body_placement = model_.frames[body_frame_id_].placement;
    Eigen::Vector3d base_position = pc - body_placement.translation();
    
    odom_log << "Center of Mass Position: (" << pc.x() << ", " << pc.y() << ", " << pc.z() << ")\n";
    odom_log << "Body placement translation: (" 
             << body_placement.translation().x() << ", " 
             << body_placement.translation().y() << ", " 
             << body_placement.translation().z() << ")\n";
    odom_log << "Base position (COM - body): (" 
             << base_position.x() << ", " 
             << base_position.y() << ", " 
             << base_position.z() << ")\n";

    transform.transform.translation.x = current_positions_[0];
    transform.transform.translation.y = current_positions_[1];
    transform.transform.translation.z = current_positions_[2];
    
    transform.transform.rotation.x = current_positions_[3];
    transform.transform.rotation.y = current_positions_[4];
    transform.transform.rotation.z = current_positions_[5];
    transform.transform.rotation.w = current_positions_[6];

    odom.pose.pose.position.x = transform.transform.translation.x;
    odom.pose.pose.position.y = transform.transform.translation.y;
    odom.pose.pose.position.z = transform.transform.translation.z;
    odom.pose.pose.orientation = transform.transform.rotation;

    odom.twist.twist.linear.x = current_velocities_[0];
    odom.twist.twist.linear.y = current_velocities_[1];
    odom.twist.twist.linear.z = current_velocities_[2];
    odom.twist.twist.angular.x = angular_velocity.x();
    odom.twist.twist.angular.y = angular_velocity.y();
    odom.twist.twist.angular.z = angular_velocity.z();

    tf_broadcaster_->sendTransform(transform);
    odom_pub_->publish(odom);
    
    odom_log << "\nPublished Transform and Odometry:\n";
    odom_log << "  Position: (" 
             << transform.transform.translation.x << ", " 
             << transform.transform.translation.y << ", " 
             << transform.transform.translation.z << ")\n";
    odom_log << "  Orientation: [" 
             << transform.transform.rotation.x << ", "
             << transform.transform.rotation.y << ", "
             << transform.transform.rotation.z << ", "
             << transform.transform.rotation.w << "]\n";
    odom_log << "  Linear Velocity: (" 
             << odom.twist.twist.linear.x << ", " 
             << odom.twist.twist.linear.y << ", " 
             << odom.twist.twist.linear.z << ")\n";
    odom_log << "  Angular Velocity: (" 
             << odom.twist.twist.angular.x << ", " 
             << odom.twist.twist.angular.y << ", " 
             << odom.twist.twist.angular.z << ")\n";

    // Publish the state estimate
    bool state_published = false;
    if (rt_state_pub_ && rt_state_pub_->trylock()) {
      auto& msg = rt_state_pub_->msg_;

      // Convert Eigen vectors to Point messages
      auto eigen_to_point = [](const Eigen::Vector3d& vec) {
        geometry_msgs::msg::Point point;
        point.x = vec.x();
        point.y = vec.y();
        point.z = vec.z();
        return point;
      };

      // Convert Eigen vectors to Vector3 messages
      auto eigen_to_vector3 = [](const Eigen::Vector3d& vec) {
        geometry_msgs::msg::Vector3 v;
        v.x = vec.x();
        v.y = vec.y();
        v.z = vec.z();
        return v;
      };

      // Update foot positions and contacts directly from foot_states_
      msg.p1 = eigen_to_point(foot_states_[0].position);
      msg.p2 = eigen_to_point(foot_states_[1].position);
      msg.p3 = eigen_to_point(foot_states_[2].position);
      msg.p4 = eigen_to_point(foot_states_[3].position);

      // Add foot velocities from foot_states_
      msg.v1 = eigen_to_vector3(foot_states_[0].velocity);
      msg.v2 = eigen_to_vector3(foot_states_[1].velocity);
      msg.v3 = eigen_to_vector3(foot_states_[2].velocity);
      msg.v4 = eigen_to_vector3(foot_states_[3].velocity);

      msg.contact_1 = foot_states_[0].in_contact;
      msg.contact_2 = foot_states_[1].in_contact;
      msg.contact_3 = foot_states_[2].in_contact;
      msg.contact_4 = foot_states_[3].in_contact;

      // Convert joint states directly from local storage
      msg.joint_positions.resize(joint_states_.size());
      msg.joint_velocities.resize(joint_states_.size());
      msg.joint_efforts.resize(joint_states_.size());

      for (size_t i = 0; i < joint_states_.size(); ++i) {
        msg.joint_positions[i] = joint_states_[i].position;
        msg.joint_velocities[i] = joint_states_[i].velocity;
        msg.joint_efforts[i] = joint_states_[i].effort;
      }

      // Populate IMU state
      msg.orientation.w = imu_orientation_[3];  // w
      msg.orientation.x = imu_orientation_[0];  // x
      msg.orientation.y = imu_orientation_[1];  // y
      msg.orientation.z = imu_orientation_[2];  // z

      // Convert Eigen vector to Vector3 message for angular velocity
      msg.angular_velocity.x = imu_angular_velocity_[0];
      msg.angular_velocity.y = imu_angular_velocity_[1];
      msg.angular_velocity.z = imu_angular_velocity_[2];

      // Convert Eigen vector to Vector3 message for COM velocity
      msg.com_velocity.x = current_velocities_[0];
      msg.com_velocity.y = current_velocities_[1];
      msg.com_velocity.z = current_velocities_[2];

      // Add COM position from current state vectors
      msg.pc.x = current_positions_[0];
      msg.pc.y = current_positions_[1];
      msg.pc.z = current_positions_[2];

      // Convert Eigen matrices to vectors for Jacobians - now using local storage
      std::vector<double> j1_vec(6), j2_vec(6), j3_vec(6), j4_vec(6);

      // Map the matrices to vectors in column-major order
      Eigen::Map<Eigen::VectorXd>(j1_vec.data(), 6) = Eigen::Map<const Eigen::VectorXd>(leg_jacobians_.J1.data(), 6);
      Eigen::Map<Eigen::VectorXd>(j2_vec.data(), 6) = Eigen::Map<const Eigen::VectorXd>(leg_jacobians_.J2.data(), 6);
      Eigen::Map<Eigen::VectorXd>(j3_vec.data(), 6) = Eigen::Map<const Eigen::VectorXd>(leg_jacobians_.J3.data(), 6);
      Eigen::Map<Eigen::VectorXd>(j4_vec.data(), 6) = Eigen::Map<const Eigen::VectorXd>(leg_jacobians_.J4.data(), 6);

      msg.j1 = j1_vec;
      msg.j2 = j2_vec;
      msg.j3 = j3_vec;
      msg.j4 = j4_vec;

      rt_state_pub_->unlockAndPublish();
      state_published = true;
      
      odom_log << "\nPublished QuadrupedState message with data:\n";
      odom_log << "  COM: (" << msg.pc.x << ", " << msg.pc.y << ", " << msg.pc.z << ")\n";
      odom_log << "  COM Velocity: (" << msg.com_velocity.x << ", " << msg.com_velocity.y << ", " << msg.com_velocity.z << ")\n";
      odom_log << "  Orientation: [" << msg.orientation.x << ", " << msg.orientation.y << ", " << msg.orientation.z << ", " << msg.orientation.w << "]\n";
      odom_log << "  Foot Contacts: [" << msg.contact_1 << ", " << msg.contact_2 << ", " << msg.contact_3 << ", " << msg.contact_4 << "]\n";
      odom_log << "  Foot Positions:\n";
      odom_log << "    p1: (" << msg.p1.x << ", " << msg.p1.y << ", " << msg.p1.z << ")\n";
      odom_log << "    p2: (" << msg.p2.x << ", " << msg.p2.y << ", " << msg.p2.z << ")\n";
      odom_log << "    p3: (" << msg.p3.x << ", " << msg.p3.y << ", " << msg.p3.z << ")\n";
      odom_log << "    p4: (" << msg.p4.x << ", " << msg.p4.y << ", " << msg.p4.z << ")\n";
    } else {
      odom_log << "\nFailed to acquire lock for QuadrupedState publication\n";
    }

    if (should_log) {
      RCLCPP_DEBUG(get_node()->get_logger(), "%s", odom_log.str().c_str());
      
      // Log a simple summary of the entire state estimation pipeline
      RCLCPP_DEBUG(get_node()->get_logger(), 
                 "\n=== STATE ESTIMATION PIPELINE SUMMARY ===\n"
                 "COM Position: (%.4f, %.4f, %.4f)\n"
                 "COM Velocity: (%.4f, %.4f, %.4f)\n"
                 "Orientation: [%.4f, %.4f, %.4f, %.4f]\n"
                 "Angular Velocity: (%.4f, %.4f, %.4f)\n"
                 "State published: %s\n"
                 "Contacts: [%s, %s, %s, %s]",
                 current_positions_[0], current_positions_[1], current_positions_[2],
                 current_velocities_[0], current_velocities_[1], current_velocities_[2],
                 current_positions_[3], current_positions_[4], current_positions_[5], current_positions_[6],
                 current_velocities_[3], current_velocities_[4], current_velocities_[5],
                 state_published ? "YES" : "NO",
                 foot_states_[0].in_contact ? "TRUE" : "FALSE",
                 foot_states_[1].in_contact ? "TRUE" : "FALSE",
                 foot_states_[2].in_contact ? "TRUE" : "FALSE",
                 foot_states_[3].in_contact ? "TRUE" : "FALSE");
      
      // Add comparison with ground truth if available
      if (latest_odom_) {
        auto gt_pos = latest_odom_->pose.pose.position;
        auto gt_vel = latest_odom_->twist.twist.linear;
        RCLCPP_DEBUG(get_node()->get_logger(),
                   "Ground Truth Position: (%.4f, %.4f, %.4f)\n"
                   "Position Error: (%.4f, %.4f, %.4f)\n"
                   "Ground Truth Velocity: (%.4f, %.4f, %.4f)\n"
                   "Velocity Error: (%.4f, %.4f, %.4f)",
                   gt_pos.x, gt_pos.y, gt_pos.z,
                   current_positions_[0] - gt_pos.x, 
                   current_positions_[1] - gt_pos.y, 
                   current_positions_[2] - gt_pos.z,
                   gt_vel.x, gt_vel.y, gt_vel.z,
                   current_velocities_[0] - gt_vel.x, 
                   current_velocities_[1] - gt_vel.y, 
                   current_velocities_[2] - gt_vel.z);
      }
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error updating odometry: %s", e.what());
    return false;
  }
}

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_CONTROL_LAWS_STATE_ESTIMATION_HPP_