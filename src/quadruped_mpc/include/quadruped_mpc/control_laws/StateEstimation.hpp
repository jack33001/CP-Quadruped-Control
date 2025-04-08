#ifndef QUADRUPED_MPC_CONTROL_LAWS_STATE_ESTIMATION_HPP_
#define QUADRUPED_MPC_STATE_ESTIMATION_HPP_

#include "quadruped_mpc/controller_interfaces/state_estimator.hpp"

namespace quadruped_mpc
{
inline bool StateEstimator::read_state_interfaces()
{
  try {
    //RCLCPP_INFO(
    //  get_node()->get_logger(),
    //  "\n------------------------------------------------------------------------------------------------------------------------------------------------\n New State\n------------------------------------------------------------------------------------------------------------------------------------------------\n"
    //);

    // Resize joint states vector if needed
    if (joint_states_.size() != joint_names_.size()) {
      joint_states_.resize(joint_names_.size());
    }

    // Read all interfaces for each joint
    const size_t interfaces_per_joint = state_interface_types_.size();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      size_t base_idx = i * interfaces_per_joint;
      // Update local storage only
      joint_states_[i].position = state_interfaces_[base_idx].get_value();      // position
      joint_states_[i].velocity = state_interfaces_[base_idx + 1].get_value();  // velocity
      joint_states_[i].effort = state_interfaces_[base_idx + 2].get_value();    // effort
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

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error reading state interfaces: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::update_model()
{
  try {
    // Clear position and velocity vectors
    current_positions_.setZero();
    current_velocities_.setZero();

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
      } else {
        current_positions_[joint.idx_q()] = state.position;
      }
      current_velocities_[joint.idx_v()] = state.velocity;
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
    // Update foot states with positions relative to center of mass
    for (size_t i = 0; i < 4; ++i) {
      foot_states_[i].position = data_->oMf[foot_frame_ids_[i]].translation() + 
                                Eigen::Vector3d(current_positions_[0], current_positions_[1], current_positions_[2]);
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
    // Just set all contacts to true for now
    for (auto& foot : foot_states_) {
      foot.in_contact = true;
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
    pinocchio::forwardKinematics(model_, *data_, current_positions_, current_velocities_);
    pinocchio::updateFramePlacements(model_, *data_);
    // Update foot positions and set default contact states
    std::stringstream foot_velocities_stream; // Add declaration for the stream
    
    for (size_t i = 0; i < 4; ++i) {
      foot_states_[i].position = data_->oMf[foot_frame_ids_[i]].translation();
      foot_states_[i].in_contact = true;  // Default to true for now

      // Calculate and store foot frame velocity
      auto frame_velocity = pinocchio::getFrameVelocity(model_, *data_, foot_frame_ids_[i], 
                               pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
      foot_states_[i].velocity = frame_velocity.linear();

      // Log velocity in different frames for debugging
      auto local_velocity = pinocchio::getFrameVelocity(model_, *data_, foot_frame_ids_[i], 
                  pinocchio::ReferenceFrame::LOCAL);
      auto world_velocity = pinocchio::getFrameVelocity(model_, *data_, foot_frame_ids_[i], 
                  pinocchio::ReferenceFrame::WORLD);

      // Collect velocity information in a stringstream
      if (i == 0) {  // Initialize the stringstream with headers on first foot only
      foot_velocities_stream << "\n----- Foot Velocity Information -----\n";
      }
      
      foot_velocities_stream << "Foot " << i << " Velocity:\n"
          << "                 | X        | Y        | Z        |\n"
          << "-----------------+----------+----------+----------|\n"
          << "LOCAL Linear     | " << std::fixed << std::setprecision(3)
          << std::setw(8) << local_velocity.linear()[0] << " | "
          << std::setw(8) << local_velocity.linear()[1] << " | " 
          << std::setw(8) << local_velocity.linear()[2] << " |\n"
          << "L_W_A Linear     | " 
          << std::setw(8) << frame_velocity.linear()[0] << " | "
          << std::setw(8) << frame_velocity.linear()[1] << " | " 
          << std::setw(8) << frame_velocity.linear()[2] << " |\n"
          << "WORLD Linear     | " 
          << std::setw(8) << world_velocity.linear()[0] << " | "
          << std::setw(8) << world_velocity.linear()[1] << " | " 
          << std::setw(8) << world_velocity.linear()[2] << " |\n";
    }
    
    // Change from DEBUG to INFO to make it show up in the terminal
    //RCLCPP_INFO(get_node()->get_logger(), "%s", foot_velocities_stream.str().c_str());

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

    // Also change this to INFO to show the Jacobian matrices
    //RCLCPP_INFO(
    //  get_node()->get_logger(), 
    //  "FL Jacobian:\n%.3f %.3f\n%.3f %.3f\n%.3f %.3f",
    //  leg_jacobians_.J1(0,0), leg_jacobians_.J1(0,1),
    //  leg_jacobians_.J1(1,0), leg_jacobians_.J1(1,1),
    //  leg_jacobians_.J1(2,0), leg_jacobians_.J1(2,1)
    //);

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in pin kinematics: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::estimate_base_position()
{
  try {
    if (latest_odom_) {
      auto odom = latest_odom_;
      auto position = odom->pose.pose.position;
      auto velocity = odom->twist.twist.linear;
      auto angular_velocity = odom->twist.twist.angular;
    // Define foot sphere radius
    constexpr double foot_radius = 0.015;  // meters
    
    // Count the contacts
    int contact_count = 0;
    for (size_t i = 0; i < foot_states_.size(); i++) {
      if (foot_states_[i].in_contact) {
        contact_count++;
      }
    }

    // We'll solve Ax = b for the COM velocity
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3*contact_count, 3);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(3*contact_count);

    // Log COM position
    //RCLCPP_INFO(
    //  get_node()->get_logger(),
    //  "COM Position: [%.3f, %.3f, %.3f]",
    //  current_positions_[0], current_positions_[1], current_positions_[2]
    //);

    Eigen::Vector3d body_velocity = Eigen::Vector3d(velocity.x, velocity.y, velocity.z);
    Eigen::Vector3d body_angular_velocity = Eigen::Vector3d(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    Eigen::Vector3d correct_foot_velocity;
    
    // For each foot in contact
    int contacting_foot_number = 0;
    std::stringstream foot_info_stream;
    foot_info_stream << "\nFoot | Position [m]                   | Velocity [m/s]\n"
             << "-----+--------------------------------+-------------------------------\n";

    for (size_t i = 0; i < foot_states_.size(); i++) {
      if (foot_states_[i].in_contact) {      
      // Log the values used in the equation
      //RCLCPP_INFO(
      //  get_node()->get_logger(),
      //  "Foot %zu equation values:\n"
      //  "  body_angular_velocity: [%.3f, %.3f, %.3f]\n"
      //  "  foot_position: [%.3f, %.3f, %.3f]\n"
      //  "  cross product result: [%.3f, %.3f, %.3f]\n"
      //  "  body_velocity: [%.3f, %.3f, %.3f]\n",
      //  i,
      //  body_angular_velocity.x(), body_angular_velocity.y(), body_angular_velocity.z(),
      //  foot_states_[i].position.x(), foot_states_[i].position.y(), foot_states_[i].position.z(),
      //  (-body_angular_velocity.cross(foot_states_[i].position)).x(),
      //  (-body_angular_velocity.cross(foot_states_[i].position)).y(),
      //  (-body_angular_velocity.cross(foot_states_[i].position)).z(),
      //  body_velocity.x(), body_velocity.y(), body_velocity.z()
      //);
      
      // Add foot information to the string stream
      foot_info_stream << " " << i << "   | X: " << std::fixed << std::setprecision(3) 
               << foot_states_[i].position[0] << " Y: " << foot_states_[i].position[1] 
               << " Z: " << foot_states_[i].position[2] 
               << " | X: " << foot_states_[i].velocity[0] << " Y: " << foot_states_[i].velocity[1] 
               << " Z: " << foot_states_[i].velocity[2];
      
      // Fill one 3x3 block of the A matrix with identity
      A.block<3,3>(3*contacting_foot_number, 0) = Eigen::Matrix3d::Identity();
      
      // Fill the corresponding section of b with negative foot velocity
      b.segment<3>(3*contacting_foot_number) = -foot_states_[i].velocity - imu_angular_velocity_.cross(foot_states_[i].position);
      
      contacting_foot_number++;
      } 
    }
    
    // Log all foot information at once after the loop
    if (contact_count > 0) {
      //RCLCPP_INFO(get_node()->get_logger(), "%s", foot_info_stream.str().c_str());
    }

    // Solve for COM velocity if we have contacts
    Eigen::Vector3d v_com = Eigen::Vector3d::Zero();
    if (contact_count > 0) {
      Eigen::IOFormat matFormat(4, 0, ", ", "\n", "[", "]");
      
      // Solve the system A*v_com = b using least squares
      v_com = A.colPivHouseholderQr().solve(b);
      
        // Update velocity in current state
        current_velocities_[0] = velocity.x;//v_com[0];//
        current_velocities_[1] = velocity.y;//v_com[1];//
        current_velocities_[2] = velocity.z;//v_com[2];//

        // Debug output in a well-formatted table style
        //RCLCPP_INFO(
        //  get_node()->get_logger(),
        //  "\nMeasurement  | X       | Y       | Z       | W       \n"
        //  "-------------+---------+---------+---------+---------\n"
        //  "IMU Orient   | %6.3f  | %6.3f  | %6.3f  | %6.3f \n"
        //  "GT Orient    | %6.3f  | %6.3f  | %6.3f  | %6.3f \n"
        //  "IMU Ang Vel  | %6.3f  | %6.3f  | %6.3f  | %6s \n"
        //  "GT Ang Vel   | %6.3f  | %6.3f  | %6.3f  | %6s \n"
        //  "Est COM Vel  | %6.3f  | %6.3f  | %6.3f  | %6s \n"
        //  "Actual Vel   | %6.3f  | %6.3f  | %6.3f  | %6s \n"
        //  "Vel Error    | %6.3f  | %6.3f  | %6.3f  | %6s",
        //  imu_orientation_[0], imu_orientation_[1], imu_orientation_[2], imu_orientation_[3],
        //  odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w,
        //  imu_angular_velocity_.x(), imu_angular_velocity_.y(), imu_angular_velocity_.z(), "",
        //  angular_velocity.x, angular_velocity.y, angular_velocity.z, "",
        //  v_com[0], v_com[1], v_com[2], "",
        //  velocity.x, velocity.y, velocity.z, "",
        //  v_com[0] - velocity.x, v_com[1] - velocity.y, v_com[2] - velocity.z, ""
        //);
      }
      
      // World z position - still compute from foot positions
      double world_z = 0.0;
      for (const auto& foot : foot_states_) {
        if (foot.in_contact) {
          world_z -= foot.position.z();
        }
      }
      world_z /= contact_count;
      world_z += foot_radius; // Adjust by foot radius
      
      // Update position
      current_positions_[0] += 0; 
      current_positions_[1] += 0;
      current_positions_[2] = world_z; // Direct Z from contact
      
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "No feet in contact with ground");
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
    current_positions_[3] = imu_orientation_[0];  // x
    current_positions_[4] = imu_orientation_[1];  // y
    current_positions_[5] = imu_orientation_[2];  // z
    current_positions_[6] = imu_orientation_[3];  // w

    // Update the body's angular velocities
    current_velocities_[3] = imu_angular_velocity_[0];
    current_velocities_[4] = imu_angular_velocity_[1];
    current_velocities_[5] = imu_angular_velocity_[2];

    // Implement moving average filter if necessary

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in orientation estimation: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::update_odometry()
{
  try {
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

    odom.twist.twist.linear = geometry_msgs::msg::Vector3();  // Zero for now
    odom.twist.twist.angular.x = angular_velocity.x();
    odom.twist.twist.angular.y = angular_velocity.y();
    odom.twist.twist.angular.z = angular_velocity.z();

    tf_broadcaster_->sendTransform(transform);
    odom_pub_->publish(odom);

    // Publish the state estimate
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

      // Update foot positions and contacts directly from foot_states_
      msg.p1 = eigen_to_point(foot_states_[0].position);
      msg.p2 = eigen_to_point(foot_states_[1].position);
      msg.p3 = eigen_to_point(foot_states_[2].position);
      msg.p4 = eigen_to_point(foot_states_[3].position);
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
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error updating odometry: %s", e.what());
    return false;
  }
}

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_CONTROL_LAWS_STATE_ESTIMATION_HPP_