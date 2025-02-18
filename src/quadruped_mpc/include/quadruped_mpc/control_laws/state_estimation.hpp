#ifndef QUADRUPED_MPC_CONTROL_LAWS_STATE_ESTIMATION_HPP_
#define QUADRUPED_MPC_STATE_ESTIMATION_HPP_

#include "quadruped_mpc/controller_interfaces/state_estimator.hpp"
#include "quadruped_mpc/utilities/shared_quadruped_info.hpp"

namespace quadruped_mpc
{

inline bool StateEstimator::read_state_interfaces()
{
  try {
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
    auto& quadruped_info = SharedQuadrupedInfo::getInstance();
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);

    // Update foot states with positions relative to center of mass
    for (size_t i = 0; i < 4; ++i) {
      foot_states_[i].position = data_->oMf[foot_frame_ids_[i]].translation() + quadruped_info.state_.pc;
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
    auto& quadruped_info = SharedQuadrupedInfo::getInstance();
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);
    
    // For now, just set all contacts to true
    quadruped_info.state_.contact_1_ = true;  // FL
    quadruped_info.state_.contact_2_ = true;  // FR
    quadruped_info.state_.contact_3_ = true;  // RL
    quadruped_info.state_.contact_4_ = true;  // RR

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in contact detection: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::pin_kinematics()
{
  try {
    auto& quadruped_info = SharedQuadrupedInfo::getInstance();
    pinocchio::forwardKinematics(model_, *data_, current_positions_, current_velocities_);
    pinocchio::updateFramePlacements(model_, *data_);
    
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);

    // Update foot positions and set default contact states
    for (size_t i = 0; i < 4; ++i) {
      foot_states_[i].position = data_->oMf[foot_frame_ids_[i]].translation();
      foot_states_[i].in_contact = true;  // Default to true for now
    }

    // Compute full Jacobians in world frame
    Eigen::MatrixXd J_temp(6, model_.nv);

    // For each leg, get the correct joint indices from the model
    // FL
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_,
                                   foot_frame_ids_[0], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                   J_temp);
    quadruped_info.state_.J1 = J_temp.block<3,2>(0, model_.joints[joint_mappings_[0].pinocchio_idx].idx_v());

    // FR
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_,
                                   foot_frame_ids_[1], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                   J_temp);
    quadruped_info.state_.J2 = J_temp.block<3,2>(0, model_.joints[joint_mappings_[2].pinocchio_idx].idx_v());

    // RL
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_,
                                   foot_frame_ids_[2], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                   J_temp);
    quadruped_info.state_.J3 = J_temp.block<3,2>(0, model_.joints[joint_mappings_[4].pinocchio_idx].idx_v());

    // RR
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_,
                                   foot_frame_ids_[3], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                   J_temp);
    quadruped_info.state_.J4 = J_temp.block<3,2>(0, model_.joints[joint_mappings_[6].pinocchio_idx].idx_v());

    // Update angular velocity in shared info from local storage
    quadruped_info.state_.angular_velocity = imu_angular_velocity_;

    RCLCPP_DEBUG(get_node()->get_logger(), 
                 "FL Jacobian:\n%.3f %.3f\n%.3f %.3f\n%.3f %.3f",
                 quadruped_info.state_.J1(0,0), quadruped_info.state_.J1(0,1),
                 quadruped_info.state_.J1(1,0), quadruped_info.state_.J1(1,1),
                 quadruped_info.state_.J1(2,0), quadruped_info.state_.J1(2,1));

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in pin kinematics: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::estimate_base_position()
{
  try {
    auto& quadruped_info = SharedQuadrupedInfo::getInstance();
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);

    // Define foot sphere radius
    constexpr double foot_radius = 0.015;  // meters

    // Calculate average Z height only using contacting feet
    double world_z = 0.0;
    int contact_count = 0;

    for (const auto& foot : foot_states_) {
      if (foot.in_contact) {
        world_z += foot.position.z();
        contact_count++;
      }
    }

    if (contact_count > 0) {
      world_z /= contact_count;
      world_z = world_z - foot_radius;  // Subtract foot radius to get body center

      // Initialize base position
      current_positions_[0] = 0.0;  // x
      current_positions_[1] = 0.0;  // y
      current_positions_[2] = -world_z;  // z

      current_velocities_[0] = 0.0;  // vx
      current_velocities_[1] = 0.0;  // vy
      current_velocities_[2] = 0.0;  // vz

      // Update with odometry if available
      if (latest_odom_) {
        auto odom = latest_odom_;
        auto position = odom->pose.pose.position;
        auto velocity = odom->twist.twist.linear;
        
        // Update position in both current state and shared info
        current_positions_[0] = position.x;
        current_positions_[1] = position.y;
        current_positions_[2] = position.z;
        
        // Update velocity in both current state and shared info
        current_velocities_[0] = velocity.x;
        current_velocities_[1] = velocity.y;
        current_velocities_[2] = velocity.z;

        // Keep shared info updated for backwards compatibility
        quadruped_info.state_.pc = Eigen::Vector3d(position.x, position.y, position.z);
        quadruped_info.state_.vc = Eigen::Vector3d(velocity.x, velocity.y, velocity.z);
      }
      
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Estimated body position: [%.3f, %.3f, %.3f] m",
        current_positions_[0],
        current_positions_[1],
        current_positions_[2]
      );
    } else {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "No feet in contact with ground"
      );
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

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in orientation estimation: %s", e.what());
    return false;
  }
}

inline bool StateEstimator::update_odometry()
{
  try {
    auto time_now = get_node()->get_clock()->now();
    
    geometry_msgs::msg::TransformStamped transform;
    nav_msgs::msg::Odometry odom;
    
    transform.header.stamp = time_now;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    
    odom.header = transform.header;
    odom.child_frame_id = transform.child_frame_id;

    Eigen::Vector3d pc;
    Eigen::Vector3d angular_velocity;
    
    {
      auto& quadruped_info = SharedQuadrupedInfo::getInstance();
      std::lock_guard<std::mutex> lock(quadruped_info.mutex_);
      pc = quadruped_info.state_.pc;
      angular_velocity = quadruped_info.state_.angular_velocity;
    }
    
    const auto& body_placement = model_.frames[body_frame_id_].placement;
    Eigen::Vector3d base_position = pc - body_placement.translation();
    
    transform.transform.translation.x = current_positions_[0];
    transform.transform.translation.y = current_positions_[1];
    transform.transform.translation.z = current_positions_[2];
    
    transform.transform.rotation.w = current_positions_[3];
    transform.transform.rotation.x = current_positions_[4];
    transform.transform.rotation.y = current_positions_[5];
    transform.transform.rotation.z = current_positions_[6];

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

    // Add realtime state publishing here
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

      // Update foot positions and contacts in message
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

      // Convert Eigen matrices to vectors for Jacobians
      auto& info = SharedQuadrupedInfo::getInstance();
      std::lock_guard<std::mutex> lock(info.mutex_);
      
      // Using row-major storage for Jacobians
      Eigen::Map<const Eigen::VectorXd> j1_vec(info.state_.J1.data(), info.state_.J1.size());
      Eigen::Map<const Eigen::VectorXd> j2_vec(info.state_.J2.data(), info.state_.J2.size());
      Eigen::Map<const Eigen::VectorXd> j3_vec(info.state_.J3.data(), info.state_.J3.size());
      Eigen::Map<const Eigen::VectorXd> j4_vec(info.state_.J4.data(), info.state_.J4.size());

      msg.j1.assign(j1_vec.data(), j1_vec.data() + j1_vec.size());
      msg.j2.assign(j2_vec.data(), j2_vec.data() + j2_vec.size());
      msg.j3.assign(j3_vec.data(), j3_vec.data() + j3_vec.size());
      msg.j4.assign(j4_vec.data(), j4_vec.data() + j4_vec.size());

      // Populate contact states
      msg.contact_1 = info.state_.contact_1_;
      msg.contact_2 = info.state_.contact_2_;
      msg.contact_3 = info.state_.contact_3_;
      msg.contact_4 = info.state_.contact_4_;

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
