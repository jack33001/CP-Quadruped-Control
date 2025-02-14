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

    auto& quadruped_info = SharedQuadrupedInfo::getInstance();
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);

    // Read all interfaces for each joint
    const size_t interfaces_per_joint = state_interface_types_.size();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      size_t base_idx = i * interfaces_per_joint;
      
      // Update local storage
      joint_states_[i].position = state_interfaces_[base_idx].get_value();      // position
      joint_states_[i].velocity = state_interfaces_[base_idx + 1].get_value();  // velocity
      joint_states_[i].effort = state_interfaces_[base_idx + 2].get_value();    // effort

      // Update shared state arrays
      quadruped_info.state_.joint_pos[i] = joint_states_[i].position;
      quadruped_info.state_.joint_vel[i] = joint_states_[i].velocity;
      quadruped_info.state_.joint_eff[i] = joint_states_[i].effort;
    }

    // Read IMU data - assuming they're after all joint interfaces
    const size_t imu_start_idx = joint_names_.size() * interfaces_per_joint;
    
    // Store quaternion in Pinocchio's order [x,y,z,w]
    quadruped_info.state_.orientation_quat = Eigen::Vector4d(
      state_interfaces_[imu_start_idx].get_value(),      // x
      state_interfaces_[imu_start_idx + 1].get_value(),  // y
      state_interfaces_[imu_start_idx + 2].get_value(),  // z
      state_interfaces_[imu_start_idx + 3].get_value()   // w
    );

    // Read and store angular velocities directly
    quadruped_info.state_.angular_velocity = Eigen::Vector3d(
      state_interfaces_[imu_start_idx + 4].get_value(),  // wx
      state_interfaces_[imu_start_idx + 5].get_value(),  // wy
      state_interfaces_[imu_start_idx + 6].get_value()   // wz
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

    // Get foot positions in correct order: FL, FR, RL,
    quadruped_info.state_.p1 = data_->oMf[foot_frame_ids_[0]].translation() + quadruped_info.state_.pc;  // FL
    quadruped_info.state_.p2 = data_->oMf[foot_frame_ids_[1]].translation() + quadruped_info.state_.pc;  // FR
    quadruped_info.state_.p3 = data_->oMf[foot_frame_ids_[2]].translation() + quadruped_info.state_.pc;  // RL
    quadruped_info.state_.p4 = data_->oMf[foot_frame_ids_[3]].translation() + quadruped_info.state_.pc;  // RR

    // Copy joint states to shared info
    for (size_t i = 0; i < joint_states_.size(); ++i) {
      quadruped_info.state_.joint_pos[i] = joint_states_[i].position;
      quadruped_info.state_.joint_vel[i] = joint_states_[i].velocity;
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in inverse kinematics: %s", e.what());
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

    // Store foot positions as before
    quadruped_info.state_.p1 = data_->oMf[foot_frame_ids_[0]].translation();  // FL
    quadruped_info.state_.p2 = data_->oMf[foot_frame_ids_[1]].translation();  // FR
    quadruped_info.state_.p3 = data_->oMf[foot_frame_ids_[2]].translation();  // RL
    quadruped_info.state_.p4 = data_->oMf[foot_frame_ids_[3]].translation();  // RR

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

    // Get foot positions and contact states
    std::vector<Eigen::Vector3d> foot_positions = {
      quadruped_info.state_.p1,  // FL
      quadruped_info.state_.p2,  // FR
      quadruped_info.state_.p3,  // RL
      quadruped_info.state_.p4   // RR
    };
    std::vector<bool> contacts = {
      quadruped_info.state_.contact_1_,
      quadruped_info.state_.contact_2_,
      quadruped_info.state_.contact_3_,
      quadruped_info.state_.contact_4_
    };

    // Calculate average Z height only, using the average of the contacting footp positions
    double world_z = 0.0;
    int contact_count = 0;

    for (size_t i = 0; i < 4; ++i) {
      if (contacts[i]) {
        world_z += foot_positions[i].z();
        contact_count++;
      }
    }

    if (contact_count > 0) {
      world_z /= contact_count;
      world_z = world_z - foot_radius;  // Subtract foot radius to get body center

      quadruped_info.state_.pc.x() = 0.0;
      quadruped_info.state_.pc.y() = 0.0;
      quadruped_info.state_.pc.z() = -world_z;

      if (latest_odom_){
        auto odom = latest_odom_;
        auto position = odom->pose.pose.position;
        auto velocity = odom->twist.twist.linear;
        quadruped_info.state_.pc.x() = position.x;
        quadruped_info.state_.pc.y() = position.y;
        quadruped_info.state_.pc.z() = position.z;
        quadruped_info.state_.vc.x() = velocity.x;
        quadruped_info.state_.vc.y() = velocity.y;
        quadruped_info.state_.vc.z() = velocity.z;
      }
      
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Estimated body position: [%.3f, %.3f, %.3f] m",
        quadruped_info.state_.pc.x(),
        quadruped_info.state_.pc.y(),
        quadruped_info.state_.pc.z()
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
    auto& quadruped_info = SharedQuadrupedInfo::getInstance();
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);

    current_positions_[3] = quadruped_info.state_.orientation_quat[0];  // x
    current_positions_[4] = quadruped_info.state_.orientation_quat[1];  // y
    current_positions_[5] = quadruped_info.state_.orientation_quat[2];  // z
    current_positions_[6] = quadruped_info.state_.orientation_quat[3];  // w

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

    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "TF frames: parent='%s', child='%s'",
      transform.header.frame_id.c_str(),
      transform.child_frame_id.c_str()
    );

    odom.pose.pose.position.x = transform.transform.translation.x;
    odom.pose.pose.position.y = transform.transform.translation.y;
    odom.pose.pose.position.z = transform.transform.translation.z;
    odom.pose.pose.orientation = transform.transform.rotation;

    odom.twist.twist.linear = geometry_msgs::msg::Vector3();  // Zero for now
    odom.twist.twist.angular.x = angular_velocity.x();
    odom.twist.twist.angular.y = angular_velocity.y();
    odom.twist.twist.angular.z = angular_velocity.z();

    RCLCPP_DEBUG(get_node()->get_logger(), 
      "Transform - Position: [%.3f, %.3f, %.3f], Rotation: [%.3f, %.3f, %.3f, %.3f]",
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z,
      transform.transform.rotation.w,
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z);

    tf_broadcaster_->sendTransform(transform);
    odom_pub_->publish(odom);

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error updating odometry: %s", e.what());
    return false;
  }
}

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_CONTROL_LAWS_STATE_ESTIMATION_HPP_
