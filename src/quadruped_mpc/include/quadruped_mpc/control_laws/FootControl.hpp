#ifndef QUADRUPED_MPC_CONTROL_LAWS_FOOT_CONTROL_HPP_
#define QUADRUPED_MPC_CONTROL_LAWS_FOOT_CONTROL_HPP_

#include "quadruped_mpc/controller_interfaces/foot_controller.hpp"
#include <Eigen/Dense>
#include <sstream>

namespace quadruped_mpc
{

// Helper to log vector values
inline std::string vec3_to_string(const Eigen::Vector3d& vec) {
  std::stringstream ss;
  ss << "[" << vec[0] << ", " << vec[1] << ", " << vec[2] << "]";
  return ss.str();
}

// Helper to log 3x2 matrix values
inline std::string mat32_to_string(const Eigen::Matrix<double, 3, 2>& mat) {
  std::stringstream ss;
  ss << "[[" << mat(0,0) << ", " << mat(0,1) << "], ";
  ss << "[" << mat(1,0) << ", " << mat(1,1) << "], ";
  ss << "[" << mat(2,0) << ", " << mat(2,1) << "]]";
  return ss.str();
}

// Helper to log vector2d values
inline std::string vec2_to_string(const Eigen::Vector2d& vec) {
  std::stringstream ss;
  ss << "[" << vec[0] << ", " << vec[1] << "]";
  return ss.str();
}

inline bool read_topics(FootController& controller)
{
  try {
    // Get latest state and gait pattern data from the realtime buffers
    auto state_msg = controller.state_buffer_->readFromRT();
    auto gait_msg = controller.gait_buffer_->readFromRT();
    
    // Update state data - properly handle the shared pointer
    if (state_msg) {
      std::lock_guard<std::mutex> lock(controller.state_mutex_);
      controller.latest_state_ = *state_msg;
      
      // Only log state sizes if they've changed or on initialization
      static size_t prev_j1_size = 0, prev_j2_size = 0, prev_j3_size = 0, prev_j4_size = 0;
      if (controller.latest_state_.j1.size() != prev_j1_size ||
          controller.latest_state_.j2.size() != prev_j2_size ||
          controller.latest_state_.j3.size() != prev_j3_size ||
          controller.latest_state_.j4.size() != prev_j4_size) {
        
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
    if (gait_msg) {
      std::lock_guard<std::mutex> lock(controller.gait_mutex_);
      controller.latest_gait_ = *gait_msg;
      // Only log once in a while or on significant changes
      static int gait_counter = 0;
      if ((gait_counter++ % 500) == 0) { 
        RCLCPP_DEBUG(controller.get_node()->get_logger(), "Updated gait pattern data");
      }
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(controller.get_node()->get_logger(), "Error in read_topics: %s", e.what());
    return false;
  }
}

inline bool determine_forces(FootController& controller)
{
  try {
    // Get latest foot forces from the realtime buffer
    auto foot_forces_msg = controller.foot_forces_buffer_->readFromRT();
    
    // Get latest gait pattern to determine foot states
    std::lock_guard<std::mutex> gait_lock(controller.gait_mutex_);
    bool have_gait_data = controller.latest_gait_.foot1_state != 0 && 
                         controller.latest_gait_.foot2_state != 0 && 
                         controller.latest_gait_.foot3_state != 0 && 
                         controller.latest_gait_.foot4_state != 0;
    
    // If we have forces data, update our local foot forces with state-based logic
    if (foot_forces_msg) {
      // Start with forces from the balance controller
      controller.foot_forces_.foot1 = foot_forces_msg->foot1_force;
      controller.foot_forces_.foot2 = foot_forces_msg->foot2_force;
      controller.foot_forces_.foot3 = foot_forces_msg->foot3_force;
      controller.foot_forces_.foot4 = foot_forces_msg->foot4_force;

      // Apply state-based logic if we have gait data
      if (have_gait_data) {
        // If foot is in swing state (state = 1), zero the force
        // Foot 1 (Front left)
        if (controller.latest_gait_.foot1_state == 1) {
          controller.foot_forces_.foot1.x = 0.0;
          controller.foot_forces_.foot1.y = 0.0;
          controller.foot_forces_.foot1.z = 0.0;
        }
        
        // Foot 2 (Front right)
        if (controller.latest_gait_.foot2_state == 1) {
          controller.foot_forces_.foot2.x = 0.0;
          controller.foot_forces_.foot2.y = 0.0;
          controller.foot_forces_.foot2.z = 0.0;
        }
        
        // Foot 3 (Rear left)
        if (controller.latest_gait_.foot3_state == 1) {
          controller.foot_forces_.foot3.x = 0.0;
          controller.foot_forces_.foot3.y = 0.0;
          controller.foot_forces_.foot3.z = 0.0;
        }
        
        // Foot 4 (Rear right)
        if (controller.latest_gait_.foot4_state == 1) {
          controller.foot_forces_.foot4.x = 0.0;
          controller.foot_forces_.foot4.y = 0.0;
          controller.foot_forces_.foot4.z = 0.0;
        }
      }
      
      controller.foot_forces_.has_data = true;
      
      // Log only occasionally to avoid flooding
      static int force_log_counter = 0;
      if ((force_log_counter++ % 500) == 0) {
        RCLCPP_DEBUG(controller.get_node()->get_logger(), 
          "Foot forces after state processing: F1=[%.3f, %.3f, %.3f], F2=[%.3f, %.3f, %.3f], F3=[%.3f, %.3f, %.3f], F4=[%.3f, %.3f, %.3f]",
          controller.foot_forces_.foot1.x, controller.foot_forces_.foot1.y, controller.foot_forces_.foot1.z,
          controller.foot_forces_.foot2.x, controller.foot_forces_.foot2.y, controller.foot_forces_.foot2.z,
          controller.foot_forces_.foot3.x, controller.foot_forces_.foot3.y, controller.foot_forces_.foot3.z,
          controller.foot_forces_.foot4.x, controller.foot_forces_.foot4.y, controller.foot_forces_.foot4.z);
        
        if (have_gait_data) {
          RCLCPP_DEBUG(controller.get_node()->get_logger(),
            "Foot states: F1=%d, F2=%d, F3=%d, F4=%d",
            controller.latest_gait_.foot1_state,
            controller.latest_gait_.foot2_state,
            controller.latest_gait_.foot3_state,
            controller.latest_gait_.foot4_state);
        }
      }
      
      return true;
    } else {
      // Log only very occasionally when there's no new data
      static int no_data_counter = 0;
      if ((no_data_counter++ % 1000) == 0 && controller.foot_forces_.has_data) {
        RCLCPP_DEBUG(controller.get_node()->get_logger(), "Using previously received foot forces");
      }
    }
    
    return controller.foot_forces_.has_data;  // Return true if we already have data from a previous call
  } catch (const std::exception& e) {
    RCLCPP_ERROR(controller.get_node()->get_logger(), "Error in determine_forces: %s", e.what());
    return false;
  }
}

inline bool swing_trajectory(FootController& controller, const rclcpp::Time& current_time)
{
  try {
    // TODO: Implement swing trajectory generation
    // For now, just a placeholder - no need to log every time
    static int swing_counter = 0;
    if ((swing_counter++ % 500) == 0) {
      RCLCPP_DEBUG(controller.get_node()->get_logger(), "Generating swing trajectory at time %f", current_time.seconds());
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(controller.get_node()->get_logger(), "Error in swing_trajectory: %s", e.what());
    return false;
  }
}

inline bool apply_jacobians(FootController& controller)
{
  try {
    // Check if we have foot force data
    if (!controller.foot_forces_.has_data) {
      RCLCPP_DEBUG(controller.get_node()->get_logger(), "No foot force data available");
      // Apply zero torques instead of returning false
      for (auto & interface : controller.joint_command_interfaces_) {
        interface.set_value(0.0);
      }
      return true;
    }
    
    // Check if we have state data with Jacobians
    std::lock_guard<std::mutex> lock(controller.state_mutex_);
    
    // Check if any of the Jacobians are empty
    bool jacobians_empty = controller.latest_state_.j1.empty() || 
                           controller.latest_state_.j2.empty() || 
                           controller.latest_state_.j3.empty() || 
                           controller.latest_state_.j4.empty();
    
    // Check if arrays have the expected size
    bool jacobians_wrong_size = controller.latest_state_.j1.size() < 6 || 
                                controller.latest_state_.j2.size() < 6 || 
                                controller.latest_state_.j3.size() < 6 || 
                                controller.latest_state_.j4.size() < 6;
    
    // If any Jacobian is empty or wrong size, apply zero torques and return true
    if (jacobians_empty || jacobians_wrong_size) {
      static int jacobian_error_counter = 0;
      if ((jacobian_error_counter++ % 100) == 0) {
        if (jacobians_empty) {
          RCLCPP_WARN(controller.get_node()->get_logger(), 
            "Jacobian data incomplete: j1=%s, j2=%s, j3=%s, j4=%s", 
            controller.latest_state_.j1.empty() ? "empty" : "has data",
            controller.latest_state_.j2.empty() ? "empty" : "has data",
            controller.latest_state_.j3.empty() ? "empty" : "has data",
            controller.latest_state_.j4.empty() ? "empty" : "has data");
        }
        
        if (jacobians_wrong_size) {
          RCLCPP_WARN(controller.get_node()->get_logger(), 
            "Jacobian arrays wrong size: j1=%zu, j2=%zu, j3=%zu, j4=%zu (expected 6 each)", 
            controller.latest_state_.j1.size(), controller.latest_state_.j2.size(),
            controller.latest_state_.j3.size(), controller.latest_state_.j4.size());
        }
      }
      
      // Apply zero torques to all joints
      for (auto & interface : controller.joint_command_interfaces_) {
        interface.set_value(0.0);
      }
      return true;
    }
    
    // Extract forces from the foot_forces_ structure
    Eigen::Vector3d force1(
      controller.foot_forces_.foot1.x,
      controller.foot_forces_.foot1.y,
      controller.foot_forces_.foot1.z
    );
    
    Eigen::Vector3d force2(
      controller.foot_forces_.foot2.x,
      controller.foot_forces_.foot2.y,
      controller.foot_forces_.foot2.z
    );
    
    Eigen::Vector3d force3(
      controller.foot_forces_.foot3.x,
      controller.foot_forces_.foot3.y,
      controller.foot_forces_.foot3.z
    );
    
    Eigen::Vector3d force4(
      controller.foot_forces_.foot4.x,
      controller.foot_forces_.foot4.y,
      controller.foot_forces_.foot4.z
    );
    
    // Debug logging at very low frequency
    static int debug_counter = 0;
    if ((debug_counter++ % 2000) == 0) {
      RCLCPP_DEBUG(controller.get_node()->get_logger(), "Processing foot forces and Jacobians");
    }

    // Check if we have enough command interfaces
    if (controller.joint_command_interfaces_.size() < 8) {
      RCLCPP_WARN(controller.get_node()->get_logger(), 
        "Not enough command interfaces available: %zu (expected 8)", 
        controller.joint_command_interfaces_.size());
      
      // Apply zero torques to all available joints
      for (auto & interface : controller.joint_command_interfaces_) {
        interface.set_value(0.0);
      }
      return true;
    }

    // Convert Jacobian vectors to Eigen matrices (3x2)
    Eigen::Matrix<double, 3, 2> J1, J2, J3, J4;
    
    // Assuming Jacobian is stored in row-major order as [J11, J12, J21, J22, J31, J32]
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
    
    // Calculate joint torques using the transpose of the Jacobian: τ = J^T * F
    Eigen::Vector2d torque1 = J1.transpose() * -force1;  // Negative force for actuation direction
    Eigen::Vector2d torque2 = J2.transpose() * -force2;
    Eigen::Vector2d torque3 = J3.transpose() * -force3;
    Eigen::Vector2d torque4 = J4.transpose() * -force4;
    
    // Map calculated torques to the correct joint indices
    // Front left leg (leg 1)
    controller.joint_command_interfaces_[0].set_value(torque1[0]);  // Hip joint
    controller.joint_command_interfaces_[1].set_value(torque1[1]);  // Knee joint
    
    // Front right leg (leg 2)
    controller.joint_command_interfaces_[2].set_value(torque2[0]);
    controller.joint_command_interfaces_[3].set_value(torque2[1]);
    
    // Rear left leg (leg 3)
    controller.joint_command_interfaces_[4].set_value(torque3[0]);
    controller.joint_command_interfaces_[5].set_value(torque3[1]);
    
    // Rear right leg (leg 4)
    controller.joint_command_interfaces_[6].set_value(torque4[0]);
    controller.joint_command_interfaces_[7].set_value(torque4[1]);
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(controller.get_node()->get_logger(), "Exception in apply_jacobians: %s", e.what());
    
    // Apply zero torques in case of an exception
    try {
      for (auto & interface : controller.joint_command_interfaces_) {
        interface.set_value(0.0);
      }
    } catch (...) {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Failed to apply zero torques after exception");
    }
    
    return true;  // Return true to avoid controller deactivation
  }
}

inline Eigen::Vector3d bezier_quadratic(
    const Eigen::Vector3d& p0, 
    const Eigen::Vector3d& p1, 
    const Eigen::Vector3d& p2, 
    double t)
{
    // Clamp t between 0 and 1
    t = std::max(0.0, std::min(1.0, t));
    
    // Quadratic Bezier formula: B(t) = (1-t)^2 * P0 + 2(1-t)t * P1 + t^2 * P2
    double t1 = 1.0 - t;
    return t1 * t1 * p0 + 2.0 * t1 * t * p1 + t * t * p2;
}

inline std::vector<Eigen::Vector3d> generate_bezier_path(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    int num_points)
{
    std::vector<Eigen::Vector3d> path;
    path.reserve(num_points);
    
    for (int i = 0; i < num_points; i++) {
        double t = static_cast<double>(i) / (num_points - 1);
        path.push_back(bezier_quadratic(p0, p1, p2, t));
    }
    
    return path;
}

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_CONTROL_LAWS_FOOT_CONTROL_HPP_
