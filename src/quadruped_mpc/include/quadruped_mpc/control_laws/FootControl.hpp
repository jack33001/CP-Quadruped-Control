#ifndef QUADRUPED_MPC_CONTROL_LAWS_FOOT_CONTROL_HPP_
#define QUADRUPED_MPC_CONTROL_LAWS_FOOT_CONTROL_HPP_

#include "quadruped_mpc/controller_interfaces/foot_controller.hpp"
#include <Eigen/Dense>

namespace quadruped_mpc
{

/**
 * @brief Generate swing trajectory for the foot
 * 
 * @param controller Reference to the foot controller
 * @param current_time Current time for trajectory generation
 * @return true if trajectory generation successful
 * @return false if there was an error
 */
inline bool swing_trajectory(FootController& controller, const rclcpp::Time& current_time)
{
  try {
    // TODO: Implement swing trajectory generation
    // For example:
    // 1. Calculate swing phase based on time
    // 2. Generate desired foot position along trajectory
    // 3. Store the desired foot position in controller's internal state
    
    // For now, just a placeholder
    RCLCPP_DEBUG(controller.get_node()->get_logger(), "Generating swing trajectory at time %f", current_time.seconds());
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(controller.get_node()->get_logger(), "Error in swing_trajectory: %s", e.what());
    return false;
  }
}

/**
 * @brief Apply Jacobians to convert foot forces to joint torques
 * 
 * @param controller Reference to the foot controller
 * @return true if Jacobian application successful
 * @return false if there was an error
 */
inline bool apply_jacobians(FootController& controller)
{
  try {
    // Check if we have foot force data
    if (!controller.foot_forces_.has_data) {
      RCLCPP_WARN(controller.get_node()->get_logger(), "No foot force data available");
      return false;
    }
    
    // Check if we have state data with Jacobians
    std::lock_guard<std::mutex> lock(controller.state_mutex_);
    
    // Check if j1-j4 arrays have data
    if (controller.latest_state_.j1.empty()) {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "j1 Jacobian data is empty");
      return false;
    }
    
    if (controller.latest_state_.j2.empty()) {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "j2 Jacobian data is empty");
      return false;
    }
    
    if (controller.latest_state_.j3.empty()) {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "j3 Jacobian data is empty");
      return false;
    }
    
    if (controller.latest_state_.j4.empty()) {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "j4 Jacobian data is empty");
      return false;
    }
    
    // First check if the arrays have the expected size
    if (controller.latest_state_.j1.size() < 6 || 
        controller.latest_state_.j2.size() < 6 || 
        controller.latest_state_.j3.size() < 6 || 
        controller.latest_state_.j4.size() < 6) {
      RCLCPP_ERROR(controller.get_node()->get_logger(), 
                   "Jacobian arrays wrong size: j1=%zu, j2=%zu, j3=%zu, j4=%zu (expected 6 each)", 
                   controller.latest_state_.j1.size(), controller.latest_state_.j2.size(),
                   controller.latest_state_.j3.size(), controller.latest_state_.j4.size());
      return false;
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
    
    // Check if we have enough command interfaces
    if (controller.joint_command_interfaces_.size() < 8) {
      RCLCPP_ERROR(controller.get_node()->get_logger(), 
        "Not enough command interfaces available: %zu (expected 8)", 
        controller.joint_command_interfaces_.size());
      return false;
    }
    
    // Map calculated torques to the correct joint indices
    // Front left leg (leg 1)
    if (!controller.joint_command_interfaces_[0].set_value(torque1[0])) {  // Hip joint
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Failed to set torque for joint 0");
    }
    if (!controller.joint_command_interfaces_[1].set_value(torque1[1])) {  // Knee joint
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Failed to set torque for joint 1");
    }
    
    // Front right leg (leg 2) - FIX: Swap the hip and knee joint torques
    if (!controller.joint_command_interfaces_[2].set_value(torque2[0])) {  // FIXED: Use index 1 (knee)
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Failed to set torque for joint 2");
    }
    if (!controller.joint_command_interfaces_[3].set_value(torque2[1])) {  // FIXED: Use index 0 (hip)
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Failed to set torque for joint 3");
    }
    
    // Rear left leg (leg 3)
    if (!controller.joint_command_interfaces_[4].set_value(torque3[0])) {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Failed to set torque for joint 4");
    }
    if (!controller.joint_command_interfaces_[5].set_value(torque3[1])) {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Failed to set torque for joint 5");
    }
    
    // Rear right leg (leg 4)
    if (!controller.joint_command_interfaces_[6].set_value(torque4[0])) {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Failed to set torque for joint 6");
    }
    if (!controller.joint_command_interfaces_[7].set_value(torque4[1])) {
      RCLCPP_ERROR(controller.get_node()->get_logger(), "Failed to set torque for joint 7");
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(controller.get_node()->get_logger(), "Exception in apply_jacobians: %s", e.what());
    return false;
  }
}

/**
 * @brief Calculate a point on a quadratic Bezier curve
 * 
 * @param p0 Start point
 * @param p1 Control point
 * @param p2 End point
 * @param t Parameter in range [0.0, 1.0]
 * @return Eigen::Vector3d Point on the curve at parameter t
 */
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

/**
 * @brief Generate points along a quadratic Bezier curve
 * 
 * @param p0 Start point
 * @param p1 Control point
 * @param p2 End point
 * @param num_points Number of points to generate
 * @return std::vector<Eigen::Vector3d> Vector of points along the curve
 */
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
