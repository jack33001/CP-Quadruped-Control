#ifndef QUADRUPED_MPC_CONTROL_LAWS_GAIT_PATTERN_GENERATOR_HPP_
#define QUADRUPED_MPC_CONTROL_LAWS_GAIT_PATTERN_GENERATOR_HPP_

#include <array>
#include <Eigen/Dense>
#include "quadruped_mpc/controller_interfaces/gait_pattern_generator.hpp"

namespace quadruped_mpc
{

inline bool GaitPatternGenerator::unpack_state()
{
  try {
    if (!latest_state_) {
      RCLCPP_WARN(get_node()->get_logger(), "No state message received yet");
      return false;
    }

    // Update foot positions and phases - for now all in stance
    foot_info_[0].position = Eigen::Vector3d(latest_state_->p1.x, latest_state_->p1.y, latest_state_->p1.z);
    foot_info_[1].position = Eigen::Vector3d(latest_state_->p2.x, latest_state_->p2.y, latest_state_->p2.z);
    foot_info_[2].position = Eigen::Vector3d(latest_state_->p3.x, latest_state_->p3.y, latest_state_->p3.z);
    foot_info_[3].position = Eigen::Vector3d(latest_state_->p4.x, latest_state_->p4.y, latest_state_->p4.z);

    // Update contact states from state message
    foot_info_[0].contact = latest_state_->contact_1;
    foot_info_[1].contact = latest_state_->contact_2;
    foot_info_[2].contact = latest_state_->contact_3;
    foot_info_[3].contact = latest_state_->contact_4;

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error unpacking state: %s", e.what());
    return false;
  }
}

inline bool GaitPatternGenerator::update_foot_phase()
{
  try {
    if (!latest_state_) {
      RCLCPP_WARN(get_node()->get_logger(), "No state message received yet");
      return false;
    }

    // For each foot, update phase and state based on contact
    for (auto& foot : foot_info_) {
        // Stance
        if (foot.contact) {
            foot.phase = 1.0;    // Add .0 for double
            foot.state = 0;    // Stance state
            foot.step_target = foot.position;  // Target is current position when in stance

        } // Swing
        else {
            if (foot.phase == 1.0) {  // Add .0 for double
                foot.phase = 0.0;      // Add .0 for double
            } else {
                foot.phase = foot.phase + 0.1;  // Add semicolon
            }
            foot.state = 1;    // Swing state
        }
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error updating foot phases: %s", e.what());
    return false;
  }
}

inline bool GaitPatternGenerator::support_polygon()
{
  try {
    if (!latest_state_) {
      RCLCPP_WARN(get_node()->get_logger(), "No state message received yet");
      return false;
    }

    // Set the error function weigting - these determine when the weight starts to shift when a leg is about to make/break contact
    double sigmac0 = .2;
    double sigmac1 = .2;
    double sigmacbar0 = .2;
    double sigmacbar1 = .2;

    // Calculate adaptive weighting factors
    std::array<double, 4> K_cp;
    std::array<double, 4> K_cbarp;
    std::array<double, 4> Phi;

    for (size_t i = 0; i < 4; i++) {
      K_cp[i] = 0.5*(std::erf(foot_info_[i].phase/(sigmac0*std::sqrt(2))) + std::erf((1-foot_info_[i].phase)/(sigmac1*std::sqrt(2))));
      K_cbarp[i] = 0.5*(std::erf(foot_info_[i].phase/(sigmacbar0*std::sqrt(2))) + std::erf((1-foot_info_[i].phase)/(sigmacbar1*std::sqrt(2))));
      // Phi is the total weighting factor
      Phi[i] = (!foot_info_[i].state ? K_cp[i] : K_cbarp[i]);
    }

    // Calculate the virtual points
    std::array<Eigen::Vector2d, 4> xi_m;
    std::array<Eigen::Vector2d, 4> xi_p;

    // Calculate virtual points using 2D positions
    for (size_t i = 0; i < 4; i++) {
      const size_t next_idx = (i + 1) % 4;
      const size_t prev_idx = (i + 3) % 4;

      Eigen::Matrix2d A;
      A << foot_info_[i].position.x(), foot_info_[next_idx].position.x(),
           foot_info_[i].position.y(), foot_info_[next_idx].position.y();
      
      Eigen::Vector2d w(Phi[i], 1-Phi[i]);
      xi_p[i] = A * w;

      A << foot_info_[i].position.x(), foot_info_[prev_idx].position.x(),
           foot_info_[i].position.y(), foot_info_[prev_idx].position.y();
      xi_m[i] = A * w;
    }

    // Calculate support polygon vertices
    std::array<Eigen::Vector2d, 4> xi;
    for (size_t i = 0; i < 4; i++) {
      const size_t next_idx = (i + 1) % 4;
      const size_t prev_idx = (i + 3) % 4;

      xi[i] = (Phi[i] * Eigen::Vector2d(foot_info_[i].position.x(), foot_info_[i].position.y()) +
               Phi[prev_idx] * xi_m[i] +
               Phi[next_idx] * xi_p[i]) /
              (Phi[prev_idx] + Phi[i] + Phi[next_idx]);
    }

    // Calculate support polygon center
    support_center_.setZero();
    for (const auto& vertex : xi) {
      support_center_ += vertex;
    }
    support_center_ /= 4.0;

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error computing support polygon: %s", e.what());
    return false;
  }
}

inline bool GaitPatternGenerator::publish_pattern()
{
  try {
    // First publish foot states
    if (rt_foot_state_pub_ && rt_foot_state_pub_->trylock()) {
      auto& msg = rt_foot_state_pub_->msg_;
      msg.foot1_state = foot_info_[0].state;
      msg.foot2_state = foot_info_[1].state;
      msg.foot3_state = foot_info_[2].state;
      msg.foot4_state = foot_info_[3].state;
      rt_foot_state_pub_->unlockAndPublish();
    }

    // Then publish gait pattern
    if (rt_gait_pub_ && rt_gait_pub_->trylock()) {
      auto& msg = rt_gait_pub_->msg_;

      // Set foot phases
      msg.foot1_phase = static_cast<float>(foot_info_[0].phase);
      msg.foot2_phase = static_cast<float>(foot_info_[1].phase);
      msg.foot3_phase = static_cast<float>(foot_info_[2].phase);
      msg.foot4_phase = static_cast<float>(foot_info_[3].phase);

      // Set step target positions
      msg.foot1_step_position.x = foot_info_[0].step_target.x();
      msg.foot1_step_position.y = foot_info_[0].step_target.y();
      msg.foot1_step_position.z = foot_info_[0].step_target.z();

      msg.foot2_step_position.x = foot_info_[1].step_target.x();
      msg.foot2_step_position.y = foot_info_[1].step_target.y();
      msg.foot2_step_position.z = foot_info_[1].step_target.z();

      msg.foot3_step_position.x = foot_info_[2].step_target.x();
      msg.foot3_step_position.y = foot_info_[2].step_target.y();
      msg.foot3_step_position.z = foot_info_[2].step_target.z();

      msg.foot4_step_position.x = foot_info_[3].step_target.x();
      msg.foot4_step_position.y = foot_info_[3].step_target.y();
      msg.foot4_step_position.z = foot_info_[3].step_target.z();

      // Set COM position (support center)
      msg.com_position.x = support_center_.x();
      msg.com_position.y = support_center_.y();
      msg.com_position.z = 0.0;  // We only compute 2D support polygon

      rt_gait_pub_->unlockAndPublish();
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error publishing pattern: %s", e.what());
    return false;
  }
}

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_CONTROL_LAWS_GAIT_PATTERN_GENERATOR_HPP_
