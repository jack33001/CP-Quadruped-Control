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

inline bool GaitPatternGenerator::update_foot_phase(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  try {
    if (!latest_state_) {
      RCLCPP_WARN(get_node()->get_logger(), "No state message received yet");
      return false;
    }

    // Get teleop commands if available
    Eigen::Vector3d cmd_linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d cmd_angular = Eigen::Vector3d::Zero();
    Eigen::Vector3d com_velocity = Eigen::Vector3d::Zero();
    std::array<Eigen::Vector3d, 4> hip_positions;

    if (latest_cmd_) {
      cmd_linear = Eigen::Vector3d(latest_cmd_->linear.x, latest_cmd_->linear.y, latest_cmd_->linear.z);
      cmd_angular = Eigen::Vector3d(latest_cmd_->angular.x, latest_cmd_->angular.y, latest_cmd_->angular.z);
      // Extract hip positions from state message
      hip_positions[0] = Eigen::Vector3d(latest_state_->h1.x, latest_state_->h1.y, latest_state_->h1.z); // Front left hip
      hip_positions[1] = Eigen::Vector3d(latest_state_->h2.x, latest_state_->h2.y, latest_state_->h2.z); // Front right hip
      hip_positions[2] = Eigen::Vector3d(latest_state_->h3.x, latest_state_->h3.y, latest_state_->h3.z); // Rear left hip
      hip_positions[3] = Eigen::Vector3d(latest_state_->h4.x, latest_state_->h4.y, latest_state_->h4.z); // Rear right hip
      com_velocity = Eigen::Vector3d(
        latest_state_->com_velocity.x,
        latest_state_->com_velocity.y,
        latest_state_->com_velocity.z
      );
    }

    // Calculate period in seconds
    double dt = period.seconds();
    double current_time = time.seconds();

    // Run the state machine for each state
    for (int i = 0; i < 4; i++) {
      auto& foot = foot_info_[i];

      // Manage state transitions first
      // Preinit -> init (only if gait start command has been received)
      if (foot.state == -2 && gait_start_received_) {
        foot.state = -1;
        foot.phase = 0;
        foot.state_start_time = current_time;
        foot.state_end_time = current_time + foot.phase_offset;
        foot.time_in_state = 0.0;
        RCLCPP_DEBUG(get_node()->get_logger(), "Foot %d transitioning from preinit to init state", i+1);
      // Init -> Stance
      } else if ((foot.state == -1) && (current_time >= foot.state_end_time)) {
        foot.state = 0;
        foot.phase = 0;
        foot.state_start_time = current_time;
        foot.state_end_time = current_time + stance_duration_;
        foot.time_in_state = 0.0;
        RCLCPP_DEBUG(get_node()->get_logger(), "Foot %d transitioning from init to stance state", i+1);
      }
      // Swing --> Stance
      else if ((foot.state == 1) && (foot.contact || (foot.phase >= 1))){ 
        // Reset the clock and the phase
        foot.phase = 0;    
        foot.state = 0;
        foot.state_start_time = current_time;
        foot.state_end_time = current_time + stance_duration_;
        foot.time_in_state = 0.0;
      }
      // Stance --> Swing
      else if ((foot.state == 0) && (foot.phase >= 1)){
        // Reset the clock and the phase
        foot.phase = 0.0;
        foot.state = 1;
        foot.state_start_time = current_time;
        foot.state_end_time = current_time + swing_duration_;
        foot.time_in_state = 0.0;
        
        // Calculate and log each term separately
        Eigen::Vector3d term1 = hip_positions[i]; // Current hip position (x-y)
        Eigen::Vector3d term2 = swing_duration_/2 * cmd_linear; // Raibert hueristic
        Eigen::Vector3d term3 = sqrt(step_height_/9.81) * (com_velocity-cmd_linear); // Step height term
          
        // Set the step target
        foot.step_target = term1 + term2 + term3;
      }

      // Now run the state
      // Stance
      if (foot.state == 0) {
        foot.time_in_state = current_time - foot.state_start_time;
        foot.phase += dt / (stance_duration_);
      } 
      // Swing
      else if (foot.state == 1)
      {
        foot.time_in_state = current_time - foot.state_start_time;
        foot.phase += dt / (swing_duration_);
      }
      // Init state - update time but don't advance phase
      else if (foot.state == -1) {
        foot.time_in_state = current_time - foot.state_start_time;
      }
      // Preinit state - don't update anything
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
    support_center_ /= 4.0;  // Average the 4 vertices

    // Replace INFO log with DEBUG log to avoid terminal clutter
    RCLCPP_DEBUG(get_node()->get_logger(), "Support polygon center (COM): x=%f, y=%f", 
                 support_center_.x(), support_center_.y());

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error computing support polygon: %s", e.what());
    return false;
  }
}

inline bool GaitPatternGenerator::publish_pattern()
{
  try {
    // Publish all data via the GaitPattern message
    if (rt_gait_pub_ && rt_gait_pub_->trylock()) {
      auto& msg = rt_gait_pub_->msg_;

      // Set foot states
      msg.foot1_state = static_cast<int32_t>(foot_info_[0].state);
      msg.foot2_state = static_cast<int32_t>(foot_info_[1].state);
      msg.foot3_state = static_cast<int32_t>(foot_info_[2].state);
      msg.foot4_state = static_cast<int32_t>(foot_info_[3].state);

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
