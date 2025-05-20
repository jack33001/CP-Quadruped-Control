/**
 * @file GaitPatternGenerator.hpp
 * @brief Implementation of the quadruped gait pattern generator
 * 
 * This file contains the implementation of the core functions that manage the
 * gait pattern generation for the quadruped robot. The system:
 * 
 * 1. Processes state information from sensors and external commands
 * 2. Manages a state machine for each foot (preinit -> init -> stance <-> swing)
 * 3. Calculates support polygons for stability 
 * 4. Generates step targets using Raibert-inspired heuristics
 * 5. Publishes all gait information for other controllers to use
 * 
 * The pattern generator uses a phase-based approach where each foot progresses
 * through its gait cycle based on timing and contact events.
 */

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

inline void GaitPatternGenerator::run_fsm(double t_curr, double t_gait_start, double t_offset, 
                                         double t_stance, double t_swing, int& state, double& phase)
{
  double t_cycle = t_stance + t_swing;
  
  // State -2: Pre-initialization             
  if (t_curr - t_gait_start < t_offset) {
    state = -2;
    phase = (t_curr - t_gait_start) / t_offset;
  }
  // State -1: Initialization
  else if (t_curr - t_gait_start < t_offset + t_cycle) {
    state = -1;
    phase = (t_curr - t_gait_start - t_offset) / t_cycle;
  } 
  else {
    // Calculate time within cycle after initialization period
    // Fix: Use fmod for floating point modulo operation instead of % operator
    double t_elapsed = t_curr - (t_gait_start + t_offset);
    double t = std::fmod(t_elapsed, t_cycle);
    if (t < 0) {
      t += t_cycle; // Ensure positive result
    }
    
    // State 0: Stance phase
    if (t < t_stance) {
      state = 0;
      phase = t / t_stance;
    } 
    // State 1: Swing phase
    else {
      state = 1;
      phase = (t - t_stance) / t_swing;
    }
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
      hip_positions[0] = Eigen::Vector3d(latest_state_->h1.x, latest_state_->h1.y, latest_state_->h1.z);
      hip_positions[1] = Eigen::Vector3d(latest_state_->h2.x, latest_state_->h2.y, latest_state_->h2.z);
      hip_positions[2] = Eigen::Vector3d(latest_state_->h3.x, latest_state_->h3.y, latest_state_->h3.z);
      hip_positions[3] = Eigen::Vector3d(latest_state_->h4.x, latest_state_->h4.y, latest_state_->h4.z);
      com_velocity = Eigen::Vector3d(
        latest_state_->com_velocity.x,
        latest_state_->com_velocity.y,
        latest_state_->com_velocity.z
      );
    }

    double dt = period.seconds();
    double current_time = time.seconds();

    // Calculate timestep for prediction
    prediction_data_.timestep = prediction_horizon_ / prediction_stages_;
    
    // Clear previous predictions and initialize new arrays
    for (int foot = 0; foot < 4; foot++) {
      prediction_data_.states[foot].resize(prediction_stages_, -2);  // Default to pre-init state
      prediction_data_.phases[foot].resize(prediction_stages_, 0.0);  // Default phase 0.0
    }
    
    // Only generate predictions if we've received the gait start command
    if (gait_start_received_) {
      // For each foot, predict future states and phases
      for (int foot = 0; foot < 4; foot++) {
        auto& foot_info = foot_info_[foot];

        // Predict states and phases over the horizon
        for (int stage = 0; stage < prediction_stages_; stage++) {
          int predicted_state;
          double predicted_phase;

          // Calculate predicted state for this foot at this stage
          double predict_time = current_time + stage * prediction_data_.timestep;
          run_fsm(predict_time, 
              gait_start_time_,  // Use stored gait start time instead of current time
              foot_info.phase_offset, 
              stance_duration_, 
              swing_duration_,
              predicted_state,
              predicted_phase);
          
          // Store prediction
          prediction_data_.states[foot][stage] = predicted_state;
          prediction_data_.phases[foot][stage] = predicted_phase;
        }
        
        // Update current foot state and phase with first prediction
        if (prediction_data_.states[foot].size() > 0) {
          foot_info.state = prediction_data_.states[foot][0];
          foot_info.phase = prediction_data_.phases[foot][0];
        }
        
        // Calculate step targets for feet in stance state going to swing
        if (foot_info.state == 0 && prediction_data_.states[foot].size() > 1 && 
            prediction_data_.states[foot][1] == 1) {
          
          // Calculate step target (only needed for stance->swing transition)
          Eigen::Vector3d term1 = hip_positions[foot]; // Current hip position
          Eigen::Vector3d term2 = swing_duration_/2 * cmd_linear; // Raibert heuristic
          Eigen::Vector3d term3 = sqrt(step_height_/9.81) * (com_velocity-cmd_linear); // Step height term
          
          RCLCPP_DEBUG(get_node()->get_logger(), 
              "Foot %d step planning: term1=[%f, %f, %f], term2=[%f, %f, %f], term3=[%f, %f, %f]",
              foot+1, term1.x(), term1.y(), term1.z(), term2.x(), term2.y(), term2.z(),
              term3.x(), term3.y(), term3.z());
              
          foot_info.step_target = term1 + term2 + term3;
        }
        
        // Update time in current state for UI and debug purposes
        if (foot_info.state > -2) {
          foot_info.time_in_state = current_time - foot_info.state_start_time;
        }
        
        // Update state boundaries for transitioning between states
        if (foot_info.state != prediction_data_.states[foot][0]) {
          foot_info.state_start_time = current_time;
          
          // Set the state end time based on which state we're in
          switch (prediction_data_.states[foot][0]) {
            case -1:  // Init
              foot_info.state_end_time = current_time + foot_info.phase_offset;
              break;
            case 0:   // Stance
              foot_info.state_end_time = current_time + stance_duration_;
              break;
            case 1:   // Swing
              foot_info.state_end_time = current_time + swing_duration_;
              break;
            default:
              foot_info.state_end_time = current_time;
          }
        }
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

      // Set foot states - these indicate which phase of the gait cycle each foot is in
      msg.foot1_state = static_cast<int32_t>(foot_info_[0].state);
      msg.foot2_state = static_cast<int32_t>(foot_info_[1].state);
      msg.foot3_state = static_cast<int32_t>(foot_info_[2].state);
      msg.foot4_state = static_cast<int32_t>(foot_info_[3].state);

      // Set foot phases - these represent the progress through the current state (0.0 to 1.0)
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

      // Set the Center of Mass (COM) position 
      msg.com_position.x = support_center_.x();
      msg.com_position.y = support_center_.y();
      msg.com_position.z = 0.0;

      // Add step height to the message
      msg.step_height = static_cast<float>(step_height_);
      
      // Add prediction data
      // Each array must be flattened into a 1D vector with the format:
      // [foot1_step0, foot1_step1, ..., foot2_step0, foot2_step1, ..., foot4_stepN]
      std::vector<int32_t> predicted_states;
      std::vector<float> predicted_phases;
      
      for (int foot = 0; foot < 4; foot++) {
        for (int step = 0; step < prediction_stages_; step++) {
          predicted_states.push_back(static_cast<int32_t>(prediction_data_.states[foot][step]));
          predicted_phases.push_back(static_cast<float>(prediction_data_.phases[foot][step]));
        }
      }
      
      // Add the prediction arrays to the message
      msg.predicted_states = predicted_states;
      msg.predicted_phases = predicted_phases;
      msg.prediction_stages = prediction_stages_;
      msg.prediction_horizon = static_cast<float>(prediction_horizon_);
      msg.prediction_timestep = static_cast<float>(prediction_data_.timestep);

      // Publish the message in a thread-safe way
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
