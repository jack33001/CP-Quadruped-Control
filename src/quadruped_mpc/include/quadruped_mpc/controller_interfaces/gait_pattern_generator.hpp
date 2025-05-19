#ifndef QUADRUPED_MPC_GAIT_PATTERN_GENERATOR_HPP_
#define QUADRUPED_MPC_GAIT_PATTERN_GENERATOR_HPP_

#include <string>
#include <vector>
#include <memory>
#include <array>
#include <cmath>  // Add this for std::erf
#include <iomanip> // Add this for std::setprecision
#include <sstream> // Add this for std::stringstream
#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "quadruped_msgs/msg/quadruped_state.hpp"
#include "quadruped_msgs/msg/gait_pattern.hpp"
#include "geometry_msgs/msg/twist.hpp"  // Add this for teleop commands
#include "std_msgs/msg/bool.hpp"  // Add this for gait start command
#include "realtime_tools/realtime_publisher.hpp"

namespace quadruped_mpc
{

class GaitPatternGenerator : public controller_interface::ControllerInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using RTPublisher = realtime_tools::RealtimePublisher<quadruped_msgs::msg::GaitPattern>;

  GaitPatternGenerator();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::return_type update(
    const rclcpp::Time & time, 
    const rclcpp::Duration & period) override;
  
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  // Protected member variables for basic controller functionality
  std::vector<std::string> joint_names_;
  std::vector<std::string> state_interface_types_;
  std::vector<std::string> command_interface_types_;

private:
  // Subscriber for state estimates
  rclcpp::Subscription<quadruped_msgs::msg::QuadrupedState>::SharedPtr state_sub_;
  std::shared_ptr<quadruped_msgs::msg::QuadrupedState> latest_state_;
  
  // Subscriber for teleop commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::shared_ptr<geometry_msgs::msg::Twist> latest_cmd_;
  
  // Subscriber for gait start command
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gait_start_sub_;
  std::atomic<bool> gait_start_received_{false};
  
  // Gait parameters from config
  std::vector<int> gait_type_;
  double stance_duration_;
  double swing_duration_;
  double duty_factor_;
  double step_height_;
  double step_length_;
  double step_width_;
  
  // Realtime publisher for gait patterns
  std::unique_ptr<RTPublisher> rt_gait_pub_;
  std::shared_ptr<quadruped_msgs::msg::GaitPattern> gait_msg_;

  // Detailed foot information
  struct FootInfo {
    // Variable values
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};        // Position in world frame
    Eigen::Vector3d step_target{Eigen::Vector3d::Zero()};     // Target position for next step
    bool contact{false};                    // True if foot has hit the ground
    double phase{0.0};                      // Progress through current step (0.0 to 1.0)
    int16_t state{-2};                       // FSM state (-2 = preinit, -1 = init, 0 = stance, 1 = swing)
    double state_start_time{0.0};           // Time when current state started
    double state_end_time{0.0};             // Time when current state will end
    double time_in_state{0.0};              // Time spent in current state
    // Constant values
    double phase_offset{0.0};               // Offset for phase calculation
  };
  std::array<FootInfo, 4> foot_info_;  // FL, FR, RL, RR

  // Support polygon information
  Eigen::Vector2d support_center_{Eigen::Vector2d::Zero()};  // Changed from Vector3d to Vector2d

  // MPC prediction parameters
  int prediction_stages_ = 50;  // Number of prediction steps
  double prediction_horizon_ = 5.0;  // Time horizon for prediction in seconds
  
  // Arrays to store predicted states and phases
  struct PredictionData {
    std::vector<int> states[4];  // States for each foot over prediction horizon
    std::vector<double> phases[4];  // Phases for each foot over prediction horizon
    double timestep = 0.1;  // Default timestep between predictions
  };
  PredictionData prediction_data_;
  
  // Helper function to predict future gait pattern
  void run_fsm(double t_curr, double t_gait_start, double t_offset, 
              double t_stance, double t_swing, int& state, double& phase);

  // Helper function for state transitions
  void transition_state(FootInfo& foot, int new_state, double new_phase, 
                        double current_time, double duration);

  // Function declarations
  bool unpack_state();
  bool update_foot_phase(const rclcpp::Time & time, const rclcpp::Duration & period);
  bool support_polygon();
  bool publish_pattern();
};

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_GAIT_PATTERN_GENERATOR_HPP_
