#ifndef QUADRUPED_MPC_GAIT_PATTERN_GENERATOR_HPP_
#define QUADRUPED_MPC_GAIT_PATTERN_GENERATOR_HPP_

#include <string>
#include <vector>
#include <memory>
#include <array>
#include <cmath>  // Add this for std::erf
#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "quadruped_msgs/msg/quadruped_state.hpp"
#include "quadruped_msgs/msg/gait_pattern.hpp"
#include "quadruped_msgs/msg/foot_states.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace quadruped_mpc
{

class GaitPatternGenerator : public controller_interface::ControllerInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using RTPublisher = realtime_tools::RealtimePublisher<quadruped_msgs::msg::GaitPattern>;
  using RTFootStatePublisher = realtime_tools::RealtimePublisher<quadruped_msgs::msg::FootStates>;

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
  
  // Realtime publisher for gait patterns
  std::unique_ptr<RTPublisher> rt_gait_pub_;
  std::shared_ptr<quadruped_msgs::msg::GaitPattern> gait_msg_;

  // Additional realtime publisher for foot states
  std::unique_ptr<RTFootStatePublisher> rt_foot_state_pub_;
  std::shared_ptr<quadruped_msgs::msg::FootStates> foot_state_msg_;

  // Detailed foot information
  struct FootInfo {
    Eigen::Vector3d position;        // Position in world frame
    Eigen::Vector3d step_target;     // Target position for next step
    bool contact;                    // True if foot has hit the ground
    double phase;                    // Progress through current step (0.0 to 1.0)
    int16_t state;                   // FSM state (0 = stance, 1 = swing)
  };
  std::array<FootInfo, 4> foot_info_;  // FL, FR, RL, RR

  // Support polygon information
  Eigen::Vector2d support_center_{Eigen::Vector2d::Zero()};  // Changed from Vector3d to Vector2d

  // Function declarations
  bool unpack_state();
  bool update_foot_phase();
  bool support_polygon();
  bool publish_pattern();
};

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_GAIT_PATTERN_GENERATOR_HPP_
