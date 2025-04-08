#ifndef QUADRUPED_MPC_FOOT_CONTROLLER_HPP
#define QUADRUPED_MPC_FOOT_CONTROLLER_HPP

#include <string>
#include <vector>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

// Add these headers for the FootForces message
#include "quadruped_msgs/msg/foot_forces.hpp"
#include "geometry_msgs/msg/vector3.hpp"
// Add state and gait pattern message includes
#include "quadruped_msgs/msg/quadruped_state.hpp"
#include "quadruped_msgs/msg/gait_pattern.hpp"
#include <mutex>

namespace quadruped_mpc
{

// Forward declare the control law functions so they can be friends
inline bool swing_trajectory(class FootController& controller, const rclcpp::Time& current_time);
inline bool apply_jacobians(class FootController& controller);

class FootController : public controller_interface::ControllerInterface
{
public:
  FootController();
  
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Make control law functions friends so they can access private members
  friend bool swing_trajectory(FootController& controller, const rclcpp::Time& current_time);
  friend bool apply_jacobians(FootController& controller);
  
private:
  // Parameters
  std::vector<std::string> joint_names_;
  std::string controller_name_;
  
  // Hardware interfaces
  std::vector<std::string> command_interface_types_;
  
  // Interfaces
  std::vector<hardware_interface::LoanedCommandInterface> joint_command_interfaces_;
  
  // Control-related variables
  size_t num_joints_;
  
  // Helpers
  bool reset();
  
  // Subscriber for foot force commands
  rclcpp::Subscription<quadruped_msgs::msg::FootForces>::SharedPtr foot_forces_subscription_;
  std::shared_ptr<realtime_tools::RealtimeBuffer<quadruped_msgs::msg::FootForces>> foot_forces_buffer_;
  
  // Storage for foot forces
  struct FootForces {
    geometry_msgs::msg::Vector3 foot1;  // Front left
    geometry_msgs::msg::Vector3 foot2;  // Front right
    geometry_msgs::msg::Vector3 foot3;  // Rear left
    geometry_msgs::msg::Vector3 foot4;  // Rear right
    bool has_data;
  };
  FootForces foot_forces_;
  
  // Subscriber callback
  void footForcesCallback(const quadruped_msgs::msg::FootForces::SharedPtr msg);
  
  // Add state estimation subscriber and buffer
  rclcpp::Subscription<quadruped_msgs::msg::QuadrupedState>::SharedPtr state_subscription_;
  std::shared_ptr<realtime_tools::RealtimeBuffer<quadruped_msgs::msg::QuadrupedState>> state_buffer_;
  std::mutex state_mutex_;
  quadruped_msgs::msg::QuadrupedState latest_state_; // Changed from SharedPtr
  
  // Add gait pattern subscriber and buffer
  rclcpp::Subscription<quadruped_msgs::msg::GaitPattern>::SharedPtr gait_subscription_;
  std::shared_ptr<realtime_tools::RealtimeBuffer<quadruped_msgs::msg::GaitPattern>> gait_buffer_;
  std::mutex gait_mutex_;
  quadruped_msgs::msg::GaitPattern latest_gait_; // Changed from SharedPtr
  
  // Add callback functions
  void stateCallback(const quadruped_msgs::msg::QuadrupedState::SharedPtr msg);
  void gaitCallback(const quadruped_msgs::msg::GaitPattern::SharedPtr msg);
};

} // namespace quadruped_mpc

#endif // QUADRUPED_MPC_FOOT_CONTROLLER_HPP
