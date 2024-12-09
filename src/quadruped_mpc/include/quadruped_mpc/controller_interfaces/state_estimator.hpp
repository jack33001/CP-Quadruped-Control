#ifndef QUADRUPED_MPC_STATE_ESTIMATOR_HPP_
#define QUADRUPED_MPC_STATE_ESTIMATOR_HPP_

#include <string>
#include <vector>
#include <memory>
#include <array>

// ROS2 headers
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/string.hpp"

// Project headers
#include "quadruped_mpc/utilities/shared_quadruped_info.hpp"

// Pinocchio headers
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace quadruped_mpc
{

class StateEstimator : public controller_interface::ControllerInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  StateEstimator();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> state_interface_types_;
  
  // Pinocchio model and data
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  
  // Frame IDs for feet
  std::array<pinocchio::FrameIndex, 4> foot_frame_ids_;
  
  // Robot description subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  std::string urdf_string_;
  bool urdf_received_{false};
  
  // Callback for robot description
  void robot_description_callback(const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_STATE_ESTIMATOR_HPP_
