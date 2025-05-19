#ifndef QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_
#define QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "acados/utils/types.h"
#include "quadruped_mpc/acados_generated/quadruped_ode_model/quadruped_ode_model.h"
#include "quadruped_mpc/acados_generated/acados_solver_quadruped_ode.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "quadruped_msgs/msg/quadruped_state.hpp"
#include "quadruped_msgs/msg/gait_pattern.hpp"
#include "quadruped_msgs/msg/foot_forces.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace quadruped_mpc
{

class BalanceController : public controller_interface::ControllerInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  BalanceController();
  
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<std::string> joint_names_;
  quadruped_ode_solver_capsule* solver_{nullptr};
  std::array<double,25> current_state_;
  std::array<double,25> desired_state_;
  std::array<double,25> optimal_control_;
  
  // COM offset parameters
  std::vector<double> com_offset_{0.0, 0.0, -0.1};  // Default COM offset [x, y, z]
  
  // Foot position storage - convert to a 2D array for more efficient access
  std::array<std::array<double,3>, 5> foot_positions_; // 4 feet + COM at index 4
  // Foot state tracking - use an array for more consistent access
  std::array<int, 4> foot_states_{0, 0, 0, 0};
  
  // 2D arrays for tracking foot states and phases across multiple stages
  std::vector<std::array<int, 4>> foot_states_prediction_;  // [stages][4 feet]
  std::vector<std::array<float, 4>> foot_phases_prediction_; // [stages][4 feet]

private:
  // Command subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_sub_;
  std::mutex cmd_mutex_;
  geometry_msgs::msg::Pose::SharedPtr latest_pose_cmd_;
  geometry_msgs::msg::Twist::SharedPtr latest_twist_cmd_;
  bool new_pose_cmd_received_{false};
  bool new_twist_cmd_received_{false};
  
  // State handling
  rclcpp::Subscription<quadruped_msgs::msg::QuadrupedState>::SharedPtr state_sub_;
  quadruped_msgs::msg::QuadrupedState::SharedPtr latest_state_;
  std::mutex state_mutex_;
  bool new_state_received_{false};
  
  // Gait pattern subscription
  rclcpp::Subscription<quadruped_msgs::msg::GaitPattern>::SharedPtr gait_sub_;
  std::shared_ptr<quadruped_msgs::msg::GaitPattern> latest_gait_;
  std::mutex gait_mutex_;
  bool new_gait_received_{false};
  
  // FootForces publisher
  std::unique_ptr<realtime_tools::RealtimePublisher<quadruped_msgs::msg::FootForces>> foot_forces_publisher_;
  rclcpp::Publisher<quadruped_msgs::msg::FootForces>::SharedPtr foot_forces_pub_;
  
  // Subscription callbacks
  void pose_cmd_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void twist_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void state_callback(const quadruped_msgs::msg::QuadrupedState::SharedPtr msg);
  void gait_callback(const quadruped_msgs::msg::GaitPattern::SharedPtr msg);

  // Controller methods
  bool update_state();
  bool update_control();
  bool update_commands();
  
  // Debug/logging methods
  void print_controller_output_table();
  void print_state_vector_table();
};

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_
