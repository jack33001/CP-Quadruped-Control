#ifndef QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_
#define QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "acados/utils/types.h"
#include "quadruped_mpc/acados_generated/quadruped_ode_model/quadruped_ode_model.h"
#include "quadruped_mpc/acados_generated/acados_solver_quadruped_ode.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"  // Add Twist message include
#include "quadruped_msgs/msg/quadruped_state.hpp"
#include "quadruped_msgs/msg/gait_pattern.hpp"
#include "quadruped_msgs/msg/foot_forces.hpp"  // Add FootForces message include
#include "realtime_tools/realtime_publisher.hpp"  // Add realtime publisher include
#include <mutex>

namespace quadruped_mpc
{

class BalanceController : public controller_interface::ControllerInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;  // Add this using declaration

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
  quadruped_ode_solver_capsule* solver_{nullptr};  // Fix type name to match generated code
  std::array<double,25> current_state_;
  std::array<double,25> desired_state_;
  std::array<double,25> optimal_control_;
  
  // Restore foot position storage
  std::array<double,3> p1_, p2_, p3_, p4_, com_;
  
  // Add foot state tracking
  int foot1_state_{0};
  int foot2_state_{0};
  int foot3_state_{0};
  int foot4_state_{0};

private:
  // Replace cmd_sub_ with separate pose and twist subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_sub_;
  std::mutex cmd_mutex_;
  geometry_msgs::msg::Pose::SharedPtr latest_pose_cmd_;
  geometry_msgs::msg::Twist::SharedPtr latest_twist_cmd_;
  bool new_pose_cmd_received_{false};
  bool new_twist_cmd_received_{false};
  
  // Add state handling members
  rclcpp::Subscription<quadruped_msgs::msg::QuadrupedState>::SharedPtr state_sub_;
  quadruped_msgs::msg::QuadrupedState::SharedPtr latest_state_;
  std::mutex state_mutex_;
  bool new_state_received_{false};
  
  // Add gait pattern subscription members
  rclcpp::Subscription<quadruped_msgs::msg::GaitPattern>::SharedPtr gait_sub_;
  std::shared_ptr<quadruped_msgs::msg::GaitPattern> latest_gait_;
  std::mutex gait_mutex_;
  bool new_gait_received_{false};
  
  // Add FootForces publisher
  std::unique_ptr<realtime_tools::RealtimePublisher<quadruped_msgs::msg::FootForces>> foot_forces_publisher_;
  rclcpp::Publisher<quadruped_msgs::msg::FootForces>::SharedPtr foot_forces_pub_;
  
  // Replace cmd_callback with separate callbacks for pose and twist
  void pose_cmd_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void twist_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void state_callback(const quadruped_msgs::msg::QuadrupedState::SharedPtr msg);
  void gait_callback(const quadruped_msgs::msg::GaitPattern::SharedPtr msg);

  // Add new private methods
  bool update_state();
  bool update_control();
  bool update_commands();
};

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_
