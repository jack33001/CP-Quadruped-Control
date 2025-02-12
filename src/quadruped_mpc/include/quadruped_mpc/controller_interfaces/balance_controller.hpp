#ifndef QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_
#define QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "quadruped_mpc/utilities/shared_quadruped_info.hpp"
#include "hardware_interface/handle.hpp"
#include "acados/utils/types.h"
#include "quadruped_mpc/acados_generated/quadruped_ode_model/quadruped_ode_model.h"
#include "quadruped_mpc/acados_generated/acados_solver_quadruped_ode.h"
#include "geometry_msgs/msg/pose.hpp"
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

private:
  // Add command handling members
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_sub_;
  std::mutex cmd_mutex_;
  geometry_msgs::msg::Pose::SharedPtr latest_cmd_;
  bool new_cmd_received_{false};
  
  // Add callback function declaration
  void cmd_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

  // Add new private methods
  void update_state();
  void update_control();
  void update_commands();
};

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_
