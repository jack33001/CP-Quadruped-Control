#ifndef QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_
#define QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"  // Add this include
#include "quadruped_mpc/utilities/shared_quadruped_info.hpp"
#include "hardware_interface/handle.hpp"

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
  
  // These are the interfaces we get from the controller manager
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;
};

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_
