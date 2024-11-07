#ifndef BALANCE_CONTROLLER_HPP_
#define BALANCE_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace balance_controller_ns
{
class BalanceController : public controller_interface::ControllerInterface
{
public:
  BalanceController();
  ~BalanceController() override = default;

  controller_interface::return_type init(const std::string & controller_name) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update() override;

protected:
  rclcpp::Time last_update_time_;
  rclcpp::Duration update_period_;

  std::vector<std::string> joint_names_;
  std::vector<double> command_values_;

  // Add any additional parameters or state variables here
};
}  // namespace balance_controller_ns

#endif  // BALANCE_CONTROLLER_HPP_
