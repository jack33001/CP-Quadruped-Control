#ifndef LEG_CONTROLLER_HPP_
#define LEG_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace leg_controller_ns
{
class LegController : public controller_interface::ControllerInterface
{
public:
  LegController();
  controller_interface::return_type init(const std::string & controller_name) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update() override;

protected:
  rclcpp::Time last_update_time_;
  rclcpp::Duration update_period_;

  std::vector<std::string> joint_names_;
  std::vector<double> command_values_;
};
}  // namespace leg_controller_ns

#endif  // LEG_CONTROLLER_HPP_
