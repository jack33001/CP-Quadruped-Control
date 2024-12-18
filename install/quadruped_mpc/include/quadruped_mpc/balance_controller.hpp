#ifndef QUADRUPED__BALANCE_CONTROLLER_HPP_
#define QUADRUPED__BALANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>

namespace balance_controller {

class BalanceController : public controller_interface::ControllerInterface
{
public:
  BalanceController();

  CallbackReturn on_init() override;

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  void read_state_interfaces();

private:
  using Tensor3D = Eigen::Tensor<double, 3>;
  Tensor3D F_star_prev_;

protected:
  void init_pinocchio(const std::string& urdf_str);

  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interfaces_;
  std::vector<std::string> state_interfaces_;
  
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr commands_subscriber_;
  realtime_tools::RealtimeBuffer<std::vector<double>> rt_command_ptr_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_command_interfaces_;
};

}  // namespace balance_controller

#endif  // QUADRUPED__BALANCE_CONTROLLER_HPP_