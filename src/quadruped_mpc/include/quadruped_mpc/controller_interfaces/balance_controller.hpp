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
#include "quadruped_ode_model/acados_c_types.h"  // Add this before ACADOS headers
#include "quadruped_ode_model/quadruped_ode_model.h"  // Generated ACADOS model header
#include "acados_solver_quadruped_ode.h"              // Generated ACADOS solver header

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
  quadruped_ode_solver_capsule* solver_{nullptr};
  std::array<double,12> current_state_;
  std::array<double,12> desired_state_;
  std::array<double,12> optimal_control_;
  
  // Restore foot position storage
  std::array<double,3> p1_, p2_, p3_, p4_, com_;

private:
  // Remove logger_ member variable since we'll use get_node()->get_logger() directly
};

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_BALANCE_CONTROLLER_HPP_
