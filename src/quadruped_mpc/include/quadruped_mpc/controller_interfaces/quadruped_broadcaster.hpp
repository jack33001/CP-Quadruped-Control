#ifndef QUADRUPED_BROADCASTER_HPP_
#define QUADRUPED_BROADCASTER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "quadruped_mpc/utilities/shared_quadruped_info.hpp"

namespace quadruped_mpc
{

class QuadrupedBroadcaster : public controller_interface::ControllerInterface
{
public:
  QuadrupedBroadcaster();
  
  controller_interface::CallbackReturn on_init() override;
  
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
    
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
    
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  std::vector<std::string> joint_names_;
  std::string imu_name_;
  
  // Hardware params
  double gravity_;
  double hardware_height_;
  double hardware_width_;
  double hardware_length_;
  
  // Balance controller params
  double balance_m_;
  double balance_kp_pos_;
  double balance_kd_pos_;
  double balance_kp_rot_;
  double balance_kd_rot_;
  
  // Desired states
  std::vector<double> p_c_des_;
  std::vector<double> v_c_des_;
  std::vector<double> th_c_des_;  // Now stores 4 values for quaternion (w,x,y,z)
  std::vector<double> om_c_des_;
};

} // namespace quadruped_mpc

#endif  // QUADRUPED_BROADCASTER_HPP_
