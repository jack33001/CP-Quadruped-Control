#ifndef QUADRUPED_MUJOCO_HARDWARE_INTERFACE_HPP_
#define QUADRUPED_MUJOCO_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

namespace quadruped
{

class MujocoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MujocoHardwareInterface)  // Removed extra semicolon

  MujocoHardwareInterface();
  ~MujocoHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // MuJoCo-related members
  mjModel* mj_model;
  mjData* mj_data;
  mjvCamera cam;
  mjvOption vopt;
  mjvScene scn;
  mjrContext con;

  // GLFW window
  GLFWwindow* window;

  // Joint states and commands
  std::vector<double> hw_states_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  
};

}  // namespace quadruped

#endif  // QUADRUPED_MUJOCO_HARDWARE_INTERFACE_HPP_