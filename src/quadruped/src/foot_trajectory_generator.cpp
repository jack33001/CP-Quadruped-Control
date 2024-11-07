#include "quadruped/foot_trajectory_generator.hpp"

namespace foot_trajectory_ns
{
FootTrajectoryGenerator::FootTrajectoryGenerator()
: ControllerInterface()
{
  // Initialization code, if needed
}

controller_interface::return_type FootTrajectoryGenerator::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  // Initialize parameters or other setup code here
  joint_names_ = {"ankle_joint", "toe_joint"};  // Example joint names for the foot trajectory
  command_values_.resize(joint_names_.size(), 0.0);

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration FootTrajectoryGenerator::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {"ankle_joint/position", "toe_joint/position"};  // Replace with actual command interfaces
  return config;
}

controller_interface::InterfaceConfiguration FootTrajectoryGenerator::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {"ankle_joint/position", "toe_joint/position"};  // Replace with actual state interfaces
  return config;
}

controller_interface::return_type FootTrajectoryGenerator::update()
{
  // Example update logic for trajectory generation
  rclcpp::Time current_time = node_->get_clock()->now();
  rclcpp::Duration elapsed_time = current_time - last_update_time_;
  last_update_time_ = current_time;

  for (size_t i = 0; i < command_values_.size(); ++i) {
    // Command processing for trajectory, e.g., simple oscillation as placeholder logic
    command_values_[i] = 0.1 * sin(current_time.seconds());  // Placeholder trajectory logic
  }

  return controller_interface::return_type::OK;
}

}  // namespace foot_trajectory_ns

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(foot_trajectory_ns::FootTrajectoryGenerator, controller_interface::ControllerInterface)
