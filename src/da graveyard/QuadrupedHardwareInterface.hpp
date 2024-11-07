#ifndef QUADRUPED_HARDWARE_INTERFACE_HPP
#define QUADRUPED_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <vector>

namespace quadruped
{

class QuadrupedHardwareInterface : public hardware_interface::SystemInterface
{
public:
    QuadrupedHardwareInterface();
    ~QuadrupedHardwareInterface() override;

    // Override methods from SystemInterface
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::vector<hardware_interface::StateInterface> state_interfaces_;
    std::vector<hardware_interface::CommandInterface> command_interfaces_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_effort_command_;
};

}  // namespace quadruped

#endif  // QUADRUPED_HARDWARE_INTERFACE_HPP
