// #include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <memory>

// #include "quadruped_hardware/bno08x.hpp"

#include "quadruped_hardware/ROS2_BNO08X.hpp"


namespace quadruped_hardware {


BNO08X::BNO08X():address(10){}

hardware_interface::CallbackReturn BNO08X::on_init(const hardware_interface::HardwareInfo& info){
    // Initialization code
    hardware_interface::ComponentInfo config;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO08X::on_configure(const rclcpp_lifecycle::State& previous_state){
    // Configuration code
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO08X::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    // Cleanup code
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO08X::on_activate(const rclcpp_lifecycle::State& previous_state) {
 
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO08X::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    // Deactivation code
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BNO08X::export_state_interfaces() {

    return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> BNO08X::export_command_interfaces() {

    return std::move(command_interfaces_);
}

hardware_interface::return_type BNO08X::read(const rclcpp::Time& time, const rclcpp::Duration& period) {

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type BNO08X::write(const rclcpp::Time& time, const rclcpp::Duration& period) {

    return hardware_interface::return_type::OK;
}


}  // namespace quadruped_hardware

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(quadruped_hardware::CANMotor, hardware_interface::ActuatorInterface)