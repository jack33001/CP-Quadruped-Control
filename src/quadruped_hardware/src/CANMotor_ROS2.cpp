#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <memory>

#include "quadruped_hardware/CANMotor_ROS2.hpp"


namespace quadruped_hardware {



CANMotor::CANMotor(){}


hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info){
    // Initialization code
    std::string joint_name = info.hardware_parameters.at("joint_name");

    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
    config.names.push_back(joint_name + "/effort");

    int motor_id = info.hardware_parameters.at("can_id");
    std::string can_bus = info.hardware_parameters.at("can_bus");



    motor_driver::MotorDriver motor_controller(
        motor_id, can_bus, motor_driver::MotorType::GIM8108);

    

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state){



    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    // Cleanup code
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) {
    // Activation code
    auto start_state = motor_controller.enableMotor(motor_id);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    // Deactivation code
    running_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> export_state_interfaces() {
    // Export state interfaces
    return {};
}

std::vector<hardware_interface::CommandInterface> export_command_interfaces() {

    // Export command interfaces
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        config.names[0], "position", &command_position_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        config.names[1], "velocity", &command_velocity_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        config.names[2], "effort", &command_effort_));
        
    return command_interfaces;
}

hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // Read data from the hardware





    return hardware_interface::return_type::OK;
}

hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // Write data to the hardware
    return hardware_interface::return_type::OK;
}


}  // namespace quadruped_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_hardware::CANMotor, hardware_interface::ActuatorInterface)