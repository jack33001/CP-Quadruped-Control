// hardware_interface.cpp
#include "quadruped_hardware/quadruped.hpp"

namespace quadruped_hardware {

hardware_interface::CallbackReturn QuadrupedHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Initialize motors
    motors_.resize(8);
    for (size_t i = 0; i < motors_.size(); i++) {
        motors_[i].can_id = info_.hardware_parameters["motor" + std::to_string(i) + "_can_id"];
    }

    // Initialize IMU
    imu_.i2c_address = std::stoi(info_.hardware_parameters["imu_i2c_address"]);

    // Initialize power board
    power_board_.can_id = std::stoi(info_.hardware_parameters["power_board_can_id"]);

    // Get interface parameters
    can_interface_ = info_.hardware_parameters["can_interface"];
    i2c_interface_ = info_.hardware_parameters["i2c_interface"];

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuadrupedHardware::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    // Initialize CAN bus
    if (!can_bus_.initialize(can_interface_)) {
        RCLCPP_ERROR(rclcpp::get_logger("QuadrupedHardware"), "Failed to initialize CAN bus");
        return CallbackReturn::ERROR;
    }

    // Initialize I2C bus
    if (!i2c_bus_.initialize(i2c_interface_)) {
        RCLCPP_ERROR(rclcpp::get_logger("QuadrupedHardware"), "Failed to initialize I2C bus");
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> QuadrupedHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Export motor state interfaces
    for (size_t i = 0; i < motors_.size(); i++) {
        state_interfaces.emplace_back(
            "motor" + std::to_string(i), "position", &motors_[i].position);
        state_interfaces.emplace_back(
            "motor" + std::to_string(i), "velocity", &motors_[i].velocity);
        state_interfaces.emplace_back(
            "motor" + std::to_string(i), "effort", &motors_[i].effort);
    }

    // Export IMU state interfaces
    state_interfaces.emplace_back("imu", "orientation.x", &imu_.orientation[0]);
    state_interfaces.emplace_back("imu", "orientation.y", &imu_.orientation[1]);
    state_interfaces.emplace_back("imu", "orientation.z", &imu_.orientation[2]);
    state_interfaces.emplace_back("imu", "orientation.w", &imu_.orientation[3]);
    
    // Add angular velocity and linear acceleration interfaces...

    // Export power board state interfaces
    state_interfaces.emplace_back("power_board", "voltage", &power_board_.voltage);
    state_interfaces.emplace_back("power_board", "current", &power_board_.current);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> QuadrupedHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Export motor command interfaces
    for (size_t i = 0; i < motors_.size(); i++) {
        command_interfaces.emplace_back(
            "motor" + std::to_string(i), "position", &motors_[i].position_command);
        command_interfaces.emplace_back(
            "motor" + std::to_string(i), "velocity", &motors_[i].velocity_command);
        command_interfaces.emplace_back(
            "motor" + std::to_string(i), "effort", &motors_[i].effort_command);
    }

    return command_interfaces;
}

hardware_interface::return_type QuadrupedHardware::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // Read from motors via CAN
    for (size_t i = 0; i < motors_.size(); i++) {
        auto data = can_bus_.read(motors_[i].can_id);
        // Parse data into position, velocity, effort...
    }

    // Read from IMU via I2C
    auto imu_data = i2c_bus_.read(imu_.i2c_address);
    // Parse IMU data...

    // Read from power board via CAN
    auto power_data = can_bus_.read(power_board_.can_id);
    // Parse power board data...

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuadrupedHardware::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // Write to motors via CAN
    for (size_t i = 0; i < motors_.size(); i++) {
        std::vector<uint8_t> data;
        // Pack position_command, velocity_command, effort_command into data...
        can_bus_.write(motors_[i].can_id, data);
    }

    return hardware_interface::return_type::OK;
}

} // namespace quadruped_hardware