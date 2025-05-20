#ifndef QUADRUPED_HARDWARE_ROS2_CAN_MOTOR_HPP_
#define QUADRUPED_HARDWARE_ROS2_CAN_MOTOR_HPP_

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <sstream>
// #include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <thread>
#include <atomic>
#include <chrono>

#include <cmath>


#include "quadruped_hardware/CANInterface.hpp"
#include "watchdog.hpp"


using namespace CAN_interface;
using namespace std::chrono_literals;
namespace quadruped_hardware {

/**
 * @class ROS2CANInterface
 * @brief A hardware interface for controlling a quadruped robot using CAN communication.
 */
class CANMotor : public hardware_interface::SystemInterface {
public:
    CANMotor();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    std::map<int, motor_driver::motorState> send_motor_cmd();
    void startThread();
    void stopThread();
    void threadLoop();
    void on_timeout();

    


private:

    std::atomic<bool> running_;
    std::thread thread_;

    hardware_interface::HardwareInfo hardware_info_;  ///< Hardware information from URDF.
    std::unique_ptr<motor_driver::MotorDriver> motor_controller_;  ///< Motor controller for managing motors.
    
    std::vector<hardware_interface::StateInterface> state_interfaces_;  ///< List of state interfaces.
    std::vector<hardware_interface::CommandInterface> command_interfaces_;  ///< List of state interfaces.
    
    std::vector<int> can_id;
    std::string joint_name;

    double effort_limit;
    
    double cmd_position;
    double cmd_velocity;
    double cmd_effort;
    double cmd_kp;
    double cmd_kd;
    double cmd_m_state; // 0 for Ready, 1 for deactivate, 2 activate, 3 zero 
    double cmd_flip; //multiplier for flip cmd

    double state_position;
    double state_velocity;
    double state_effort;
    double state_kp;
    double state_kd;
    double state_m_state;
    double state_flip;
    double state_watchdog;
    
    std::string command_type;
    double frequency;
    double period;
    std::chrono::duration<double> thread_period;

    double position_offset;

    double min_pos;
    double max_pos;

    std::map<int, motor_driver::motorCommand> commandMap;

    std::map<int, motor_driver::motorState> stateMap;

    motor_driver::motorCommand movecmd ;

    std::unique_ptr<Watchdog>  watchdog;
    // std::optional<Watchdog> wd;
};

} // namespace quadruped_hardware

PLUGINLIB_EXPORT_CLASS(quadruped_hardware::CANMotor, hardware_interface::SystemInterface)

#endif // QUADRUPED_HARDWARE_ROS2_CAN_INTERFACE_HPP_
