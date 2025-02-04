#ifndef QUADRUPED_HARDWARE_CANMOTOR_ROS2_HPP_
#define QUADRUPED_HARDWARE_CANMOTOR_ROS2_HPP_

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <sstream>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "quadruped_hardware/CANInterface.hpp"


using namespace CAN_interface;
namespace quadruped_hardware {

/**
 * @class ROS2CANInterface
 * @brief A hardware interface for controlling a quadruped robot using CAN communication.
 */
class CANMotor : public hardware_interface::ActuatorInterface {
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

   

    


private:
    hardware_interface::HardwareInfo hardware_info_;  ///< Hardware information from URDF.
    std::unique_ptr<CAN_interface::CANInterface> can_interface_;  ///< CAN interface for communication.

    
    
    // Command data buffer (8-byte array for commands).
    std::vector<uint8_t> command_data_;

    // State data buffer (8-byte array for state information).
    std::vector<uint8_t> state_data_;


    };

} // namespace quadruped_hardware

PLUGINLIB_EXPORT_CLASS(quadruped_hardware::ROS2CANInterface, hardware_interface::ActuatorInterface)

#endif // QUADRUPED_HARDWARE_ROS2_CAN_INTERFACE_HPP_
