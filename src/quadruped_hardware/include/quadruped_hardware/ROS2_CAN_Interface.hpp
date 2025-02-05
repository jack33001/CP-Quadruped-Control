#ifndef QUADRUPED_HARDWARE_ROS2_CAN_INTERFACE_HPP_
#define QUADRUPED_HARDWARE_ROS2_CAN_INTERFACE_HPP_

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
class ROS2CANInterface : public hardware_interface::SystemInterface {
public:
    ROS2CANInterface();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    // hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;



    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

   

    


private:
    hardware_interface::HardwareInfo hardware_info_;  ///< Hardware information from URDF.
    std::unique_ptr<CAN_interface::CANInterface> can_interface_;  ///< CAN interface for communication.
    
    // CAN_interface::CANInterface can_interface;
    
    std::unordered_map<std::string, std::map<size_t, uint8_t>> can_state_map_;  ///< State values mapped by CAN IDs.
    std::unordered_map<std::string, std::map<size_t, uint8_t>> can_command_map_; // Change the type to store 8-byte arrays
    
    std::vector<hardware_interface::StateInterface> state_interfaces_;  ///< List of state interfaces.
    std::unordered_map<std::string, hardware_interface::StateInterface> state_interfaces_map_;  ///< State interfaces mapped by name.
    std::unordered_map<std::string, hardware_interface::CommandInterface> command_interfaces_map_;  ///< Command interfaces mapped by name.
    


    // Command data buffer (8-byte array for commands).
    std::vector<uint8_t> command_data_;

    // State data buffer (8-byte array for state information).
    std::vector<uint8_t> state_data_;

    std::atomic<bool> running;
    void readthread(std::atomic<bool>& running);


    };

} // namespace quadruped_hardware

PLUGINLIB_EXPORT_CLASS(quadruped_hardware::ROS2CANInterface, hardware_interface::SystemInterface)

#endif // QUADRUPED_HARDWARE_ROS2_CAN_INTERFACE_HPP_
