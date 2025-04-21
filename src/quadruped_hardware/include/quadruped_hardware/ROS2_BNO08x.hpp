#ifndef QUADRUPED_HARDWARE_ROS2_BNO08X_HPP
#define QUADRUPED_HARDWARE_ROS2_BNO08X_HPP

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


#include <chrono>
#include "std_msgs/msg/string.hpp"


#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp> // Include the JSON library 




namespace quadruped_hardware {

class BNO08X : public hardware_interface::SystemInterface {
    public:
        BNO08X();
       

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        // hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        // hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        // // hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        // hardware_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        // std::ifstream usbPort("/dev/ttyACM0");
    
        std::string jsonString;
            // BNO08x sensor_;
        double quaternion_[4] = {0.0, 0.0, 0.0, 0.0};
        std::ifstream usbPort;
        



        // bool setup();
        // void setReports();
        

    // private:

    //     hardware_interface::HardwareInfo hardware_info_;  ///< Hardware information from URDF.
    //     std::vector<hardware_interface::StateInterface> state_interfaces_;  ///< List of state interfaces.
    //     std::vector<hardware_interface::CommandInterface> command_interfaces_;  ///< List of state interfaces.

};

} // namespace quadruped_hardware

PLUGINLIB_EXPORT_CLASS(quadruped_hardware::BNO08X, hardware_interface::SystemInterface)

#endif // ROS2_BNO08X_HPP_
