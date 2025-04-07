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

#include "quadruped_hardware/bno08x_driver.h" // Include the BNO08x driver header for the actual implementation


#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "quadruped_hardware/i2c_device.hpp"
// #include "quadruped_hardware/Adafruit_BNO08x.h"
// #include "quadruped_hardware/BNO08x/Adafruit_BNO08x.h"



namespace quadruped_hardware {

class BNO08X : public hardware_interface::SystemInterface {
    public:
        BNO08X();
       

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        // hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        // hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        // // hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        // std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        // hardware_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;



        // bool setup();
        // void setReports();
        std::unique_ptr< JetBotControl::I2CDevice> bno08x_; 
        double address;
        std::optional<uint8_t> var;

    // private:

    //     hardware_interface::HardwareInfo hardware_info_;  ///< Hardware information from URDF.
    //     std::vector<hardware_interface::StateInterface> state_interfaces_;  ///< List of state interfaces.
    //     std::vector<hardware_interface::CommandInterface> command_interfaces_;  ///< List of state interfaces.

};

} // namespace quadruped_hardware

PLUGINLIB_EXPORT_CLASS(quadruped_hardware::BNO08X, hardware_interface::SystemInterface)

#endif // ROS2_BNO08X_HPP_
