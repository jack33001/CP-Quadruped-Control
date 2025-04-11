#include <hardware_interface/system_interface.hpp>
#include "quadruped_hardware/bno08x_driver.h" // Include your BNO08x class from above
#include "quadruped_hardware/bno_HAL.hpp" // Include your BNO08x class from above
#include "quadruped_hardware/sh2.h"


namespace quadruped_hardware {
class BNO08X : public hardware_interface::SystemInterface {
private:
    BNO08x sensor_;
    double quaternion_[4] = {0.0, 0.0, 0.0, 0.0};
    
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override {
        // if (!sensor_.begin()) {
        //     RCLCPP_ERROR(rclcpp::get_logger("BNO08X"), "Failed to initialize BNO08x");
        //     return hardware_interface::CallbackReturn::ERROR;
        // }


        static JetsonHal_t jetsonHalInstance;
        static sh2_Hal_t hal = {
            .open = jetsonOpen,
            .close = jetsonClose,
            .read = jetsonRead,
            .write = jetsonWrite,
            .getTimeUs = jetsonGetTimeUs,
        };

        // Initialize the SH2 sensor hub
        if (sh2_open(&hal, NULL, NULL) == SH2_ERR) {
            fprintf(stderr, "Failed to open SH2 sensor hub.\n");
            
        }
            
        // bnostart();





        RCLCPP_INFO(rclcpp::get_logger("BNO08X"), "BNO08x initialized successfully");


        // sensor_.scan();

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "quaternion_w", &quaternion_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "quaternion_x", &quaternion_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "quaternion_y", &quaternion_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "quaternion_z", &quaternion_[3]));
        return state_interfaces;
    }
    
    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override {
        // float quat[4];
        // if (sensor_.readQuaternion(quat)) {
        //     quaternion_[0] = quat[0];
        //     quaternion_[1] = quat[1];
        //     quaternion_[2] = quat[2];
        //     quaternion_[3] = quat[3];
        //     return hardware_interface::return_type::OK;
        // }
        // return hardware_interface::return_type::ERROR;

        return hardware_interface::return_type::OK;
    }
    
    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override {
        // Nothing to write for this sensor
        return hardware_interface::return_type::OK;
    }
};
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(quadruped_hardware::BNO08X, hardware_interface::SystemInterface)