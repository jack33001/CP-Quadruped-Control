// // // #include <hardware_interface/actuator_interface.hpp>
// // #include <hardware_interface/hardware_info.hpp>
// // #include <hardware_interface/types/hardware_interface_return_values.hpp>
// // #include <rclcpp/rclcpp.hpp>
// // #include <vector>
// // #include <memory>


// // #include <chrono>
// // #include "sensor_msgs/msg/imu.hpp"
// // #include "sensor_msgs/msg/magnetic_field.hpp"
// // #include <linux/i2c-dev.h>

// #include "i2c_tools.cpp"



// // // #include "quadruped_hardware/bno08x.hpp"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "quadruped_hardware/ROS2_BNO08x.hpp"
#include "quadruped_hardware/i2c_device.hpp"



using namespace JetBotControl;
namespace quadruped_hardware {

BNO08X::BNO08X():address(10) {}

hardware_interface::CallbackReturn BNO08X::on_init(const hardware_interface::HardwareInfo& info){
    // I2CDevice();


    bno08x_ = std::make_unique< JetBotControl::I2CDevice>();

   


    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO08X::on_configure(const rclcpp_lifecycle::State& previous_state){
    // // Configuration code
    uint8_t reg=0x4A;
    

    // bno08x_ -> enable_feature(bno_msgs::BNO_REPORT_ACCELEROMETER, 20);
    bno08x_ -> enable_feature(0x01, 20);

    // bno08x_ -> enable_feature(bno_msgs::BNO_REPORT_MAGNETOMETER,20 );
    // bno08x_ -> enable_feature(bno_msgs::BNO_REPORT_GYROSCOPE,20 );
    // bno08x_ -> enable_feature(bno_msgs::BNO_REPORT_GAME_ROTATION_VECTOR, 10);
    // bno08x_ -> set_quaternion_euler_vector(bno_msgs::BNO_REPORT_GAME_ROTATION_VECTOR);


    //print the address to check if the device is initialized correctly
    // var= bno08x_->tryReadReg(reg); // Example read to check if the device is responsive
    
    // RCLCPP_ERROR(rclcpp::get_logger("BNO08X"), "NOTE XXXXXXXXXX %02X", static_cast<unsigned int>(var.value()));

    


    // I2CDevice device;
    // memset(&device, 0, sizeof(device));

    // /* 24C04 */
    // device.bus = 0;	/* Bus 0 */
    // device.addr = 0x50;	/* Slave address is 0x50, 7-bit */
    // device.iaddr_bytes = 1;	/* Device internal address is 1 byte */
    // device.page_bytes = 16; /* Device are capable of 16 bytes per page */
    return hardware_interface::CallbackReturn::SUCCESS;
}

// // hardware_interface::CallbackReturn BNO08X::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
// //     // Cleanup code
// //     return hardware_interface::CallbackReturn::SUCCESS;
// // }

// // hardware_interface::CallbackReturn BNO08X::on_activate(const rclcpp_lifecycle::State& previous_state) {
 
// //     return hardware_interface::CallbackReturn::SUCCESS;
// // }

// // hardware_interface::CallbackReturn BNO08X::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
// //     // Deactivation code
    
// //     return hardware_interface::CallbackReturn::SUCCESS;
// // }

// // std::vector<hardware_interface::StateInterface> BNO08X::export_state_interfaces() {

// //     return std::move(state_interfaces_);
// // }

// // std::vector<hardware_interface::CommandInterface> BNO08X::export_command_interfaces() {

// //     return std::move(command_interfaces_);
// // }

hardware_interface::return_type BNO08X::read(const rclcpp::Time& time, const rclcpp::Duration& period) {

    // delay(10);

    // if (bno08x.wasReset()) {
    //     Serial.print("sensor was reset ");
    //     setReports();
    // }
    
    // if (! bno08x.getSensorEvent(&sensorValue)) {
    //     return;
    // }
    
    // switch (sensorValue.sensorId) {
        
    //     case SH2_GAME_ROTATION_VECTOR:
    //     Serial.print("Game Rotation Vector - r: ");
    //     Serial.print(sensorValue.un.gameRotationVector.real);
    //     Serial.print(" i: ");
    //     Serial.print(sensorValue.un.gameRotationVector.i);
    //     Serial.print(" j: ");
    //     Serial.print(sensorValue.un.gameRotationVector.j);
    //     Serial.print(" k: ");
    //     Serial.println(sensorValue.un.gameRotationVector.k);
    //     break;
    // }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type BNO08X::write(const rclcpp::Time& time, const rclcpp::Duration& period) {

    return hardware_interface::return_type::OK;
}


}  // namespace quadruped_hardware

// // // #include "pluginlib/class_list_macros.hpp"
// // // PLUGINLIB_EXPORT_CLASS(quadruped_hardware::CANMotor, hardware_interface::ActuatorInterface)