// // #include <hardware_interface/actuator_interface.hpp>
// #include <hardware_interface/hardware_info.hpp>
// #include <hardware_interface/types/hardware_interface_return_values.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <vector>
// #include <memory>


// #include <chrono>
// #include "sensor_msgs/msg/imu.hpp"
// #include "sensor_msgs/msg/magnetic_field.hpp"
// #include <linux/i2c-dev.h>

#include "i2c_tools.cpp"



// // #include "quadruped_hardware/bno08x.hpp"

#include "quadruped_hardware/ROS2_BNO08x.hpp"


namespace quadruped_hardware {


// bool setup(void) {
//     RCLCPP_INFO(rclcpp::get_logger("ROS2_BNO08X"), "Adafruit BNO08x test!");

//     // Serial.begin(115200);
//     // while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
//     // Serial.println("Adafruit BNO08x test!");
  
//     // Try to initialize!
//     if (!bno08x.begin_I2C()) {
//     //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
//     //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
//         RCLCPP_ERROR(rclcpp::get_logger("ROS2_BNO08X"), "Failed to find BNO08x chip");
//         // Serial.println("Failed to find BNO08x chip");
//        return false;
//     }
//     RCLCPP_INFO(rclcpp::get_logger("ROS2_BNO08X"), "BNO08x Found!");

//     for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
//       Serial.print("Part ");
//       Serial.print(bno08x.prodIds.entry[n].swPartNumber);
//       Serial.print(": Version :");
//       Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
//       Serial.print(".");
//       Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
//       Serial.print(".");
//       Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
//       Serial.print(" Build ");
//       Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
//     }
  
//     setReports();
  
//     Serial.println("Reading events");
//     delay(100);
//     return true;
//     }

// void setReports(void) {
//     Serial.println("Setting desired reports");
//     if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
//         Serial.println("Could not enable game vector");
//     }
//     }

BNO08X::BNO08X():address(10){}

hardware_interface::CallbackReturn BNO08X::on_init(const hardware_interface::HardwareInfo& info){
    int bus;

    /* Open i2c bus /dev/i2c-0 */
    if ((bus = i2c_open("/dev/i2c-0")) == -1) {

        /* Error process */
    }


    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO08X::on_configure(const rclcpp_lifecycle::State& previous_state){
    // Configuration code

    I2CDevice device;
    memset(&device, 0, sizeof(device));

    /* 24C04 */
    device.bus = 0;	/* Bus 0 */
    device.addr = 0x50;	/* Slave address is 0x50, 7-bit */
    device.iaddr_bytes = 1;	/* Device internal address is 1 byte */
    device.page_bytes = 16; /* Device are capable of 16 bytes per page */
    return hardware_interface::CallbackReturn::SUCCESS;
}

// hardware_interface::CallbackReturn BNO08X::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
//     // Cleanup code
//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn BNO08X::on_activate(const rclcpp_lifecycle::State& previous_state) {
 
//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn BNO08X::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
//     // Deactivation code
    
//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// std::vector<hardware_interface::StateInterface> BNO08X::export_state_interfaces() {

//     return std::move(state_interfaces_);
// }

// std::vector<hardware_interface::CommandInterface> BNO08X::export_command_interfaces() {

//     return std::move(command_interfaces_);
// }

// hardware_interface::return_type BNO08X::read(const rclcpp::Time& time, const rclcpp::Duration& period) {

//     delay(10);

//     if (bno08x.wasReset()) {
//         Serial.print("sensor was reset ");
//         setReports();
//     }
    
//     if (! bno08x.getSensorEvent(&sensorValue)) {
//         return;
//     }
    
//     switch (sensorValue.sensorId) {
        
//         case SH2_GAME_ROTATION_VECTOR:
//         Serial.print("Game Rotation Vector - r: ");
//         Serial.print(sensorValue.un.gameRotationVector.real);
//         Serial.print(" i: ");
//         Serial.print(sensorValue.un.gameRotationVector.i);
//         Serial.print(" j: ");
//         Serial.print(sensorValue.un.gameRotationVector.j);
//         Serial.print(" k: ");
//         Serial.println(sensorValue.un.gameRotationVector.k);
//         break;
//     }
    
//     return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type BNO08X::write(const rclcpp::Time& time, const rclcpp::Duration& period) {

//     return hardware_interface::return_type::OK;
// }


}  // namespace quadruped_hardware

// // #include "pluginlib/class_list_macros.hpp"
// // PLUGINLIB_EXPORT_CLASS(quadruped_hardware::CANMotor, hardware_interface::ActuatorInterface)