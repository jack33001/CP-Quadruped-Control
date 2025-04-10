// // // // #include <hardware_interface/actuator_interface.hpp>
// // // #include <hardware_interface/hardware_info.hpp>
// // // #include <hardware_interface/types/hardware_interface_return_values.hpp>
// // // #include <rclcpp/rclcpp.hpp>
// // // #include <vector>
// // // #include <memory>


// // // #include <chrono>
// // // #include "sensor_msgs/msg/imu.hpp"
// // // #include "sensor_msgs/msg/magnetic_field.hpp"
// // // #include <linux/i2c-dev.h>

// // #include "i2c_tools.cpp"



// // // // #include "quadruped_hardware/bno08x.hpp"
// #include <fcntl.h>
// #include <linux/i2c-dev.h>
// #include <linux/i2c.h>
// #include <linux/types.h>
// #include <sys/ioctl.h>
// #include <unistd.h>
// #include <stdio.h>
// #include <stdlib.h>

// #include "quadruped_hardware/ROS2_BNO08x.hpp"
// #include "quadruped_hardware/i2c_device.hpp"



// using namespace JetBotControl;
// namespace quadruped_hardware {

// BNO08X::BNO08X():address(10) {}

// hardware_interface::CallbackReturn BNO08X::on_init(const hardware_interface::HardwareInfo& info){
//     // I2CDevice();

//     // int file;
//     // const char *filename = "/dev/i2c-7"; // Replace with your I2C bus device
//     // int i2c_address = 0x4a; // Replace with the I2C address of your device
//     // char buf[2] = {0};

//     // // 1. Open the I2C bus
//     // if ((file = open(filename, O_RDWR)) < 0) {
//     //     perror("Failed to open the i2c bus");
//     //     exit(1);
//     // }

//     // // 2. Set the I2C slave address
//     // if (ioctl(file, I2C_SLAVE, i2c_address) < 0) {
//     //     printf("Failed to acquire bus access and/or talk to slave.\n");
//     //     close(file);
//     //     exit(1);
//     // }

//     // // 3. Example: Write a byte
//     // buf[0] = 0x01; // Data to write
//     // if (write(file, buf, 1) != 1) {
//     //     perror("Failed to write to the i2c bus");
//     //     close(file);
//     //     exit(1);
//     // }
//     // printf("Successfully wrote 0x01 to I2C device at address 0x%02X\n", i2c_address);

//     // // 4. Example: Read a byte
//     // if (read(file, buf, 1) != 1) {
//     //     perror("Failed to read from the i2c bus");
//     //     close(file);
//     //     exit(1);
//     // }
//     // printf("Successfully read 0x%02X from I2C device at address 0x%02X\n", buf[0], i2c_address);

//     // // 5. Close the I2C bus
//     // close(file);


//     bno08x_ = std::make_unique< JetBotControl::I2CDevice>();




//     // int i2c_fd_;
//     // i2c_fd_ = open("/dev/i2c-1", O_RDWR);

//     // std::cout << "I2C file descriptor: " << i2c_fd_ << std::endl;
//     // if (i2c_fd_ < 0) {
//     //     throw std::runtime_error("Failed to open I2C interface!");
//     // }


//     // if(ioctl(i2c_fd_, I2C_SLAVE_FORCE, 0X4A) < 0){
//     //     close(i2c_fd_);
//     //     throw std::runtime_error("Failed to set slave address!");
        
//     // }

   


//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn BNO08X::on_configure(const rclcpp_lifecycle::State& previous_state){
//     // // Configuration code
//     uint8_t reg=0x4A;
    
//     uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
//     int length = sizeof(data) / sizeof(data[0]);

//     try {
//         int bytes_written = bno08x_->i2c_write_array(data, length);
//         std::cout << "Successfully wrote " << bytes_written << " bytes to the I2C device." << std::endl;
        
//         uint8_t reg = 0x00;
//         auto result = bno08x_->tryReadReg(reg);


//     if (result.has_value()) {
//         bytes_written = static_cast<int>(result.value());  // Access the value and cast it to int
//     } else {
//         RCLCPP_ERROR(rclcpp::get_logger("BNO08X"), "Failed to read register: %d", reg);
//         return hardware_interface::CallbackReturn::ERROR;  // Handle the error appropriately
//     }



//         std::cout << "Successfully read " << bytes_written << " from the I2C device." << std::endl;


//     } catch (const std::exception& e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//     }


    

//     // bno08x_ -> enable_feature(bno_msgs::BNO_REPORT_ACCELEROMETER, 20);
//     // bno08x_ -> enable_feature(0x01, 20);
//     // bno08x_ -> enable_feature(0x02, 20);
//     // bno08x_ -> enable_feature(0x03, 20);
//     // bno08x_ -> enable_feature(0x08, 20);

//     // bno08x_ -> enable_feature(bno_msgs::BNO_REPORT_MAGNETOMETER,20 );
//     // bno08x_ -> enable_feature(bno_msgs::BNO_REPORT_GYROSCOPE,20 );
//     // bno08x_ -> enable_feature(bno_msgs::BNO_REPORT_GAME_ROTATION_VECTOR, 10);
//     // bno08x_ -> set_quaternion_euler_vector(bno_msgs::BNO_REPORT_GAME_ROTATION_VECTOR);


//     //print the address to check if the device is initialized correctly
//     // var= bno08x_->tryReadReg(reg); // Example read to check if the device is responsive
    
//     // RCLCPP_ERROR(rclcpp::get_logger("BNO08X"), "NOTE XXXXXXXXXX %02X", static_cast<unsigned int>(var.value()));

    


//     // I2CDevice device;
//     // memset(&device, 0, sizeof(device));

//     // /* 24C04 */
//     // device.bus = 0;	/* Bus 0 */
//     // device.addr = 0x50;	/* Slave address is 0x50, 7-bit */
//     // device.iaddr_bytes = 1;	/* Device internal address is 1 byte */
//     // device.page_bytes = 16; /* Device are capable of 16 bytes per page */
//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// // // hardware_interface::CallbackReturn BNO08X::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
// // //     // Cleanup code
// // //     return hardware_interface::CallbackReturn::SUCCESS;
// // // }

// // // hardware_interface::CallbackReturn BNO08X::on_activate(const rclcpp_lifecycle::State& previous_state) {
 
// // //     return hardware_interface::CallbackReturn::SUCCESS;
// // // }

// // // hardware_interface::CallbackReturn BNO08X::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
// // //     // Deactivation code
    
// // //     return hardware_interface::CallbackReturn::SUCCESS;
// // // }

// // // std::vector<hardware_interface::StateInterface> BNO08X::export_state_interfaces() {

// // //     return std::move(state_interfaces_);
// // // }

// // // std::vector<hardware_interface::CommandInterface> BNO08X::export_command_interfaces() {

// // //     return std::move(command_interfaces_);
// // // }

// hardware_interface::return_type BNO08X::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
//     // bno08x_ -> tryReadReg(0x4A);
//     // delay(10);

//     // if (bno08x.wasReset()) {
//     //     Serial.print("sensor was reset ");
//     //     setReports();
//     // }
    
//     // if (! bno08x.getSensorEvent(&sensorValue)) {
//     //     return;
//     // }
    
//     // switch (sensorValue.sensorId) {
        
//     //     case SH2_GAME_ROTATION_VECTOR:
//     //     Serial.print("Game Rotation Vector - r: ");
//     //     Serial.print(sensorValue.un.gameRotationVector.real);
//     //     Serial.print(" i: ");
//     //     Serial.print(sensorValue.un.gameRotationVector.i);
//     //     Serial.print(" j: ");
//     //     Serial.print(sensorValue.un.gameRotationVector.j);
//     //     Serial.print(" k: ");
//     //     Serial.println(sensorValue.un.gameRotationVector.k);
//     //     break;
//     // }
    
//     return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type BNO08X::write(const rclcpp::Time& time, const rclcpp::Duration& period) {

//     return hardware_interface::return_type::OK;
// }


// }  // namespace quadruped_hardware

// // // // #include "pluginlib/class_list_macros.hpp"
// // // // PLUGINLIB_EXPORT_CLASS(quadruped_hardware::CANMotor, hardware_interface::ActuatorInterface)