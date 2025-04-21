#include <hardware_interface/system_interface.hpp>


#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"


#include "ROS2_BNO08x.hpp"


using json = nlohmann::json;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */


namespace quadruped_hardware {
    
BNO08X::BNO08X (): usbPort("/dev/ttyACM0") {}



    hardware_interface::CallbackReturn BNO08X::on_init(const hardware_interface::HardwareInfo& info)  {
        
        // std::ifstream usbPort("/dev/ttyACM0"); // Replace with "COMx" for Windows
        // std::ifstream usbPort("COM5");
        if (!usbPort.is_open()) {
            std::cerr << "Failed to open USB port!" << std::endl;
            return hardware_interface::CallbackReturn::FAILURE;
        }


        RCLCPP_INFO(rclcpp::get_logger("BNO08X"), "Serial connection initialized successfully");


        

        return hardware_interface::CallbackReturn::SUCCESS;
    }

 
    hardware_interface::CallbackReturn BNO08X::on_configure(const rclcpp_lifecycle::State& previous_state)  {
        RCLCPP_INFO(rclcpp::get_logger("BNO08X"), "Configuring BNO08X sensor");
        // if (!setup()) {
        //     RCLCPP_ERROR(rclcpp::get_logger("BNO08X"), "Failed to configure BNO08X sensor");
        //     return hardware_interface::CallbackReturn::FAILURE;
        // }
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    std::vector<hardware_interface::StateInterface> BNO08X::export_state_interfaces()  {
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
    
    hardware_interface::return_type BNO08X::read(
        const rclcpp::Time& time, const rclcpp::Duration& period)  {
        
        // Read a line of JSON from the USB port
        if (std::getline(usbPort, jsonString)) {
            try {
                // Parse the JSON string
                json jsonData = json::parse(jsonString);
                std::cout << "json string: "<< jsonString<< std::endl;

                // Print the parsed JSON data
                std::cout << "status: " << jsonData["status"] << std::endl;
                std::cout << "yaw: "    << jsonData["yaw"] << "°" << std::endl;
                std::cout << "pitch: "  << jsonData["pitch"] << "°" << std::endl;
                std::cout << "roll: "   << jsonData["roll"] << "°" << std::endl;
  
                std::cout << "---------------------------------" << std::endl;


                quaternion_[0] = jsonData["status"];
                quaternion_[1] = jsonData["yaw"];
                quaternion_[2] = jsonData["pitch"];
                quaternion_[3] = jsonData["roll"];

                std::cout << "jobs done" << std::endl;

            } catch (const std::exception& e) {
                std::cerr << "Failed to parse JSON: " << e.what() << std::endl;
            }
        }
            
 

        return hardware_interface::return_type::OK;
    }
    
    hardware_interface::return_type BNO08X::write(const rclcpp::Time& time, const rclcpp::Duration& period)  {
        // Nothing to write for this sensor


        return hardware_interface::return_type::OK;
    }
};

