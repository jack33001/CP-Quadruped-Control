// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"


#include <iostream>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp> // Include the JSON library (https://github.com/nlohmann/json)

using json = nlohmann::json;




using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("imu_publisher"), count_(0)
  {

    // two publishers 
    publisher_ = this->create_publisher<std_msgs::msg::String>("pitch", 10);


    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<MinimalPublisher>());
  // rclcpp::shutdown();

    // Open the USB port (adjust the path as needed for your platform)
    std::ifstream usbPort("/dev/ttyACM0"); // Replace with "COMx" for Windows
    // std::ifstream usbPort("COM5");
    if (!usbPort.is_open()) {
        std::cerr << "Failed to open USB port!" << std::endl;
        return 1;
    }

    std::string jsonString;
    while (true) {
        // Read a line of JSON from the USB port
        if (std::getline(usbPort, jsonString)) {
            try {
                // Parse the JSON string
                json jsonData = json::parse(jsonString);
                std::cout << "json string: "<< jsonString<< std::endl;

                // Print the parsed JSON data
                std::cout << "status: " << jsonData["status"] << std::endl;
                std::cout << "yaw: " << jsonData["yaw"] << "°" << std::endl;
                std::cout << "pitch: " << jsonData["pitch"] << "°" << std::endl;
                std::cout << "roll: " << jsonData["roll"] << "°" << std::endl;
  
                std::cout << "---------------------------------" << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Failed to parse JSON: " << e.what() << std::endl;
            }
        }
    }

    return 0;
}