#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include <memory>

#include <iostream>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp> // Include the JSON library (https://github.com/nlohmann/json)

using json = nlohmann::json;




using namespace std::chrono_literals;


class MultiTopicPublisher : public rclcpp::Node
{
public:
  MultiTopicPublisher(size_t num_topics)
  : Node("multi_topic_publisher")
  {

    usbPort = std::make_unique<std::ifstream>("/dev/ttyACM0");

    // std::ifstream usbPort("/dev/ttyACM0"); // Replace with "COMx" for Windows
    // std::ifstream usbPort("COM5");
    if (!usbPort->is_open()) {
        std::cerr << "Failed to open USB port!" << std::endl;
       
    }

    std::string jsonString;





    std::string topic_name = "i";
    publishers_.emplace_back(
    this->create_publisher<std_msgs::msg::String>(topic_name, 10));
    RCLCPP_INFO(this->get_logger(), "Created publisher for %s", topic_name.c_str());

    topic_name = "j";
    publishers_.emplace_back(
    this->create_publisher<std_msgs::msg::String>(topic_name, 10));
    RCLCPP_INFO(this->get_logger(), "Created publisher for %s", topic_name.c_str());





    // Create a timer to publish messages periodically
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),  // Adjust the interval as needed
      std::bind(&MultiTopicPublisher::publish_messages, this)




    );
  }

private:
  std::unique_ptr<std::ifstream> usbPort;

  void publish_messages()
  {

    std::string jsonString;
    json jsonData;
   
    // Read a line of JSON from the USB port
    if (std::getline(*usbPort, jsonString)) {
        try {
            // Parse the JSON string
            jsonData = json::parse(jsonString);
            

            RCLCPP_INFO(this->get_logger(), "JSON received: %s ", jsonString.c_str());

            auto message = std_msgs::msg::String();


            message.data = std::to_string(jsonData["pitch"].get<double>());
            publishers_[0]->publish(message);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data);

            
            message.data = std::to_string(jsonData["roll"].get<double>());
            publishers_[1]->publish(message);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data);
            


        } catch (const std::exception& e) {
            std::cerr << "Failed to parse JSON: " << e.what() << std::endl;
        }
    }
    

    
  }

  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Specify the number of topics to publish
  size_t num_topics = 5;  // Change this to your desired number of topics
  if (argc > 1) {
    num_topics = std::stoul(argv[1]);  // Optionally pass as a command-line argument
  }

  auto node = std::make_shared<MultiTopicPublisher>(num_topics);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



