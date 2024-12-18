#include "quadruped_hardware/ROS2_CAN_Interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"  // For CAN messages

namespace quadruped_hardware {

ROS2CANInterface::ROS2CANInterface(const std::string& node_name, const char* socketName)
    : Node(node_name), can_interface_(socketName) {
    
    // Create publisher for joint states
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);

    // Create subscription for CAN messages
    can_subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "can_messages", 10,
        std::bind(&ROS2CANInterface::canMsgCallback, this, std::placeholders::_1));

    // Create timer for reading CAN messages
    read_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ROS2CANInterface::readCANCallback, this));
}

void ROS2CANInterface::canMsgCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    if (msg->data.size() != 8) {
        RCLCPP_ERROR(this->get_logger(), "Invalid CAN message size");
        return;
    }

    // Send CAN message
    unsigned char can_data[8];
    std::copy(msg->data.begin(), msg->data.end(), can_data);
    can_interface_.sendCANFrame(0x123, can_data); // Using default CAN ID
}

void ROS2CANInterface::readCANCallback() {
    unsigned char received_data[8];
    if (can_interface_.receiveCANFrame(received_data)) {
        // Create and publish joint state message
        auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
        joint_state_msg->header.stamp = this->now();
        // Process received_data and fill joint_state_msg
        joint_state_publisher_->publish(std::move(joint_state_msg));
    }
}

} // namespace quadruped_hardware

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Create node with default CAN interface
    auto node = std::make_shared<quadruped_hardware::ROS2CANInterface>(
        "ros2_can_interface", "can0");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

