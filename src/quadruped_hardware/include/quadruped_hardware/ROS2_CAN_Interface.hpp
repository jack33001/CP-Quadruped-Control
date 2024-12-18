#ifndef ROS2_CAN_INTERFACE_HPP
#define ROS2_CAN_INTERFACE_HPP

#include "quadruped_hardware/CANInterface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace quadruped_hardware {
    class ROS2CANInterface : public rclcpp::Node {
        public:
            explicit ROS2CANInterface(const std::string& node_name, const char* socketName);
            virtual ~ROS2CANInterface() = default;

        private:
            void canMsgCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
            void readCANCallback();

            CAN_interface::CANInterface can_interface_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
            rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr can_subscription_;
            rclcpp::TimerBase::SharedPtr read_timer_;
    };
}

#endif // ROS2_CAN_INTERFACE_HPP
