#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "quadruped_hardware/ROS2_CAN_Interface.hpp"

class StateBroadcaster : public rclcpp::Node
{
public:
    StateBroadcaster()
        : Node("state_broadcaster")
    {
        state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("state_interfaces", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&StateBroadcaster::publish_states, this));
    }

private:
    void publish_states()
    {
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->now();

        // Access the state interfaces from the hardware interface
        for (const auto &state_interface : state_interfaces_map_)
        {
            double value;
            if (state_interface.second.get_value(value))
            {
                message.name.push_back(state_interface.first);
                message.position.push_back(value);
            }
        }

        state_publisher_->publish(message);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<std::string, hardware_interface::StateInterface> state_interfaces_map_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}