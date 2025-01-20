#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <string>

class ZeroJointController : public rclcpp::Node {
public:
    ZeroJointController(const std::string &joint_name) : Node("zero_joint_controller"), joint_name_(joint_name) {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>(joint_name_ + "_command", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ZeroJointController::publish_zero_position, this)
        );
    }

private:
    void publish_zero_position() {
        auto message = std_msgs::msg::Float64();
        message.data = 0.0;
        publisher_->publish(message);
    }

    std::string joint_name_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ZeroJointController <joint_name>");
        return 1;
    }

    std::string joint_name = argv[1];
    rclcpp::spin(std::make_shared<ZeroJointController>(joint_name));
    rclcpp::shutdown();
    return 0;
}
