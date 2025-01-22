// #ifndef QUADRUPED_HARDWARE__CAN_COMMAND_CONTROLLER_HPP_
// #define QUADRUPED_HARDWARE__CAN_COMMAND_CONTROLLER_HPP_

// #include <controller_interface/controller_interface.hpp>
// #include <hardware_interface/handle.hpp>
// #include <hardware_interface/types/hardware_interface_type_values.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/u_int8_multi_array.hpp>

// namespace quadruped_hardware
// {
// class CANCommandController : public controller_interface::ControllerInterface
// {
// public:
//     CANCommandController();

//     controller_interface::CallbackReturn on_init() override;

//     controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;


// private:
//     void command_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

//     std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
//     rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr command_subscriber_;
//     std::array<uint8_t, 8> command_;
// };
// }  // namespace quadruped_hardware

// #endif  // QUADRUPED_HARDWARE__CAN_COMMAND_CONTROLLER_HPP_