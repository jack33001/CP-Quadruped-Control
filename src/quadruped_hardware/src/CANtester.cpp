// #include "quadruped_hardware/CANtester.hpp"

// namespace quadruped_hardware
// {
// CANCommandController::CANCommandController()
// : command_({0})
// {
// }

// controller_interface::return_type CANCommandController::init(const std::string & controller_name)
// {
//     auto ret = ControllerInterface::init(controller_name);
//     if (ret != controller_interface::return_type::OK) {
//         return ret;
//     }

//     auto node = get_node();
//     command_subscriber_ = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
//         "~/command", 10, std::bind(&CANCommandController::command_callback, this, std::placeholders::_1));

//     return controller_interface::return_type::OK;
// }

// controller_interface::return_type CANCommandController::update()
// {
//     // set all available cmd interfaces to the current command_ value
//     for (auto & command_interface : command_interfaces_) {
//         for (size_t i = 0; i < command_.size(); ++i) {
//             command_interface.set_value(static_cast<double>(command_[i]));
//         }
//     }
//     return controller_interface::return_type::OK;
// }

// void CANCommandController::command_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
// {
//     if (msg->data.size() == 8) {
//         std::copy(msg->data.begin(), msg->data.end(), command_.begin());
//     } else {
//         RCLCPP_WARN(get_node()->get_logger(), "Received command with incorrect size");
//     }
// }

// }  // namespace quadruped_hardware

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(quadruped_hardware::CANCommandController, controller_interface::ControllerInterface)