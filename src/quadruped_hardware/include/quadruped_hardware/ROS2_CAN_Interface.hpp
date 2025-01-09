#ifndef ROS2_CAN_INTERFACE_HPP
#define ROS2_CAN_INTERFACE_HPP

#include "quadruped_hardware/CANInterface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace quadruped_hardware {
    class ROS2CANInterface : public hardware_interface::SystemInterface {
        public:
            ROS2CANInterface();
            virtual ~ROS2CANInterface() = default;

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::unique_ptr<CAN_interface::CANInterface> can_interface_;
            sensor_msgs::msg::JointState joint_state_;
    };
}

#endif // ROS2_CAN_INTERFACE_HPP