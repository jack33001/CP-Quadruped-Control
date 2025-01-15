#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"  // For CAN messages
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "quadruped_hardware/CANInterface.hpp"


// std::vector<hardware_interface::StateInterface> QuadrupedHardware::export_state_interfaces()
// {
//     std::vector<hardware_interface::StateInterface> state_interfaces;
//     for (const auto & can_id : hardware_info_.hardware_parameters["can_ids"])
//     {
//         int id = std::stoi(can_id); // Convert CAN ID from string to integer
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             std::to_string(id), "canbusmsg", &can_state_map_[id]));
//     }
//     return state_interfaces;
// }

// std::vector<hardware_interface::CommandInterface> QuadrupedHardware::export_command_interfaces()
// {
//     std::vector<hardware_interface::CommandInterface> command_interfaces;
//     for (const auto & can_id : hardware_info_.hardware_parameters["can_ids"])
//     {
//         int id = std::stoi(can_id); // Convert CAN ID from string to integer
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             std::to_string(id), "canbusmsg", &can_state_map_[id]));
//     }
//     return command_interfaces;
// }


namespace quadruped_hardware {

class ROS2CANInterface : public hardware_interface::SystemInterface
{
public:
    ROS2CANInterface() = default;





    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve the socket name from the hardware info parameters
        std::string socket_name = info.hardware_parameters.at("can_interface");
        can_interface_ = std::make_unique<CAN_interface::CANInterface>(socket_name.c_str());



        
        // Initialize state interface for each CAN ID
        for (const auto & can_id : info.hardware_parameters["can_ids"])
        {
            int id = std::stoi(can_id); // Convert CAN ID from string to integer
            can_interfaces_.emplace_back(hardware_interface::StateInterface(
                std::to_string(id), "canbusmsg", &can_state_map_[id]));
        }

        // Initialize command interface for bus
        command_interfaces_.emplace_back(hardware_interface::CommandInterface(
                std::to_string(id), "canbusmsg", &can_state_map_[id]));

            std::vector<double> CAN_commands_;
            std::vector<double> CAN_received_;



        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
    {
        // Configuration code here

        // do nothing
        



        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
    {
        // Activation code here
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
    {
        // Deactivation code here
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override
    {
        // Read CAN messages and update joint states
        unsigned char received_data[8];
        if (can_interface_->receiveCANFrame(received_data))
        {
            
            // Assuming received_data contains the CAN bus messages in a specific format
            for (size_t i = 0; i < state_interfaces_.size(); ++i)
            {
                *state_interfaces_[i].get_value_ptr() = static_cast<double>(received_data[i]);
            }
            // Process received_data and update joint_state_
            // Example: joint_state_.position[0] = some_value_from_received_data;
        }


        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override
    {
        // Write commands to CAN bus
        unsigned char cmd[8];
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            cmd[i] = static_cast<unsigned char>(*command_interfaces_[i].get_value_ptr());
        }
        can_interface_->sendCANFrame(10, cmd);
        return hardware_interface::return_type::OK;
    }

private:
    std::unique_ptr<CAN_interface::CANInterface> can_interface_;
    sensor_msgs::msg::JointState joint_state_;
};

}  // namespace quadruped_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_hardware::ROS2CANInterface, hardware_interface::SystemInterface)