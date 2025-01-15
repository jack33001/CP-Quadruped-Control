#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "quadruped_hardware/CANInterface.hpp"
#include "quadruped_hardware/ROS2_CAN_Interface.hpp"


namespace QuadrupedHardware {


class ROS2CANInterface final : public hardware_interface::SystemInterface {
    
ROS2CANInterface() {
    // Initialization code
    state_interfaces_ = {};
}


public:
    hardware_interface::HardwareInfo hardware_info_;

    std::unordered_map<int, double> can_state_map_;

    std::vector<std::unique_ptr<hardware_interface::StateInterface>> state_interfaces_;

    // std::unique_ptr<CAN_interface::CANInterface> can_interface_;

    


    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (const auto & can_id : hardware_info_.hardware_parameters["can_ids"])
        {
            int id = std::stoi(std::string(1, can_id)); // Convert CAN ID from string to integer
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(std::to_string(id), "canbusmsg", &can_state_map_[id]));
        }
        return state_interfaces;
    }


    
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve the socket name from the hardware info parameters
        std::string socket_name = info.hardware_parameters.at("can_interface");
        can_interface_ = std::make_unique<CAN_interface::CANInterface>(socket_name.c_str());

        
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
    
        // Read CAN messages and update 
        unsigned char received_data[8];
        if (can_interface_->receiveCANFrame(received_data))
        {
            for (size_t i = 0; i < state_interfaces_.size(); ++i) {
                if (state_interfaces_[i].get_name() == std::to_string(received_data[0])) {
                    double value = static_cast<double>(received_data[1]); // Example conversion
                    state_interfaces_[i].set_value(value);
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override
    {
       
    }

private:
    std::unique_ptr<CAN_interface::CANInterface> can_interface_;
    
};

} // namespace quadruped_hardware