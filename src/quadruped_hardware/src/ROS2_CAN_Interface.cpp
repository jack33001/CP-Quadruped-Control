#include <vector>

#include <unordered_map>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "quadruped_hardware/CANInterface.hpp"
#include "quadruped_hardware/ROS2_CAN_Interface.hpp"


namespace QuadrupedHardware {


class ROS2CANInterface final : public hardware_interface::SystemInterface {
    
ROS2CANInterface() {
    // Initialization code
    state_interfaces_.clear();
}


public:
    hardware_interface::HardwareInfo hardware_info_;

    std::unordered_map<int, double> can_state_map_;

    
    // std::vector<std::unique_ptr<hardware_interface::StateInterface>> state_interfaces_;

    // std::unique_ptr<CAN_interface::CANInterface> can_interface_;
    std::vector<hardware_interface::StateInterface> state_interfaces_;
    


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

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto & command_interface : command_interfaces_map_)
        {
            command_interfaces.emplace_back(std::move(command_interface.second));
        }
        return command_interfaces;
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


        // Retrieve the can_ids string from the hardware info parameters
        std::string can_ids_str = info.hardware_parameters.at("can_ids");

        // Tokenize the can_ids string and populate the state_interfaces_map_
        std::istringstream iss(can_ids_str);
        std::string token;
        while (iss >> token)
        {
            int id = std::stoi(token); // Convert CAN ID from string to integer
            std::string name = std::to_string(id);
            state_interfaces_map_.emplace(name, hardware_interface::StateInterface(name, "canbusmsg", &can_state_map_[id]));
            command_interfaces_map_.emplace(name, hardware_interface::CommandInterface(name, "canbuscmd", &can_command_map_[id]));
    
        }

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
        unsigned char received_data[8];
        if (can_interface_->receiveCANFrame(received_data))
        {
            std::string name = std::to_string(received_data[0]);
            auto it = state_interfaces_map_.find(name);
            if (it != state_interfaces_map_.end())
            {
                double value = static_cast<double>(received_data[1]); // Example conversion
                auto result = it->second.set_value(value);
                if (!result)
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to set value for state interface %s", name.c_str());
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "State interface %s not found", name.c_str());
            }
        }

        // std::string hex_message = "A1B2C3D4";

        // // Extract the first two bytes (4 characters, 2 bytes)
        // std::string first_two_bytes = hex_message.substr(0, 4);

        // // Convert the hexadecimal string to an integer
        // unsigned int numeral_value;
        // std::stringstream ss;
        // ss << std::hex << first_two_bytes;
        // ss >> numeral_value;

        // // read CAN Bus
        //         unsigned char received_data[8];
        // if (can_interface_->receiveCANFrame(received_data))
        // {
        //     for (size_t i = 0; i < state_interfaces_.size(); ++i)
        //     {
        //         if (state_interfaces_[i].get_name() == std::to_string(received_data[0]))
        //         {
        //             double value = static_cast<double>(received_data[1]); // Example conversion
                    
        //             //set state interface value with logging
        //             auto result = state_interfaces_[i].set_value(value);
        //             if (!result) //if failed 
        //             {
        //                 RCLCPP_WARN(this->get_logger(), "Failed to set value for state interface %s", state_interfaces_[i].get_name().c_str());
        //             }
        //         }
        //     }
        // }

        // // Read CAN messages and update 
        // unsigned char received_data[8];
        // if (can_interface_->receiveCANFrame(received_data))
        // {
        //     for (size_t i = 0; i < state_interfaces_.size(); ++i) {
        //         if (state_interfaces_[i].get_name() == std::to_string(received_data[0])) {
        //             double value = static_cast<double>(received_data[1]); // Example conversion
        //             state_interfaces_[i].set_value(value);
        //         }
        //     }
        // }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override
    {
       //pass

       for (const auto & cmd : command_interfaces_map_)
        {
        int id = std::stoi(cmd.first);
        double value;
        if (cmd.second.get_value(value))
        {
            unsigned char CANMsg[8] = {0};
            CANMsg[0] = static_cast<unsigned char>(id);
            CANMsg[1] = static_cast<unsigned char>(value); // Example conversion
            can_interface_->sendCANFrame(id, CANMsg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to get value for command interface %s", cmd.first.c_str());
        }
        }
        
        return hardware_interface::return_type::OK; // CORRECT

    }

private:
    std::unique_ptr<CAN_interface::CANInterface> can_interface_;
    std::unordered_map<std::string, hardware_interface::StateInterface> state_interfaces_map_;
    std::unordered_map<std::string, hardware_interface::CommandInterface> command_interfaces_map_;
    std::unordered_map<int, double> can_command_map_;
    
};

} // namespace quadruped_hardware