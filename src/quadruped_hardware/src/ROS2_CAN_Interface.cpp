#include "quadruped_hardware/ROS2_CAN_Interface.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace quadruped_hardware {

ROS2CANInterface::ROS2CANInterface() {
    state_interfaces_.clear();
}

hardware_interface::CallbackReturn ROS2CANInterface::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hardware_info_ = info_;
    RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "Hardware Info: %s", info.name.c_str());

    // Log all hardware parameters
    // for (const auto &param : info.hardware_parameters) {
    //     RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "Parameter: %s = %s", param.first.c_str(), param.second.c_str());
    // }

    // Retrieve the socket name from the hardware info parameters
    std::string socket_name = info.hardware_parameters.at("can_interface");
    can_interface_ = std::make_unique<CAN_interface::CANInterface>(socket_name.c_str());

    // Retrieve the can_ids string from the hardware info parameters
    std::string can_ids_str = info.hardware_parameters.at("can_ids");


    // Tokenize the can_ids string and populate the state and command interfaces
    std::istringstream iss(can_ids_str);
    std::string token;


    // while (iss >> token) {
        
    //     int id = std::stoi(token); // Convert CAN ID from string to integer

    //     std::cout << "ID: " << id << std::endl;

    //     std::string name = std::to_string(id);
    //     state_interfaces_map_.emplace(name, hardware_interface::StateInterface(name, "canbusmsg", &can_state_map_[id]));
    //     command_interfaces_map_.emplace(name, hardware_interface::CommandInterface(name, "canbuscmd", &can_command_map_[id]));
    //     RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "Added CAN ID: %d", id);
            
    // }
    std::array<uint8_t, 8> test_bytes = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}; // Example 8-byte string


    while (std::getline(iss, token, ',')) {
        try {
            int id = std::stoi(token); // Convert CAN ID from string to integer
            std::string name = std::to_string(id);
            state_interfaces_map_.emplace(name, hardware_interface::StateInterface("can_interface", name + "_msg", &can_state_map_[id]));
            // command_interfaces_map_.emplace(name, hardware_interface::CommandInterface("can_interface", name + "_cmd", &can_command_map_[id]));
            command_interfaces_map_.emplace(name, CustomCommandInterface("can_interface", name + "_cmd", &can_command_map_[id]));
            RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "Added CAN ID: %d", id);

            can_command_map_[id] = test_bytes; // Set the initial 8-byte string value
    
        } catch (const std::invalid_argument &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ROS2CANInterface"), "Invalid CAN ID: %s", token.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        } catch (const std::out_of_range &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ROS2CANInterface"), "CAN ID out of range: %s", token.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "DONE MAKING INTERFACES");

    // try {
    //     std::string socket_name = info.hardware_parameters.at("can_interface");
    //     can_interface_ = std::make_unique<CAN_interface::CANInterface>(socket_name.c_str());

    //     std::cout << "**************Socket name: " << socket_name << std::endl;
    //     // Retrieve the can_ids parameter and process it
    //     std::string can_ids_str = info.hardware_parameters.at("can_ids");
    //     std::istringstream iss(can_ids_str);
    //     std::string token;
    //     while (iss >> token) {
    //         int id = std::stoi(token); // Convert CAN ID from string to integer
    //         std::string name = std::to_string(id);
    //         state_interfaces_map_.emplace(name, hardware_interface::StateInterface(name, "canbusmsg", &can_state_map_[id]));
    //         command_interfaces_map_.emplace(name, hardware_interface::CommandInterface(name, "canbuscmd", &can_command_map_[id]));
    //     }
    // } catch (const std::out_of_range &e) {
    //     RCLCPP_ERROR(rclcpp::get_logger("ROS2CANInterface"), "Missing required hardware parameters: %s", e.what());
    //     return hardware_interface::CallbackReturn::ERROR;
    // } catch (const std::exception &e) {
    //     RCLCPP_ERROR(rclcpp::get_logger("ROS2CANInterface"), "Error initializing hardware: %s", e.what());
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ROS2CANInterface::on_configure(const rclcpp_lifecycle::State &) {
    // Configuration logic here (if needed)
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ROS2CANInterface::on_activate(const rclcpp_lifecycle::State &) {
    // Activation logic here (if needed)

        for (const auto &cmd : command_interfaces_map_) {
        int id = std::stoi(cmd.first);
        double value;
        if (cmd.second.get_value(value)) {
            unsigned char CANMsg[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
            CANMsg[0] = static_cast<unsigned char>(id);
            CANMsg[1] = static_cast<unsigned char>(value); // Example conversion
            can_interface_->sendCANFrame(id, CANMsg);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ROS2CANInterface"), "Failed to get value for command interface %s", cmd.first.c_str());
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ROS2CANInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    // Deactivation logic here (if needed)
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ROS2CANInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    std::string can_ids_str = hardware_info_.hardware_parameters.at("can_ids");

    // Tokenize the can_ids string and populate the state interfaces
    std::istringstream iss(can_ids_str);
    std::string token;

    while (std::getline(iss, token, ',')) {
        try {
            int id = std::stoi(token); // Convert CAN ID from string to integer
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(std::to_string(id), "canbusmsg", &can_state_map_[id]));
            RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "Added State Interface for CAN ID: %d", id);
        } catch (const std::invalid_argument &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ROS2CANInterface"), "Invalid CAN ID: %s", token.c_str());
        } catch (const std::out_of_range &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ROS2CANInterface"), "CAN ID out of range: %s", token.c_str());
        }
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ROS2CANInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto &command_interface : command_interfaces_map_) {
        command_interfaces.emplace_back(std::move(command_interface.second));
    }
    return command_interfaces;
}

hardware_interface::return_type ROS2CANInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
    unsigned char received_data[8];
    if (can_interface_->receiveCANFrame(received_data)) {
        std::string name = std::to_string(received_data[0]);
        auto it = state_interfaces_map_.find(name);
        if (it != state_interfaces_map_.end()) {
            double value = static_cast<double>(received_data[1]); // Example conversion
            if (!it->second.set_value(value)) {
                RCLCPP_WARN(rclcpp::get_logger("ROS2CANInterface"), "Failed to set value for state interface %s", name.c_str());
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ROS2CANInterface"), "State interface %s not found", name.c_str());
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ROS2CANInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {


    for (const auto &cmd : command_interfaces_map_) {
        int id = std::stoi(cmd.first);
        double value;
        if (cmd.second.get_value(value)) {
            unsigned char CANMsg[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
            CANMsg[0] = static_cast<unsigned char>(id);
            CANMsg[1] = static_cast<unsigned char>(value); // Example conversion
            can_interface_->sendCANFrame(id, CANMsg);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ROS2CANInterface"), "Failed to get value for command interface %s", cmd.first.c_str());
        }
    }




    return hardware_interface::return_type::OK;
}

} // namespace quadruped_hardware
