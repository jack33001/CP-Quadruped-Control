#include "quadruped_hardware/ROS2_CAN_Interface.hpp"

#include <rclcpp/logging.hpp>
#include <sstream>

namespace quadruped_hardware {

ROS2CANInterface::ROS2CANInterface() {
    state_interfaces_.clear();
}

hardware_interface::CallbackReturn ROS2CANInterface::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hardware_info_ = info;
    RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "Hardware Info: %s", info.name.c_str());

    // Log all hardware parameters
    for (const auto &param : info.hardware_parameters) {
        RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "Parameter: %s = %s", param.first.c_str(), param.second.c_str());
    }

    // Retrieve the socket name from the hardware info parameters
    std::string socket_name = info.hardware_parameters.at("can_interface");
    can_interface_ = std::make_unique<CAN_interface::CANInterface>(socket_name.c_str());

    // Retrieve the can_ids string from the hardware info parameters
    std::string can_ids_str = info.hardware_parameters.at("can_ids");


    
    std::cout << "IDs: " << can_ids_str << std::endl;


    // Tokenize the can_ids string and populate the state and command interfaces
    std::istringstream iss(can_ids_str);
    std::string token;

    RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "ABOUT TO START MAKING INTERFACES");

    while (iss >> token) {
        
        int id = std::stoi(token); // Convert CAN ID from string to integer

        std::cout << "ID: " << id << std::endl;

        std::string name = std::to_string(id);
        state_interfaces_map_.emplace(name, hardware_interface::StateInterface(name, "canbusmsg", &can_state_map_[id]));
        command_interfaces_map_.emplace(name, hardware_interface::CommandInterface(name, "canbuscmd", &can_command_map_[id]));
        RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "Added CAN ID: %d", id);
            
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
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ROS2CANInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    // Deactivation logic here (if needed)
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ROS2CANInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (const auto &can_id : hardware_info_.hardware_parameters.at("can_ids")) {
        int id = std::stoi(std::string(1, can_id)); // Convert CAN ID from string to integer
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(std::to_string(id), "canbusmsg", &can_state_map_[id]));
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
            unsigned char CANMsg[8] = {0};
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
