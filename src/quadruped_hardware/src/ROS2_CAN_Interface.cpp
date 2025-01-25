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

    // Retrieve the socket name from the hardware info parameters
    std::string socket_name = info.hardware_parameters.at("can_interface");
    can_interface_ = std::make_unique<CAN_interface::CANInterface>(socket_name.c_str());

    // Retrieve the can_ids string from the hardware info parameters
    std::string can_ids_str = info.hardware_parameters.at("can_ids");

    // Define the initial value as an array of 8 bytes of hexadecimal values
    std::array<uint8_t, 8> enable = {0xFF, 0xFF, 0xFF, 0xFF,
                                         0xFF, 0xFF, 0xFF, 0xFC}; // disable 8-byte hex values


    std::array<uint8_t, 8> disable =  {0xFF, 0xFF, 0xFF, 0xFF,
                                          0xFF, 0xFF, 0xFF, 0xFD};

    // Tokenize the can_ids string and populate the state and command interfaces
    std::istringstream iss(can_ids_str);
    std::string token;

    while (std::getline(iss, token, ',')) {
        try {
            int id = std::stoi(token); // Convert CAN ID from string to integer
            std::string name = std::to_string(id);


            // Create a map for the current message and initialize all bytes to 0
            std::map<size_t, uint8_t> byte_map;

            for (size_t i = 0; i < 8; ++i) {
                byte_map.emplace(i, enable[i]);
            }

            can_command_map_[name] = byte_map;
            can_state_map_[name] = byte_map;


            
            // state_interfaces_map_.emplace(name, hardware_interface::StateInterface("can_interface", name + "_msg", &can_state_map_[id]));
            // command_interfaces_map_.emplace(name, hardware_interface::CommandInterface("can_interface", name + "_cmd", &can_command_map_[id]));
            // RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "Added CAN ID: %d", id);

            // can_state_map_[id] = test_bytes;
            // can_command_map_[id] = test_bytes; // Set the initial 8-byte string value
    
        } catch (const std::invalid_argument &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ROS2CANInterface"), "Invalid CAN ID: %s", token.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        } catch (const std::out_of_range &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ROS2CANInterface"), "CAN ID out of range: %s", token.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ROS2CANInterface::on_configure(const rclcpp_lifecycle::State &) {
    // Configuration logic here (if needed)
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ROS2CANInterface::on_activate(const rclcpp_lifecycle::State &) {
    // Activation logic here (if needed)

    //     for (const auto &cmd : command_interfaces_map_) {
    //     int id = std::stoi(cmd.first);
    //     double value;
    //     if (cmd.second.get_value(value)) {
    //         unsigned char CANMsg[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
            
    //         // CANMsg[0] = static_cast<unsigned char>(id);
    //         // CANMsg[1] = static_cast<unsigned char>(value); // Example conversion
    //         // can_interface_->sendCANFrame(id, CANMsg);
    //         RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "SENDING %d ON %s ", id,CANMsg);
        
    //     } else {
    //         RCLCPP_WARN(rclcpp::get_logger("ROS2CANInterface"), "Failed to get value for command interface %s", cmd.first.c_str());
    //     }
    // }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ROS2CANInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    // Deactivation logic here (if needed)
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ROS2CANInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto &[message_name, byte_map] : can_state_map_) {
        
        for (size_t i = 0; i < 8; ++i) {
            double value = static_cast<double>(byte_map[i]); // Convert uint8_t to double
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                "can_interface", message_name + "_byte" + std::to_string(i), &value));
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ROS2CANInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto &[message_name, byte_map] : can_command_map_) {
        
        for (size_t i = 0; i < 8; ++i) {
            double value = static_cast<double>(byte_map[i]); // Convert uint8_t to double
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                "can_interface", message_name + "_byte" + std::to_string(i), &value));
        }
    }

    return command_interfaces;
}

hardware_interface::return_type ROS2CANInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
    RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "Starting read loop");

    unsigned char received_data[8];
    if (can_interface_->receiveCANFrame(received_data)) {

        RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "Exiting if statement");
        

        unsigned char id = received_data[0];

        // Convert the ID to an integer
        int number = static_cast<int>(id);
       

        // Convert the ID to a string (if needed)
        std::string name = std::to_string(id);

        

        
        // Print the ID for debugging
        RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "Received ID: %d", number);



        // auto it = state_interfaces_map_.find(name);
        // if (it != state_interfaces_map_.end()) {
        //     std::array<uint8_t, 8> value = static_cast<double>(received_data[1]); // Example conversion
        //     if (!it->second.set_value(value)) {
        //         RCLCPP_WARN(rclcpp::get_logger("ROS2CANInterface"), "Failed to set value for state interface %s", name.c_str());
        //     }
        // } else {
        //     RCLCPP_WARN(rclcpp::get_logger("ROS2CANInterface"), "State interface %s not found", name.c_str());
    }


    // for (auto &[message_name, byte_map] : can_command_map_) {
    //     unsigned char CANMsg[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
        
    //     for (size_t i = 0; i < 8; ++i) {
    //         double value = static_cast<double>(byte_map[i]); // Convert uint8_t to double

    //         CANMsg[i] = static_cast<unsigned char>(value); // Example conversion
    //     }

    //     // Convert message_name to an integer CAN ID
    //     int can_id = std::stoi(message_name);

    //     can_interface_->sendCANFrame(can_id, CANMsg);
    //     RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "ID: %d CANMSG %02x %02x %02x %02x %02x %02x %02x %02x", can_id, 
    //         CANMsg[0], CANMsg[1], CANMsg[2], CANMsg[3], CANMsg[4], CANMsg[5], CANMsg[6], CANMsg[7]);
    
    //     rate.sleep();
    // }



    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ROS2CANInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {

    rclcpp::Rate rate(1);

    for (auto &[message_name, byte_map] : can_command_map_) {
        unsigned char CANMsg[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
        
        for (size_t i = 0; i < 8; ++i) {
            double value = static_cast<double>(byte_map[i]); // Convert uint8_t to double

            CANMsg[i] = static_cast<unsigned char>(value); // Example conversion
        }

        // Convert message_name to an integer CAN ID
        int can_id = std::stoi(message_name);

        can_interface_->sendCANFrame(can_id, CANMsg);
        RCLCPP_INFO(rclcpp::get_logger("ROS2CANInterface"), "ID: %d CANMSG %02x %02x %02x %02x %02x %02x %02x %02x", can_id, 
            CANMsg[0], CANMsg[1], CANMsg[2], CANMsg[3], CANMsg[4], CANMsg[5], CANMsg[6], CANMsg[7]);
    
        
    }

    rate.sleep();

    // for (const auto &cmd : command_interfaces_map_) {
    //     int id = std::stoi(cmd.first);
    //     std::array<uint8_t, 8> value = {0};
    //     if (cmd.second.get_value(value)) {
    //         unsigned char CANMsg[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    //         // CANMsg[0] = static_cast<unsigned char>(id);
    //         // CANMsg[1] = static_cast<unsigned char>(value); // Example conversion
    //         // can_interface_->sendCANFrame(id, CANMsg);
    //     } else {
    //         RCLCPP_WARN(rclcpp::get_logger("ROS2CANInterface"), "Failed to get value for command interface %s", cmd.first.c_str());
    //     }
    // }


    return hardware_interface::return_type::OK;
}

} // namespace quadruped_hardware
