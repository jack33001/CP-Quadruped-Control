// #include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <memory>
#include "quadruped_hardware/MotorDriver.hpp"


#include "quadruped_hardware/ROS2_CAN_Motor.hpp"


namespace quadruped_hardware {

std::vector<int> parseCanId(const std::string& can_id_str) {
    std::vector<int> can_id;
    std::stringstream ss(can_id_str);
    std::string token;
    while (std::getline(ss, token, ',')) {
        can_id.push_back(std::stoi(token)); // Convert CAN ID from string to integer and add to vector
    }
    return can_id;
}



CANMotor::CANMotor() : cmd_position(0), cmd_velocity(0), cmd_effort(0), cmd_kp(0), cmd_kd(0),
                       state_position(0), state_velocity(0), state_effort(0) {}


hardware_interface::CallbackReturn CANMotor::on_init(const hardware_interface::HardwareInfo& info){
    // Initialization code
    hardware_interface::ComponentInfo config;

    joint_name = info.hardware_parameters.at("joint_name");
    const char* can_bus = info.hardware_parameters.at("can_bus").c_str();
    can_id = parseCanId(info.hardware_parameters.at("can_id"));

    motor_controller_ = std::make_unique<motor_driver::MotorDriver>(
        can_id, can_bus, motor_driver::MotorType::GIM8108
    );

    // command_data_ = {0,0,0};



    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CANMotor::on_configure(const rclcpp_lifecycle::State& previous_state){



    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CANMotor::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    // Cleanup code
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CANMotor::on_activate(const rclcpp_lifecycle::State& previous_state) {
    // Activation code
    auto start_state = motor_controller_->disableMotor(can_id);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CANMotor::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    // Deactivation code
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CANMotor::export_state_interfaces() {
    // Export state interfaces
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_name,  "position", &state_position));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_name,  "velocity", &state_velocity));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_name,  "effort", &state_effort));


    return std::move(state_interfaces);
}

std::vector<hardware_interface::CommandInterface> CANMotor::export_command_interfaces() {
    

    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_name,  "position", &cmd_position));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_name,  "velocity", &cmd_velocity));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_name,  "effort", &cmd_effort));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_name,  "kp", &cmd_kp));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_name,  "kd", &cmd_kd));

    // command_interfaces.emplace_back(joint_name, "position", &command_data_[0]);
    // command_interfaces.emplace_back(joint_name, "velocity", &command_data_[1]);
    // command_interfaces.emplace_back(joint_name, "effort", &command_data_[2]);

    return std::move(command_interfaces);
}

hardware_interface::return_type CANMotor::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // Read data from the hardware





    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CANMotor::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // Write data to the hardware

    motor_controller_->enableMotor(can_id); 
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    
    movecmd = {static_cast<float>(cmd_position),
     static_cast<float>(cmd_velocity),
      static_cast<float>(cmd_kp),
       static_cast<float>(cmd_kd),
        static_cast<float>(cmd_effort)};

    commandMap[can_id[0]] = movecmd;



    stateMap = motor_controller_->sendDegreeCommand( commandMap);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    state_position = stateMap[can_id[0]].position;
    state_velocity = stateMap[can_id[0]].velocity;
    state_effort = stateMap[can_id[0]].torque;



    motor_controller_->disableMotor(can_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    return hardware_interface::return_type::OK;
}


}  // namespace quadruped_hardware

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(quadruped_hardware::CANMotor, hardware_interface::ActuatorInterface)