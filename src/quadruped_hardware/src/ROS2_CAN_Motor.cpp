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



CANMotor::CANMotor() : cmd_position(0), cmd_velocity(0), cmd_effort(0), cmd_kp(2), cmd_kd(1), cmd_m_state(0),
                       state_position(0), state_velocity(0), state_effort(0), state_kp(2), state_kd(1), state_m_state(0) {}


hardware_interface::CallbackReturn CANMotor::on_init(const hardware_interface::HardwareInfo& info){
    // Initialization code
    hardware_interface::ComponentInfo config;

    joint_name = info.hardware_parameters.at("joint_name");
    const char* can_bus = info.hardware_parameters.at("can_bus").c_str();
    can_id = parseCanId(info.hardware_parameters.at("can_id"));

    motor_controller_ = std::make_unique<motor_driver::MotorDriver>(
        can_id, can_bus, motor_driver::MotorType::GIM8108
    );

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CANMotor::on_configure(const rclcpp_lifecycle::State& previous_state){
    // Configuration code
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CANMotor::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    // Cleanup code
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CANMotor::on_activate(const rclcpp_lifecycle::State& previous_state) {
    // Activation code

    motor_controller_->disableMotor(can_id);
    auto start_state = motor_controller_->enableMotor(can_id);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CANMotor::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    // Deactivation code
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CANMotor::export_state_interfaces() {
    // Export state interfaces
    state_interfaces_.emplace_back(
        hardware_interface::StateInterface(joint_name,  "position", &state_position));
    state_interfaces_.emplace_back(
        hardware_interface::StateInterface(joint_name,  "velocity", &state_velocity));
    state_interfaces_.emplace_back(
        hardware_interface::StateInterface(joint_name,  "effort", &state_effort));
    state_interfaces_.emplace_back(
        hardware_interface::StateInterface(joint_name,  "kp", &state_kp));
    state_interfaces_.emplace_back(
        hardware_interface::StateInterface(joint_name,  "kd", &state_kd));
    state_interfaces_.emplace_back(
        hardware_interface::StateInterface(joint_name,  "m_state", &state_m_state));


    return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> CANMotor::export_command_interfaces() {
    // Export command interfaces
    command_interfaces_.emplace_back(
        hardware_interface::CommandInterface(joint_name,  "position", &cmd_position));
    command_interfaces_.emplace_back(
        hardware_interface::CommandInterface(joint_name,  "velocity", &cmd_velocity));
    command_interfaces_.emplace_back(
        hardware_interface::CommandInterface(joint_name,  "effort", &cmd_effort));
    command_interfaces_.emplace_back(
        hardware_interface::CommandInterface(joint_name,  "kp", &cmd_kp));
    command_interfaces_.emplace_back(
        hardware_interface::CommandInterface(joint_name,  "kd", &cmd_kd));
    command_interfaces_.emplace_back(
        hardware_interface::CommandInterface(joint_name,  "m_state", &cmd_m_state));

    return std::move(command_interfaces_);
}

hardware_interface::return_type CANMotor::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // Read data from the hardware
    // std::lock_guard<std::mutex> lock(read_write_mutex_);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CANMotor::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // Write data to the hardware
    std::cout << "_START WRITE MOTOR_" << can_id[0] << std::endl;
    // std::lock_guard<std::mutex> lock(read_write_mutex_);

    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // if effort is over safe limits, drop kp kd to zero
    if (state_effort > 1)
    {
        RCLCPP_INFO(rclcpp::get_logger("CANMotor"), "limit exceeded");

        cmd_kp  = 0;
        cmd_kd = 0;

        movecmd = {static_cast<float>(cmd_position),
     static_cast<float>(cmd_velocity),
      static_cast<float>(cmd_kp),
       static_cast<float>(cmd_kd),
        static_cast<float>(cmd_effort)};

        commandMap[can_id[0]] = movecmd;
    }
    else
    {
    
    movecmd = {static_cast<float>(cmd_position),
     static_cast<float>(cmd_velocity),
      static_cast<float>(cmd_kp),
       static_cast<float>(cmd_kd),
        static_cast<float>(cmd_effort)};

    commandMap[can_id[0]] = movecmd;
    }


    if(cmd_m_state == 0)
    {
        stateMap = motor_controller_->sendDegreeCommand( commandMap);

    }
    else if(cmd_m_state == 1)
    {
        stateMap = motor_controller_->enableMotor(can_id);
        cmd_m_state = 0;
        std::cout << "Motor: " << can_id[0] << "Enabled"<< std::endl;
    }
    else if(cmd_m_state == 2)
    {
        stateMap = motor_controller_->disableMotor(can_id);
        // cmd_m_state = 0;
        std::cout << "Motor: " << can_id[0] << "Disabled"<< std::endl;
    }
    else if(cmd_m_state == 3)
    {
        // check if zeroing would cause the motor to jump
        if ( abs(cmd_position) < 0.2 || (state_kp==0 and state_kd==0))
        {
            movecmd = {static_cast<float>(0),
                static_cast<float>(0),
                static_cast<float>(0),
                static_cast<float>(0),
                    static_cast<float>(0)};

            commandMap[can_id[0]] = movecmd;
            
            stateMap = motor_controller_->sendDegreeCommand( commandMap);
            
            stateMap = motor_controller_->setZeroPosition(can_id);
            cmd_m_state = 0;
    
            std::cout << "Motor: " << can_id[0] << " Zeroed"<< std::endl;
        }

    }


    // stateMap = motor_controller_->sendDegreeCommand( commandMap);




    //update state variables AFTER sending command
    state_position = stateMap[can_id[0]].position;
    state_velocity = stateMap[can_id[0]].velocity;
    state_effort = stateMap[can_id[0]].torque;
    state_kp = cmd_kp;
    state_kd = cmd_kd;
    state_m_state = cmd_m_state;





    // LOGGING
    std::cout << "_COMMANDS MOTOR_" << can_id[0] << std::endl;
    // std::cout << "Commanded position: " << cmd_position << std::endl;
    // std::cout << "Commanded velocity: " << cmd_velocity << std::endl;
    // std::cout << "Commanded kp: " << cmd_kp << std::endl;
    // std::cout << "Commanded kd: " << cmd_kd << std::endl;
    // std::cout << "Commanded effort: " << cmd_effort << std::endl;
    // std::cout << "Commanded state: " << cmd_m_state << std::endl;

    // std::cout << "_STATES MOTOR_" << can_id[0] << std::endl;
    // std::cout << "State position: " << state_position << std::endl;
    // std::cout << "State velocity: " << state_velocity << std::endl;
    // std::cout << "State torque: " << state_effort << std::endl;
    // std::cout << "State kp: " << state_kp << std::endl;
    // std::cout << "State kd: " << state_kd << std::endl;
    // std::cout << "State m_state: " << state_m_state << std::endl;



    // motor_controller_->disableMotor(can_id);

    // sleep(1);
    return hardware_interface::return_type::OK;
}


}  // namespace quadruped_hardware

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(quadruped_hardware::CANMotor, hardware_interface::ActuatorInterface)