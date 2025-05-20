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




CANMotor::CANMotor() : cmd_position(0), cmd_velocity(0), cmd_effort(0), cmd_kp(0), cmd_kd(1), cmd_m_state(0), cmd_flip(1),
                       state_position(0), state_velocity(0), state_effort(0), state_kp(0), state_kd(1), state_m_state(0),state_flip(1),
                        effort_limit(1.5),frequency(100), min_pos(-10000), max_pos(10000),state_watchdog(0) {}


std::map<int, motor_driver::motorState> CANMotor::send_motor_cmd(){
    // move command for motor with config for deg, rad and position limits
    std::map<int, motor_driver::motorState> stateMap;

    // Set command position limits
    if (cmd_position > max_pos)
    {
        cmd_position = max_pos;
    }
    if (cmd_position < min_pos)
    {
        cmd_position = min_pos;
    }
    
    // Radian vs degree control
    if(command_type=="degree")
    {
        stateMap = motor_controller_->sendDegreeCommand( commandMap);
    }
    else if (command_type == "radian")
    {
        stateMap = motor_controller_->sendRadCommand( commandMap);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("CANMotor"), "Obscure error: Command type not set to degree or radian in URDF and safety failed");
    }

    return stateMap;
}

void CANMotor::startThread()
{
    std::cout << "____THREAD START____" << std::endl;
    running_ = true;
    thread_ = std::thread(&CANMotor::threadLoop, this);
}

void CANMotor::stopThread()
{
    running_ = false;
    if (thread_.joinable())
    {
        thread_.join();
    }

    std::cout << "____THREAD STOPPED____" << std::endl;
}

void CANMotor::threadLoop()
{
  using namespace std::chrono_literals;
  
  while (running_)
  {
    // reset watchdog timer
    watchdog->reset();


    // Estop conditions
    // if out of position limits
    if(state_position > max_pos || state_position < min_pos)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CANMotor"), "Motor %d Position Limit Exceeded: pos %f range: %f - %f", can_id[0],state_position, min_pos, max_pos);
        // set E stop state
        cmd_m_state = 4;
    }
    // if cmd effort is too high
    if(abs(cmd_effort) > 18)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CANMotor"), "Command Effort Exceeded Motor Max of 18: %f", cmd_effort);
        // set E stop state
        cmd_m_state = 4;
    }
    // if effort is over safe limits, drop kp kd to zero
    if (state_effort > effort_limit)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CANMotor"), "State Effort Exceeded Set Motor limit of %f: %f", effort_limit, state_effort);
        // set E stop state
        cmd_m_state = 4;
    }

    // Update command data from the hardware
    movecmd = {static_cast<float>(cmd_position*cmd_flip + (cmd_flip*position_offset)),
        static_cast<float>(cmd_velocity*cmd_flip),
        static_cast<float>(cmd_kp),
        static_cast<float>(cmd_kd),
        static_cast<float>(cmd_effort*cmd_flip)};

    commandMap[can_id[0]] = movecmd;


    // State Machine
    if(cmd_m_state == 0)
    // send standard cmd state
    {
        stateMap = send_motor_cmd();
    }
    else if(cmd_m_state == 1)
    // enable motor state
    {
        stateMap = motor_controller_->enableMotor(can_id);
        cmd_m_state = 0;
        std::cout << "Motor: " << can_id[0] << "Enabled"<< std::endl;
        
    }
    else if(cmd_m_state == 2)
    // disable motor state
    {
        stateMap = motor_controller_->disableMotor(can_id);
        // cmd_m_state = 0;
        std::cout << "Motor: " << can_id[0] << "Disabled"<< std::endl;
    }
    else if(cmd_m_state == 3)
    // zero motor state
    {
        // check if zeroing would cause the motor to jump
        if ( abs(cmd_position) < 0.2 || (state_kp==0 and state_kd==0))
        {
            // stop motor exert 0 effort
            movecmd = {static_cast<float>(0),
                        static_cast<float>(0),
                        static_cast<float>(0),
                        static_cast<float>(0), 
                        static_cast<float>(0)};
            commandMap[can_id[0]] = movecmd;
            
            stateMap = send_motor_cmd();
            
            stateMap = motor_controller_->setZeroPosition(can_id);
            cmd_m_state = 0;
    
            // Set Position limits
            min_pos = std::stod(info_.hardware_parameters.at("min_position_limit"));
            max_pos = std::stod(info_.hardware_parameters.at("max_position_limit"));

            std::cout << "Motor: " << can_id[0] << " Zeroed"<< std::endl;
        }
    }
    else if(cmd_m_state == 4)
        // E_stop
    { 
        movecmd = {static_cast<float>(0),
                    static_cast<float>(0),
                    static_cast<float>(0),
                    static_cast<float>(0.5), //kd
                    static_cast<float>(0)};
        commandMap[can_id[0]] = movecmd;
        
        stateMap = send_motor_cmd();

        std::this_thread::sleep_for(100ms);  // extra timing as needed
    }

    else
    {
        std::cout << "Motor: "<< joint_name << "ID: " << can_id[0] << " has entered an unknown state "<< std::endl;
    }

    //update state variables AFTER sending command
    state_position = stateMap[can_id[0]].position * cmd_flip - (cmd_flip*position_offset);
    state_velocity = stateMap[can_id[0]].velocity * cmd_flip;
    state_effort = stateMap[can_id[0]].torque * cmd_flip;
    state_kp = cmd_kp;
    state_kd = cmd_kd;
    state_m_state = cmd_m_state;
    state_flip = cmd_flip;
    state_watchdog = 0; //watchdog set to okay every loop, if hung up watchdog will be set to 1

    // Sleep to maintain your desired thread update rate
    std::this_thread::sleep_for(thread_period);  // Adjust timing as needed
  }
}


void CANMotor::on_timeout(){
    // timeout function for watchdog timer
    state_watchdog=1;
    RCLCPP_ERROR(rclcpp::get_logger("CANMotor"), "Watchdog triggered on motor: %d", can_id[0]);
    std::this_thread::sleep_for(10ms);
}

hardware_interface::CallbackReturn CANMotor::on_init(const hardware_interface::HardwareInfo& info){
    // Initialization code
    hardware_interface::ComponentInfo config;

    // Joint name for interface names
    joint_name = info.hardware_parameters.at("joint_name");

    // Motor Controller params
    const char* can_bus = info.hardware_parameters.at("can_bus").c_str();
    can_id = parseCanId(info.hardware_parameters.at("can_id"));

    // Thread period
    frequency = std::stod(info.hardware_parameters.at("frequency"));
    period = std::round(1/frequency*1000)/1000; //round to ms
    thread_period = std::chrono::duration<double>(period);

    // Motor limit parameters
    effort_limit = std::stod(info.hardware_parameters.at("effort_limit"));

    // set Degree or radian controll
    command_type = info.hardware_parameters.at("command_type").c_str();
    if (command_type == "degree" || command_type == "deg"){
        command_type = "degree";
    }
    else if (command_type == "radian" || command_type == "rad"){
        command_type = "radian";
    }
    else{
        RCLCPP_ERROR(rclcpp::get_logger("CANMotor"), "Set command type to degree or radian in the motor hardware interface URDF");
        RCLCPP_ERROR(rclcpp::get_logger("CANMotor"), "Command type will be set to radian!");
        command_type = "radian";
    }

    // Create the motor controller object
    motor_controller_ = std::make_unique<motor_driver::MotorDriver>(
        can_id, can_bus, motor_driver::MotorType::GIM8108
    );

    unsigned int timeout_ms = static_cast<unsigned int>(thread_period.count() * 5 * 1000);
    watchdog = std::make_unique <Watchdog> (std::chrono::milliseconds(timeout_ms),
     [this]() { this->on_timeout(); });
    // 
    // wd.emplace(timeout_ms, [this]() { this->on_timeout(); });

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CANMotor::on_configure(const rclcpp_lifecycle::State& previous_state){
    // Configuration code
    if (info_.hardware_parameters.at("flip")== "true") //if flip is true, set cmd_flip to -1
    {
        cmd_flip = -1;
    }
    else if (info_.hardware_parameters.at("flip")== "false") //if flip is false, set cmd_flip to 1
    {
        cmd_flip = 1;
    }
    else if (info_.hardware_parameters.at("flip") != "true" && info_.hardware_parameters.at("flip") != "false")    {
        RCLCPP_ERROR(rclcpp::get_logger("CANMotor"), "Set motor flip to true or false in the motor hardware interface URDF");
        RCLCPP_ERROR(rclcpp::get_logger("CANMotor"), "Motor will not be flipped!");
        // return hardware_interface::CallbackReturn::ERROR;}
    }
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

    startThread();

    watchdog->start();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CANMotor::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    // Deactivation code
    stopThread();

    stateMap = motor_controller_->disableMotor(can_id);

    watchdog->stop();
    
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
    state_interfaces_.emplace_back(
        hardware_interface::StateInterface(joint_name,  "flip", &state_flip));
    state_interfaces_.emplace_back(
        hardware_interface::StateInterface(joint_name,  "watchdog", &state_watchdog));
    


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
    command_interfaces_.emplace_back(
        hardware_interface::CommandInterface(joint_name,  "flip", &cmd_flip));

    return std::move(command_interfaces_);
}

hardware_interface::return_type CANMotor::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // // Read data from the hardware and check for over limit


    //     // if effort is over safe limits, drop kp kd to zero
    //     if (state_effort > effort_limit)
    //     {
    //         RCLCPP_INFO(rclcpp::get_logger("CANMotor"), "Effort Limit Exceeded during WRITE: %f", state_effort);

    //         cmd_kp  = 0;
    //         cmd_kd = 0;
        
    //     }

    //     // Update command data from the hardware
    //     movecmd = {static_cast<float>(cmd_position*cmd_flip),
    //         static_cast<float>(cmd_velocity*cmd_flip),
    //         static_cast<float>(cmd_kp),
    //         static_cast<float>(cmd_kd),
    //         static_cast<float>(cmd_effort*cmd_flip)};

    //     commandMap[can_id[0]] = movecmd;

    return hardware_interface::return_type::OK;
}



hardware_interface::return_type CANMotor::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // // Write data to the hardware
    // // std::cout << "_START WRITE MOTOR_" << can_id[0] << std::endl;

    // // State Machine
    // if(cmd_m_state == 0)
    // // send standard cmd state
    // {

    //     stateMap = send_motor_cmd();

    //     // if(command_type=="degree"){

    //     //     stateMap = motor_controller_->sendDegreeCommand( commandMap);
    //     // }
    //     // else if (command_type == "radian")
    //     // {
    //     //     stateMap = motor_controller_->sendRadCommand( commandMap);
    //     // }
    //     // else
    //     // {
    //     //     RCLCPP_ERROR(rclcpp::get_logger("CANMotor"), "Obscure error: Command type not set to degree or radian in URDF and safety failed");
    //     // }
    

    // }
    // else if(cmd_m_state == 1)
    // // enable motor state
    // {
    //     stateMap = motor_controller_->enableMotor(can_id);
    //     cmd_m_state = 0;
    //     std::cout << "Motor: " << can_id[0] << "Enabled"<< std::endl;
    // }
    // else if(cmd_m_state == 2)
    // // disable motor state
    // {
    //     stateMap = motor_controller_->disableMotor(can_id);
    //     // cmd_m_state = 0;
    //     std::cout << "Motor: " << can_id[0] << "Disabled"<< std::endl;
    // }
    // else if(cmd_m_state == 3)
    // // zero motor state
    // {
    //     // check if zeroing would cause the motor to jump
    //     if ( abs(cmd_position) < 0.2 || (state_kp==0 and state_kd==0))
    //     {
    //         // stop motor exert 0 effort
    //         movecmd = {static_cast<float>(0),
    //                     static_cast<float>(0),
    //                     static_cast<float>(0),
    //                     static_cast<float>(0),
    //                     static_cast<float>(0)};

    //         commandMap[can_id[0]] = movecmd;
            


    //         stateMap = send_motor_cmd();

    //         // if(command_type=="degree"){

    //         //     stateMap = motor_controller_->sendDegreeCommand( commandMap);
    //         // }
    //         // else if (command_type == "radian")
    //         // {
    //         //     stateMap = motor_controller_->sendRadCommand( commandMap);
    //         // }
    //         // else
    //         // {
    //         //     RCLCPP_ERROR(rclcpp::get_logger("CANMotor"), "Obscure error: Command type not set to degree or radian in URDF and safety failed");
    //         // }
            
    //         stateMap = motor_controller_->setZeroPosition(can_id);
    //         cmd_m_state = 0;
    
    //         std::cout << "Motor: " << can_id[0] << " Zeroed"<< std::endl;
    //     }

    // }

    // //update state variables AFTER sending command
    // state_position = stateMap[can_id[0]].position * cmd_flip;
    // state_velocity = stateMap[can_id[0]].velocity * cmd_flip;
    // state_effort = stateMap[can_id[0]].torque * cmd_flip;
    // state_kp = cmd_kp;
    // state_kd = cmd_kd;
    // state_m_state = cmd_m_state;
    // state_flip = cmd_flip;


    // // RCLCPP_INFO(rclcpp::get_logger("CANMotor"), "motor write: %d", can_id[0]);



    // // LOGGING
    // // std::cout << "_COMMANDS MOTOR_" << can_id[0] << std::endl;
    // // std::cout << "Commanded position: " << cmd_position << std::endl;
    // // std::cout << "Commanded velocity: " << cmd_velocity << std::endl;
    // // std::cout << "Commanded kp: " << cmd_kp << std::endl;
    // // std::cout << "Commanded kd: " << cmd_kd << std::endl;
    // // std::cout << "Commanded effort: " << cmd_effort << std::endl;
    // // std::cout << "Commanded state: " << cmd_m_state << std::endl;

    // // std::cout << "_STATES MOTOR_" << can_id[0] << std::endl;
    // // std::cout << "State position: " << state_position << std::endl;
    // // std::cout << "State velocity: " << state_velocity << std::endl;
    // // std::cout << "State torque: " << state_effort << std::endl;
    // // std::cout << "State kp: " << state_kp << std::endl;
    // // std::cout << "State kd: " << state_kd << std::endl;
    // // std::cout << "State m_state: " << state_m_state << std::endl;



    // // motor_controller_->disableMotor(can_id);

    // sleep(1);
    return hardware_interface::return_type::OK;
}


}  // namespace quadruped_hardware

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(quadruped_hardware::CANMotor, hardware_interface::ActuatorInterface)