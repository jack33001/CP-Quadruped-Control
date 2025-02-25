// based on ROS2 control demo example 7
// https://github.com/ros-controls/ros2_control_demos/blob/master/example_7/controller/r6bot_controller.cpp


#include "quadruped_utils/ZeroJointsController.hpp"
#include <cassert>


// using namespace quadruped_hardware; 

using config_type = controller_interface::interface_configuration_type;

namespace quadruped_utils

{


ZeroJointController::ZeroJointController(): controller_interface::ControllerInterface()
    {
        RCLCPP_INFO(rclcpp::get_logger("ZeroJointController"), "XXX___XXX Starting ZeroJointController contructor");
       
    }

controller_interface::CallbackReturn ZeroJointController::on_init()
    {
        RCLCPP_INFO(rclcpp::get_logger("ZeroJointController"), "Starting ZeroJointController on_init");

        // Get parameters from yaml
        try {
            fprintf(stderr, "Zero controller trying to declare parameters\n");
            
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);

            // zero status is vector of size joint_names_ with all values set to 1
            zero_status.resize(joint_names_.size(), 1);
            

            command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);


            for (const auto & interface_type : command_interface_types_) {

                // std::string interface_name = "joint_" + interface_type + "_command_interface_";
                std::string interface_name = interface_type;
                std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interface_vector;

                command_interface_map_[interface_name] = &command_interface_vector;

                // print command interface name
                RCLCPP_INFO(get_node()->get_logger(), "Command interface init: %s", interface_name.c_str());
            }


            for (const auto & interface_type : state_interface_types_) {

                // std::string interface_name = "joint_" + interface_type + "_command_interface_";
                std::string interface_name = interface_type;
                std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interface_vector;

                state_interface_map_[interface_name] = &state_interface_vector;

                // print command interface name
                RCLCPP_INFO(get_node()->get_logger(), "State interface init: %s", interface_name.c_str());
            }


            // // print command and state interface types
            // RCLCPP_INFO(get_node()->get_logger(), "Command interface types:");
            // for (const auto & command_interface_type : command_interface_types_) {
            //     RCLCPP_INFO(get_node()->get_logger(), "Command interface type: %s", command_interface_type.c_str());
            // }
            // RCLCPP_INFO(get_node()->get_logger(), "State interface types:");
            // for (const auto & state_interface_type : state_interface_types_) {
            //     RCLCPP_INFO(get_node()->get_logger(), "State interface type: %s", state_interface_type.c_str());
            // }


            if (joint_names_.empty()) {
                RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
                return CallbackReturn::ERROR;
            }

            RCLCPP_INFO(get_node()->get_logger(), "ZeroJointController initilized with %zu joints", joint_names_.size());

  
            return CallbackReturn::SUCCESS;
        } catch (const std::exception & e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
    }
    
controller_interface::CallbackReturn ZeroJointController::on_configure(const rclcpp_lifecycle::State & previous_state) 
    {

    return CallbackReturn::SUCCESS;
    }


// ________INTERFACE CONFIGURATION________
controller_interface::InterfaceConfiguration ZeroJointController::command_interface_configuration() const
    {
    // claim individual command interfaces for each joint and type
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

    conf.names.reserve(joint_names_.size() * command_interface_types_.size());
    // for each joint name
    for (const auto & joint_name : joint_names_)
    {
        // for each interface type
        for (const auto & interface_type : command_interface_types_)
        {
        conf.names.push_back(joint_name + "/" + interface_type);

        // print command interface name
        RCLCPP_INFO(get_node()->get_logger(), "Command interface config: %s", (joint_name + "/" + interface_type).c_str());
        }
    }

    return conf;
    }

controller_interface::InterfaceConfiguration ZeroJointController::state_interface_configuration() const
    {
    // claim individual state interfaces for each joint and type
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

    // reserve all types of state interfaces for each joint
    conf.names.reserve(joint_names_.size() * state_interface_types_.size());

    // for each joint name
    for (const auto & joint_name : joint_names_)
    {
        // for each interface type
        for (const auto & interface_type : state_interface_types_)
        {
        conf.names.push_back(joint_name + "/" + interface_type);
        }
    }

    return conf;
    }

    // ___________________________

controller_interface::CallbackReturn ZeroJointController::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
    // clear out vectors in case of restart
    joint_position_command_interface_.clear();
    joint_velocity_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    joint_effort_state_interface_.clear();


    // // assign command interfaces
    for (auto & interface : command_interfaces_)
    {
        command_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

    // assign state interfaces
    for (auto & interface : state_interfaces_)
    {
        state_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

    return CallbackReturn::SUCCESS;
    }
     

controller_interface::return_type ZeroJointController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
    RCLCPP_INFO(get_node()->get_logger(), "___________________ UPDATING ZERO JOINT CONTROLLER ___________________");
    float pos = 10;
    float vel = 0;


    // for each joint name
    for (size_t i = 0; i < joint_names_.size(); ++i) 
    {
        // set position command interface to 0

        auto &command_interface = command_interface_map_["position"]->at(i).get();
        RCLCPP_INFO(get_node()->get_logger(), "Data type of command interface: %s", typeid(command_interface).name());

        bool success = command_interface_map_["position"]->at(i).get().set_value(pos);
        // bool success = joint_position_command_interface_[i].get().set_value(pos);
        if (!success) {
            RCLCPP_ERROR(rclcpp::get_logger("ZeroJointController"), "Failed to set position value for joint %zu", i);
            return controller_interface::return_type::ERROR;
        }
    }

   
    
    


    // for (size_t i = 0; i < joint_position_command_interface_.size(); ++i) 
    // {
    //     bool success = joint_position_command_interface_[i].get().set_value(pos);
    //     if (!success) {
    //         RCLCPP_ERROR(rclcpp::get_logger("ZeroJointController"), "Failed to set position value for joint %zu", i);
    //         return controller_interface::return_type::ERROR;
    //     }
    // }

   

   

    if (true)
    {
    // print command_interfaces
    for (const auto & command_interface : command_interfaces_)
        {
        RCLCPP_INFO(get_node()->get_logger(), "Command interface %s: %f", command_interface.get_name().c_str(), command_interface.get_value());
        }

    for (const auto & state_interface : state_interfaces_)
        {
        RCLCPP_INFO(get_node()->get_logger(), "State interface %s: %f", state_interface.get_name().c_str(), state_interface.get_value());
        }

    }

    return controller_interface::return_type::OK;
    }

controller_interface::CallbackReturn ZeroJointController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
    RCLCPP_INFO(get_node()->get_logger(), "on_deactivate called");
    return CallbackReturn::SUCCESS;
    }

} // namespace quadruped_utils

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_utils::ZeroJointController, controller_interface::ControllerInterface)