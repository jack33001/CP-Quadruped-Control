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

            command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);


            for (const auto & interface_type : command_interface_types_) {

                // std::string interface_name = "joint_" + interface_type + "_command_interface_";
                std::string interface_name = interface_type;
                std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interface_vector;

                command_interface_map_[interface_name] = &command_interface_vector;
            }


            // print command and state interface types
            RCLCPP_INFO(get_node()->get_logger(), "Command interface types:");
            for (const auto & command_interface_type : command_interface_types_) {
                RCLCPP_INFO(get_node()->get_logger(), "Command interface type: %s", command_interface_type.c_str());
            }
            RCLCPP_INFO(get_node()->get_logger(), "State interface types:");
            for (const auto & state_interface_type : state_interface_types_) {
                RCLCPP_INFO(get_node()->get_logger(), "State interface type: %s", state_interface_type.c_str());
            }


            if (joint_names_.empty()) {
                RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
                return CallbackReturn::ERROR;
            }
            RCLCPP_INFO(get_node()->get_logger(), "ZeroJointController configured with %zu joints", joint_names_.size());

  
            


            return CallbackReturn::SUCCESS;
        } catch (const std::exception & e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
    }
    
controller_interface::CallbackReturn ZeroJointController::on_configure(const rclcpp_lifecycle::State & previous_state) 
    {
    // auto callback =
    //     [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
    // {
    //     traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
    //     new_msg_ = true;
    // };

    // joint_command_subscriber_ =
    //     get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    //     "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

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
        // RCLCPP_INFO(get_node()->get_logger(), "on_activate called");

        // try {
        //     // Resize joint states vector if needed
        //     if (joint_states_.size() != joint_names_.size()) {
        //       joint_states_.resize(joint_names_.size());
        //     }
        // } catch (const std::exception& e) {
        //     RCLCPP_ERROR(get_node()->get_logger(), "Error reading state interfaces: %s", e.what());
        //     return CallbackReturn::ERROR;
        //   }

        // RCLCPP_INFO(get_node()->get_logger(), "State interfaces:");
        // for (const auto & state_interface : state_interfaces_) {
        //     RCLCPP_INFO(get_node()->get_logger(), "State interface: %s", state_interface.get_name().c_str());
        // }

    // clear out vectors in case of restart
    joint_position_command_interface_.clear();
    joint_velocity_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();


    // print all command interfaces
    RCLCPP_INFO(get_node()->get_logger(), "Activating with command interfaces:");
    for (const auto & command_interface : command_interfaces_)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Command interface: %s", command_interface.get_name().c_str());  
    }

    // print size of command interfaces
    RCLCPP_INFO(get_node()->get_logger(), "command_interfaces_ size: %zu", command_interfaces_.size());

    // print all state interfaces
    RCLCPP_INFO(get_node()->get_logger(), "Activating with state interfaces:");
    for (const auto & state_interface : state_interfaces_)
    {
        RCLCPP_INFO(get_node()->get_logger(), "State interface: %s", state_interface.get_name().c_str());  
    }

    //print command interface map keys
    RCLCPP_INFO(get_node()->get_logger(), "Command interface map keys:");
    for (const auto & command_interface_map : command_interface_map_)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Command interface map key: %s", command_interface_map.first.c_str());
    } 

    // print state interface map keys
    RCLCPP_INFO(get_node()->get_logger(), "State interface map keys:");
    for (const auto & state_interface_map : state_interface_map_)
    {
        RCLCPP_INFO(get_node()->get_logger(), "State interface map key: %s", state_interface_map.first.c_str());
    }


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

    // print joint position command interfaces
    // RCLCPP_INFO(get_node()->get_logger(), "Joint position command interfaces:");
    // for (size_t i = 0; i < joint_position_command_interface_.size(); i++)
    // {
    //   RCLCPP_INFO(get_node()->get_logger(), "Joint position command interface: %s", joint_position_command_interface_[i].get().get_name().c_str());
    // }

    // // print joint position command interface size
    // RCLCPP_INFO(get_node()->get_logger(), "Joint position command interface size: %zu", joint_position_command_interface_.size());

    // command_interfaces_[0].set_value(pos);

    // for (size_t i = 0; i < joint_position_command_interface_.size(); ++i) 
    //     {
    //         bool success = joint_position_command_interface_[i].get().set_value(pos);
            
    //         if (!success) {
    //             RCLCPP_ERROR(rclcpp::get_logger("ZeroJointController"), "Failed to set position value for joint %zu", i);
    //             return controller_interface::return_type::ERROR;
    //         }
    //     }

    // for each element of the map
    for (const auto & command_interface_map : command_interface_map_)
    {
        // print command interface map key
        // RCLCPP_INFO(get_node()->get_logger(), "Command interface map key: %s", command_interface_map.first.c_str());

        // check what data type command_interface_map is
        // RCLCPP_INFO(get_node()->get_logger(), "XXXXXXXXXXXXXXXXXXXXXXXXXX: %s", typeid(command_interface_map.second).name());

        // for each element of the vector
        for (const auto & command_interface : *command_interface_map.second)
        {

            // RCLCPP_INFO(get_node()->get_logger(), "Command interface name: %s",command_interface.get().get_name().c_str());
            // RCLCPP_INFO(get_node()->get_logger(), "Command interface type: %s", command_interface.get().get_interface_name().c_str());

            // print command interface name
            // RCLCPP_INFO(get_node()->get_logger(), "Command interface name: %s", command_interface.get().get_name().c_str());
            // set value to 0
            // print the command interface type

            // print what type command_interface is
            // RCLCPP_INFO(get_node()->get_logger(), "Command interface type: %s", *command_interface);
            // RCLCPP_INFO(get_node()->get_logger(), "Command interface type: %s", command_interface.get().get_interface_name().c_str());
            
            // RCLCPP_INFO(get_node()->get_logger(), "XXXXXXXXXXXXXXXXXXXXXXXXXX: %s", typeid(command_interface).name());
            bool success = command_interface.get().set_value(pos);
            assert(success);

            if (!success) {
                RCLCPP_ERROR(rclcpp::get_logger("ZeroJointController"), "Failed to set position value for joint ");
                return controller_interface::return_type::ERROR;
            }
        }
    }



    for (size_t i = 0; i < joint_velocity_command_interface_.size(); ++i) 
        {
            bool success = joint_velocity_command_interface_[i].get().set_value(vel);
            if (!success) {
                RCLCPP_ERROR(rclcpp::get_logger("ZeroJointController"), "Failed to set velocity value for joint %zu", i);
                return controller_interface::return_type::ERROR;
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