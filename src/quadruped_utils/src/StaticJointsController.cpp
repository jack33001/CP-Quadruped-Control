// based on ROS2 control demo example 7
// https://github.com/ros-controls/ros2_control_demos/blob/master/example_7/controller/r6bot_controller.cpp


#include "quadruped_utils/StaticJointsController.hpp"
#include <cassert>


// using namespace quadruped_hardware; 

using config_type = controller_interface::interface_configuration_type;

namespace quadruped_utils

{



StaticJointsController::StaticJointsController(): controller_interface::ControllerInterface()
    {
        RCLCPP_INFO(rclcpp::get_logger("StaticJointsController"), "XXX___XXX Starting StaticPoseController contructor");
       
    }

controller_interface::CallbackReturn StaticJointsController::on_init()
    {
        RCLCPP_INFO(rclcpp::get_logger("StaticJointsController"), "Starting StaticPoseController on_init");

        // Get parameters from yaml
        try {

            alt = false;
            fprintf(stderr, "static controller trying to declare parameters\n");
            
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            // zero_direction_ = auto_declare<std::vector<double>>("zero_direction", std::vector<double>(joint_names_.size(), 1.0));
            pose_position = auto_declare<std::vector<double>>("positions", std::vector<double>(joint_names_.size(), 1.0));
            pose_velocity = auto_declare<std::vector<double>>("velocities", std::vector<double>(joint_names_.size(), 1.0));
            


            command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // zero_effort_lim = auto_declare<double>("zero_effort_lim", zero_effort_lim);
            kp = auto_declare<double>("kp", kp);
            kd = auto_declare<double>("kd", kd);

            vel = auto_declare<double>("velocity", vel);


            // zeroed_kp = auto_declare<double>("zeroed_kp", kp);
            // zeroed_kd = auto_declare<double>("zeroed_kd", kd);



            // initialize zero status vector: 0 =zeroed, 1 = not zeroed
            zero_status = std::vector<int>(joint_names_.size(), 1);

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
            RCLCPP_INFO(get_node()->get_logger(), "StaticJointsController configured with %zu joints", joint_names_.size());

            return CallbackReturn::SUCCESS;
        } catch (const std::exception & e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
    }
    
controller_interface::CallbackReturn StaticJointsController::on_configure(const rclcpp_lifecycle::State & previous_state) 
    {

    return CallbackReturn::SUCCESS;
    }


// ________INTERFACE CONFIGURATION________
controller_interface::InterfaceConfiguration StaticJointsController::command_interface_configuration() const
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

controller_interface::InterfaceConfiguration StaticJointsController::state_interface_configuration() const
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

controller_interface::CallbackReturn StaticJointsController::on_activate(const rclcpp_lifecycle::State & previous_state)
    {


    // clear out vectors in case of restart
    joint_position_command_interface_.clear();
    joint_velocity_command_interface_.clear();
    joint_kp_command_interface_.clear();
    joint_kd_command_interface_.clear();
    joint_m_state_command_interface_.clear();
    joint_flip_command_interface_.clear();

    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    joint_effort_state_interface_.clear();


    // assign command interfaces
    for (auto & interface : command_interfaces_)
    {
        command_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

    // assign state interfaces
    for (auto & interface : state_interfaces_)
    {
        state_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

    // Set motor state to receive commands  
    for (size_t i = 0; i < joint_names_.size(); ++i) 
        {
        success = joint_m_state_command_interface_[i].get().set_value(0.0);
        assert(success); 
        }

    return CallbackReturn::SUCCESS;
    }
     

controller_interface::return_type StaticJointsController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {

    double delta = 0.1;
    // for all joints

    for (size_t i = 0; i < joint_names_.size(); ++i) 
        {
            success = joint_position_command_interface_[i].get().set_value(pose_position[i]);
            assert(success);
            success = joint_velocity_command_interface_[i].get().set_value(pose_velocity[i]);
            assert(success);
            success = joint_kp_command_interface_[i].get().set_value(kp);
            assert(success);   
            success = joint_kd_command_interface_[i].get().set_value(kd);
            assert(success);
        }

    // if(alt==false) {
    //     for (size_t i = 0; i < joint_names_.size(); ++i) 
    //     {
    //         success = joint_position_command_interface_[i].get().set_value(pose_position[i]);
    //         assert(success);
    //         success = joint_velocity_command_interface_[i].get().set_value(pose_velocity[i]);
    //         assert(success);
    //         success = joint_kp_command_interface_[i].get().set_value(kp);
    //         assert(success);   
    //         success = joint_kd_command_interface_[i].get().set_value(kd);
    //         assert(success);
    //     }
    //     alt=true;
    // } 
    // else if(alt==true) {
    //     for (size_t i = 0; i < joint_names_.size(); ++i) 
    //     {
    //         success = joint_position_command_interface_[i].get().set_value(pose_position[i]+delta);
    //         assert(success);
    //         success = joint_velocity_command_interface_[i].get().set_value(pose_velocity[i]);
    //         assert(success);
    //         success = joint_kp_command_interface_[i].get().set_value(kp);
    //         assert(success);   
    //         success = joint_kd_command_interface_[i].get().set_value(kd);
    //         assert(success);
    //     }
    //     alt=false;
    // }


        return controller_interface::return_type::OK;
    }


 
controller_interface::CallbackReturn StaticJointsController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
    for (size_t i = 0; i < joint_names_.size(); ++i) 
        {
        success = joint_kp_command_interface_[i].get().set_value(zeroed_kp);
        assert(success);   
        success = joint_kd_command_interface_[i].get().set_value(zeroed_kd);
        assert(success);
        success = joint_velocity_command_interface_[i].get().set_value(0.0);
        assert(success);
        }


    return CallbackReturn::SUCCESS;
    }

} // namespace quadruped_utils

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_utils::StaticJointsController, controller_interface::ControllerInterface)