// based on ROS2 control demo example 7
// https://github.com/ros-controls/ros2_control_demos/blob/master/example_7/controller/r6bot_controller.cpp


#include "quadruped_utils/ZeroJointsController.hpp"
#include <cassert>


// using namespace quadruped_hardware; 

using config_type = controller_interface::interface_configuration_type;

namespace quadruped_utils

{

       
void deactivate_controller(const std::string &controller_name)
    {
    auto node = rclcpp::Node::make_shared("deactivate_controller_node");
    auto client = node->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->deactivate_controllers.push_back(controller_name);
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Successfully deactivated controller: %s", controller_name.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to deactivate controller: %s", controller_name.c_str());
    }
    }


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


            // initialize zero status vector: 0 =zeroed, 1 = not zeroed
            zero_status = std::vector<int>(joint_names_.size(), 1);


            // for (const auto & interface_type : command_interface_types_) {


            //     std::string interface_name = interface_type;
            //     std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interface_vector;

            //     command_interface_map_[interface_name] = &command_interface_vector;
            // }




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


    // clear out vectors in case of restart
    joint_position_command_interface_.clear();
    joint_velocity_command_interface_.clear();
    joint_m_state_command_interface_.clear();

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

    return CallbackReturn::SUCCESS;
    }
     

controller_interface::return_type ZeroJointController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
    RCLCPP_INFO(get_node()->get_logger(), "___ UPDATING ZERO JOINT CONTROLLER ___");
    float pos = 10;
    float vel = 10;


    // if all elements of zero status are 0, then end the controller
    bool all_zero = true;
    bool success;
    // for all joints
    for (size_t i = 0; i < joint_names_.size(); ++i) 
        {
            // if not zeroed
            if (zero_status[i] == 1)
            {
                auto effort = joint_effort_state_interface_[i].get().get_value();

                // check if run into limit
                if (effort < 0.6)
                { 
                    RCLCPP_INFO(get_node()->get_logger(), "JOINT %s EFFORT LIMIT EXCEEDED IN ZERO: %f", joint_names_[i].c_str(),effort);

                    // set Velocity to zero
                    bool success = joint_velocity_command_interface_[i].get().set_value(0);
                    assert(success);
                    // Zero motor
                    success = joint_m_state_command_interface_[i].get().set_value(3);
                    assert(success);

                    zero_status[i] = 0;
                } 
            }
            // if still not zeroed
            if (zero_status[i] == 1)
            {
                all_zero = false;
                
                bool success = joint_velocity_command_interface_[i].get().set_value(vel);
                assert(success);
            }
        }

        if (all_zero)
        {
            deactivate_controller("zero_joints_controller"); 
        }

        
        return controller_interface::return_type::OK;
    }


 
controller_interface::CallbackReturn ZeroJointController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {




    return CallbackReturn::SUCCESS;
    }

} // namespace quadruped_utils

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_utils::ZeroJointController, controller_interface::ControllerInterface)