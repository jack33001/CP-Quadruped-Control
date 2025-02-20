#include "quadruped_utils/ZeroJointsController.hpp"

namespace quadruped_utils
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


    ZeroJointController::ZeroJointController()
    : controller_interface::ControllerInterface()
    {
        RCLCPP_INFO(rclcpp::get_logger("ZeroJointController"), "XXX___XXX Starting ZeroJointController contructor");
       
    }





    // ________INTERFACE CONFIGURATION________
    controller_interface::InterfaceConfiguration ZeroJointController::command_interface_configuration() const
    {
        
        // const auto left_result =
        //     configure_side("left", params_.left_wheel_names, registered_motor_handles_);
        

        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        
        // Add command interfaces for each joint
        for (const auto & joint : joint_names_) {
            config.names.push_back(joint + "/velocity");
            RCLCPP_INFO(get_node()->get_logger(), "Adding command interface for joint %s", joint.c_str());
        }
        


        return config;
    
    }

    controller_interface::InterfaceConfiguration ZeroJointController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        
        // Add command interfaces for each joint
        for (const auto & joint : joint_names_) {
            config.names.push_back(joint + "/effort");
            RCLCPP_INFO(get_node()->get_logger(), "Adding state interface for joint %s", joint.c_str());
        }

        return config;
    }

    // void controller_interface::InterfaceConfiguration ZeroJointController::cmd_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    //     {
    //     std::lock_guard<std::mutex> lock(cmd_mutex_);
    //     latest_cmd_ = msg;
    //     new_cmd_received_ = true;
    //     }

    // void controller_interface::InterfaceConfiguration ZeroJointController::state_callback(const quadruped_msgs::msg::QuadrupedState::SharedPtr msg)
    //     {
    //     std::lock_guard<std::mutex> lock(state_mutex_);
    //     latest_state_ = msg;
    //     new_state_received_ = true;
    //     }


    // ________CONTROLLER LIFECYCLE FUNCTIONS________
    controller_interface::CallbackReturn ZeroJointController::on_init()
    {

        RCLCPP_INFO(rclcpp::get_logger("ZeroJointController"), "Starting ZeroJointController on_init");

        // auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());

        
      



        try {
            fprintf(stderr, "Balance controller trying to declare parameters\n");
            // Get parameters from yaml
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("state_interfaces", std::vector<std::string>());
            fprintf(stderr, "Balance controller parameters declared\n");
            
            fprintf(stderr, "Balance controller on_init completed successfully\n");
            fprintf(stderr, "Balance controller waiting for on_configure\n");
            return CallbackReturn::SUCCESS;
        } catch (const std::exception & e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
      
        // return controller_interface::CallbackReturn::SUCCESS;
    }
    
    controller_interface::CallbackReturn ZeroJointController::on_configure(const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(get_node()->get_logger(), "Configuring ZeroJointController...");

        // List all parameters
        auto parameters = get_node()->list_parameters({}, 10);
        RCLCPP_INFO(get_node()->get_logger(), "Listing all parameters:");
        for (const auto & name : parameters.names) {
            RCLCPP_INFO(get_node()->get_logger(), "Parameter: %s", name.c_str());
            auto param_value = get_node()->get_parameter(name).value_to_string();
            RCLCPP_INFO(get_node()->get_logger(), "Parameter value: %s", param_value.c_str());
        }
        
        
        // Check if joints parameter is set from yaml and initilized
        if (!get_node()->has_parameter("joints")) {
            RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' not set");
            return CallbackReturn::ERROR;
        }
        state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();

        // update joint_names with all motors in use
        joint_names_ = get_node()->get_parameter("joints").as_string_array();
        if (joint_names_.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_node()->get_logger(), "ZeroJointController configured with %zu joints", joint_names_.size());

        // Add subscriber initialization before returning
        // cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
        //     "zero_joints_velocity", 10,
        //     std::bind(&ZeroJointController::cmd_callback, this, std::placeholders::_1));

        // RCLCPP_INFO(get_node()->get_logger(), "Subscribed to quadruped/cmd/single_state topic");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ZeroJointController::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(get_node()->get_logger(), "on_activate called");

        try {
            // Resize joint states vector if needed
            if (joint_states_.size() != joint_names_.size()) {
              joint_states_.resize(joint_names_.size());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Error reading state interfaces: %s", e.what());
            return CallbackReturn::ERROR;
          }

        RCLCPP_INFO(get_node()->get_logger(), "State interfaces:");
        for (const auto & state_interface : state_interfaces_) {
            RCLCPP_INFO(get_node()->get_logger(), "State interface: %s", state_interface.get_name().c_str());
        }
        
        
    
        // // Get command and state interfaces
        // for (const auto & joint : joint_names_) {
        //     auto command_interface = get_node()->get_command_interface(joint + "/velocity");
        //     if (!command_interface) {
        //         RCLCPP_ERROR(get_node()->get_logger(), "Failed to get command interface for joint %s", joint.c_str());
        //         return CallbackReturn::ERROR;
        //     }
        //     command_interfaces_.push_back(hardware_interface::LoanedCommandInterface(command_interface));
    
        //     auto state_interface = get_node()->get_state_interface(joint + "/effort");
        //     if (!state_interface) {
        //         RCLCPP_ERROR(get_node()->get_logger(), "Failed to get state interface for joint %s", joint.c_str());
        //         return CallbackReturn::ERROR;
        //     }
        //     state_interfaces_.push_back(hardware_interface::LoanedStateInterface(state_interface));
        // }
    
        // RCLCPP_INFO(get_node()->get_logger(), "Zero Joints controller activated");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ZeroJointController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
        {
        RCLCPP_INFO(get_node()->get_logger(), "on_deactivate called");
        return CallbackReturn::SUCCESS;
        }


    controller_interface::return_type ZeroJointController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "___________________ UPDATING ZERO JOINT CONTROLLER ___________________");
        RCLCPP_INFO(get_node()->get_logger(), "State interface types:");
        for (const auto & type : state_interface_types_) {
            RCLCPP_INFO(get_node()->get_logger(), "State interface type: %s", type.c_str());
        }

        RCLCPP_INFO(get_node()->get_logger(), "Joint states:");
        fprintf(stderr, "Joint states size: %zu\n", joint_states_.size());
        fprintf(stderr, "Joint names size: %zu\n", joint_names_.size());


        for (size_t i = 0; i < joint_states_.size(); ++i) {
            RCLCPP_INFO(get_node()->get_logger(), "Joint %s - Position: %f, Velocity: %f, Effort: %f",
                        joint_names_[i].c_str(),
                        joint_states_[i].position,
                        joint_states_[i].velocity,
                        joint_states_[i].effort);
        }


        // try {
        //     // Resize joint states vector if needed
        //     if (joint_states_.size() != joint_names_.size()) {
        //       joint_states_.resize(joint_names_.size());
        //     }
            
        //     // Read all interfaces for each joint
        //     const size_t interfaces_per_joint = state_interface_types_.size();
        //     for (size_t i = 0; i < joint_names_.size(); ++i) {
        //       size_t base_idx = i * interfaces_per_joint;
        //       // Update local storage only
        //       joint_states_[i].position = state_interfaces_[base_idx].get_value();      // position
        //       joint_states_[i].velocity = state_interfaces_[base_idx + 1].get_value();  // velocity
        //       joint_states_[i].effort = state_interfaces_[base_idx + 2].get_value();    // effort
        //     }
        // }
        // catch (const std::exception & e) {
        //     fprintf(stderr, "Exception thrown during update stage with message: %s \n", e.what());
        //     return controller_interface::return_type::ERROR;
        // }




        RCLCPP_INFO(get_node()->get_logger(), "Command interfaces and their values:");
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            RCLCPP_INFO(get_node()->get_logger(), "Command interface %zu: %f", i, command_interfaces_[i].get_value());
        }
        
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            double state_effort = state_interfaces_[i].get_value();

            RCLCPP_INFO(get_node()->get_logger(), "Updating joint %s", joint_names_[i].c_str());
            RCLCPP_INFO(get_node()->get_logger(), "State interface: %f", state_effort);


            
            // RCLCPP_INFO(get_node()->get_logger(), "Command effort: %f", command_position);

            // double command_position = state_position; // Example: command position is the same as state position
            // command_interfaces_[i].set_value(command_position);
        }
        RCLCPP_INFO(get_node()->get_logger(), "___________________ UPDATING COMPLETE ___________________");

        return controller_interface::return_type::OK;
    }


} // namespace quadruped_utils

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_utils::ZeroJointController, controller_interface::ControllerInterface)