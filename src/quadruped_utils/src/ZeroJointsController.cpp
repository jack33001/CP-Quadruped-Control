#include "quadruped_utils/ZeroJointsController.hpp"

namespace quadruped_utils
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


    ZeroJointController::ZeroJointController()
    : controller_interface::ControllerInterface()
    {
        RCLCPP_INFO(rclcpp::get_logger("ZeroJointController"), "XXX___XXX Starting ZeroJointController contructor");
       
    }

    controller_interface::CallbackReturn ZeroJointController::on_init()
    {

        RCLCPP_INFO(rclcpp::get_logger("ZeroJointController"), "Starting ZeroJointController on_init");

        auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());

        // auto ret = forward_command_controller::ForwardCommandController::on_init();
        // if (ret != controller_interface::CallbackReturn::SUCCESS)
        // {
        //   return ret;
        // }
      
        // try
        // {
        //   // Explicitly set the interface parameter declared by the forward_command_controller
        //   // to match the value set in the JointGroupEffortController constructor.
        //   get_node()->set_parameter(
        //     rclcpp::Parameter("interface_name", hardware_interface::HW_IF_EFFORT));
        // }
        // catch (const std::exception & e)
        // {
        //   fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        //   return controller_interface::CallbackReturn::ERROR;
        // }
      
        return controller_interface::CallbackReturn::SUCCESS;
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
        

        if (!get_node()->has_parameter("joints")) {
            RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' not set");
            return CallbackReturn::ERROR;
        }

        joint_names_ = get_node()->get_parameter("joints").as_string_array();
        if (joint_names_.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_node()->get_logger(), "ZeroJointController configured with %zu joints", joint_names_.size());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration ZeroJointController::command_interface_configuration() const
    {
        // controller_interface::InterfaceConfiguration config;
        // config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        // config.names = {
        //     "fr_hip/velocity",

        // };
        // return config;

        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        
        // Add command interfaces for each joint
        for (const auto & joint : joint_names_) {
            config.names.push_back(joint + "/velocity");
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
        }

        return config;
    }

    controller_interface::return_type ZeroJointController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "___________________ UPDATING ZERO JOINT CONTROLLER ___________________");
        
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