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
      

    controller_interface::InterfaceConfiguration ZeroJointController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            "fr_hip/velocity",

        };
        return config;
    }

    controller_interface::InterfaceConfiguration ZeroJointController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            "fr_hip/velocity",

        };
        return config;
    }

    controller_interface::return_type ZeroJointController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
    //     for (size_t i = 0; i < command_interfaces_.size(); ++i)
    //     {
    //         double state_position = state_interfaces_[i].get_value();
    //         double command_position = state_position; // Example: command position is the same as state position
    //         command_interfaces_[i].set_value(command_position);
    //     }
        return controller_interface::return_type::OK;
    }


} // namespace quadruped_utils

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_utils::ZeroJointController, controller_interface::ControllerInterface)