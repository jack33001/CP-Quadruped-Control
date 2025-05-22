// #include "controller_interface/controller_interface.hpp"
// #include "hardware_interface/loaned_state_interface.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_lifecycle/state.hpp"

// namespace estopcontroller
// {

// class EstopController : public controller_interface::ControllerInterface
// {
// public:
// EstopController() = default;

//   controller_interface::InterfaceConfiguration command_interface_configuration() const override
//   {
//     return controller_interface::InterfaceConfiguration{
//       controller_interface::interface_configuration_type::NONE};
//   }

//   controller_interface::InterfaceConfiguration state_interface_configuration() const override
//   {
//     controller_interface::InterfaceConfiguration interfaces_config;
//     interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
//     // External state interface to monitor
//     interfaces_config.names.push_back("external_hardware/state_name");
    
//     // Our Jetson GPIO interface state to update
//     interfaces_config.names.push_back("jetson_gpio/monitored_state");
    
//     return interfaces_config;
//   }

//   controller_interface::return_type update(
//     const rclcpp::Time & time, const rclcpp::Duration & period) override
//   {
//     // Read value from the external hardware interface
//     double external_state = state_interfaces_[0].get_value();
    
//     // Update our monitored_state with the external state value
//     state_interfaces_[1].set_value(external_state);
    
//     return controller_interface::return_type::OK;
//   }

//   // Other required lifecycle methods...
// };

// }  // namespace state_bridge_controller

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(
//   state_bridge_controller::EstopController,
//   controller_interface::ControllerInterface
// )