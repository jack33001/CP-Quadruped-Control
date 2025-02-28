#include "quadruped_mpc/controller_interfaces/gait_pattern_generator.hpp"
#include "quadruped_mpc/control_laws/GaitPatternGenerator.hpp"

namespace quadruped_mpc
{

GaitPatternGenerator::GaitPatternGenerator()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration 
GaitPatternGenerator::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return config;
}

controller_interface::InterfaceConfiguration 
GaitPatternGenerator::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return config;
}

controller_interface::return_type
GaitPatternGenerator::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!unpack_state()) {
        return controller_interface::return_type::ERROR;
    }

    if (!update_foot_phase()) {
        return controller_interface::return_type::ERROR;
    }

    if (!support_polygon()) {
        return controller_interface::return_type::ERROR;
    }

    if (!publish_pattern()) {
        return controller_interface::return_type::ERROR;
    }

  return controller_interface::return_type::OK;
}

auto GaitPatternGenerator::on_init() -> CallbackReturn
{
  try {
    // Initialize publishers first since we want to publish in init state
    auto foot_state_pub = get_node()->create_publisher<quadruped_msgs::msg::FootStates>(
      "/quadruped/state/fsm/foot_states", 10);
    
    rt_foot_state_pub_ = std::make_unique<RTFootStatePublisher>(foot_state_pub);
    foot_state_msg_ = std::make_shared<quadruped_msgs::msg::FootStates>();

    // Initialize with all feet in stance
    if (rt_foot_state_pub_ && rt_foot_state_pub_->trylock()) {
      auto& msg = rt_foot_state_pub_->msg_;
      msg.foot1_state = 0;
      msg.foot2_state = 0;
      msg.foot3_state = 0;
      msg.foot4_state = 0;
      rt_foot_state_pub_->unlockAndPublish();
    }

    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
}

auto GaitPatternGenerator::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn
{
  try {
    // Set up state subscription
    state_sub_ = get_node()->create_subscription<quadruped_msgs::msg::QuadrupedState>(
      "/quadruped/state/state_estimate",
      rclcpp::QoS(1).reliable(),
      [this](const quadruped_msgs::msg::QuadrupedState::SharedPtr msg) {
        latest_state_ = msg;
      }
    );

    // Create regular publisher first
    auto gait_pub = get_node()->create_publisher<quadruped_msgs::msg::GaitPattern>(
      "/quadruped/gait/gait_pattern", 10);
    
    // Create realtime publisher
    rt_gait_pub_ = std::make_unique<RTPublisher>(gait_pub);
    gait_msg_ = std::make_shared<quadruped_msgs::msg::GaitPattern>();

    RCLCPP_INFO(get_node()->get_logger(), "Gait pattern generator configured successfully");
    return CallbackReturn::SUCCESS;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during configure stage with message: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

auto GaitPatternGenerator::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn
{
  return CallbackReturn::SUCCESS;
}

auto GaitPatternGenerator::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn
{
  return CallbackReturn::SUCCESS;
}

}  // namespace quadruped_mpc

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  quadruped_mpc::GaitPatternGenerator, controller_interface::ControllerInterface)
