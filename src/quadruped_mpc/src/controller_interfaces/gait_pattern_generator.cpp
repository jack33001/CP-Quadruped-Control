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
GaitPatternGenerator::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // Pass the time information to the functions that need it
    if (!unpack_state()) {
        return controller_interface::return_type::ERROR;
    }

    if (!update_foot_phase(time, period)) {
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
    // Initialize publisher - we'll now use the GaitPattern for all data
    auto gait_pub = get_node()->create_publisher<quadruped_msgs::msg::GaitPattern>(
      "/quadruped/gait/gait_pattern", 10);
    
    rt_gait_pub_ = std::make_unique<RTPublisher>(gait_pub);
    gait_msg_ = std::make_shared<quadruped_msgs::msg::GaitPattern>();

    // Initialize with all feet in stance with a starting message
    if (rt_gait_pub_ && rt_gait_pub_->trylock()) {
      auto& msg = rt_gait_pub_->msg_;
      // Set initial states to stance
      msg.foot1_state = static_cast<int32_t>(-2);  // Changed to -2 (preinit state)
      msg.foot2_state = static_cast<int32_t>(-2);  // Changed to -2 (preinit state)
      msg.foot3_state = static_cast<int32_t>(-2);  // Changed to -2 (preinit state)
      msg.foot4_state = static_cast<int32_t>(-2);  // Changed to -2 (preinit state)
      // Initialize other fields with default values
      msg.foot1_phase = 0.0f;
      msg.foot2_phase = 0.0f;
      msg.foot3_phase = 0.0f;
      msg.foot4_phase = 0.0f;
      // Initialize step positions to prevent NaN values
      msg.foot1_step_position.x = 0.0;
      msg.foot1_step_position.y = 0.0;
      msg.foot1_step_position.z = 0.0;
      msg.foot2_step_position.x = 0.0;
      msg.foot2_step_position.y = 0.0;
      msg.foot2_step_position.z = 0.0;
      msg.foot3_step_position.x = 0.0;
      msg.foot3_step_position.y = 0.0;
      msg.foot3_step_position.z = 0.0;
      msg.foot4_step_position.x = 0.0;
      msg.foot4_step_position.y = 0.0;
      msg.foot4_step_position.z = 0.0;
      // Initialize COM position
      msg.com_position.x = 0.0;
      msg.com_position.y = 0.0;
      msg.com_position.z = 0.0;
      // Initialize step height
      msg.step_height = 0.05f;  // Default 5cm
      
      rt_gait_pub_->unlockAndPublish();
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
    // Load parameters from the parameter server
    auto node = get_node();
    
    // Default values if parameters are not set
    // Fix the type mismatch by using initializer list instead of direct assignment
    auto gait_type_param = node->declare_parameter<std::vector<int64_t>>("gait_type", {1, 3, 2, 4});
    gait_type_.clear();
    for (auto val : gait_type_param) {
      gait_type_.push_back(static_cast<int>(val));
    }

    stance_duration_ = node->declare_parameter<double>("stance_duration", 0.2);
    swing_duration_ = node->declare_parameter<double>("swing_duration", 0.2);
    duty_factor_ = node->declare_parameter<double>("duty_factor", 0.5);
    step_height_ = node->declare_parameter<double>("step_height", 0.05);
    step_length_ = node->declare_parameter<double>("step_length", 0.1);
    step_width_ = node->declare_parameter<double>("step_width", 0.1);

    // Use the gait type to determine the phase offsets
    // First, figure out how many steps are in the sequence
    int step_count_ = *std::max_element(gait_type_.begin(), gait_type_.end());
    // Calculate total cycle time based on stance and swing durations
    double total_cycle_time_ = (stance_duration_ + swing_duration_);
    // Evenly space the foot phases over the cycle

    // Add a declaration for current_time
    auto current_time = get_node()->get_clock()->now();

    for (int i = 0; i < 4; i++) {
      // Set up the phase offset per-foot
      foot_info_[i].phase_offset = static_cast<double>(gait_type_[i]-1) * (total_cycle_time_)/step_count_;
      // Start all feet in the -2 (preinit) state
      foot_info_[i].state = -2;
    }
    
    RCLCPP_INFO(node->get_logger(), "Loaded gait parameters:");
    RCLCPP_INFO(node->get_logger(), "  Gait type: [%d, %d, %d, %d]", 
                gait_type_[0], gait_type_[1], gait_type_[2], gait_type_[3]);
    RCLCPP_INFO(node->get_logger(), "  Stance duration: %f", stance_duration_);
    RCLCPP_INFO(node->get_logger(), "  Swing duration: %f", swing_duration_);
    RCLCPP_INFO(node->get_logger(), "  Duty factor: %f", duty_factor_);
    RCLCPP_INFO(node->get_logger(), "  Step height: %f", step_height_);
    RCLCPP_INFO(node->get_logger(), "  Step length: %f", step_length_);
    RCLCPP_INFO(node->get_logger(), "  Step width: %f", step_width_);
    RCLCPP_INFO(node->get_logger(), "  Total cycle time: %f", total_cycle_time_);
    
    // Set up state subscription
    state_sub_ = node->create_subscription<quadruped_msgs::msg::QuadrupedState>(
      "/quadruped/state/state_estimate",
      rclcpp::QoS(1).reliable(),
      [this](const quadruped_msgs::msg::QuadrupedState::SharedPtr msg) {
        latest_state_ = msg;
      }
    );

    // Fix the lambda capture to include the node
    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/quadruped/cmd/twist_cmd",
      rclcpp::QoS(1).reliable(),
      [this, node](const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_cmd_ = msg;
        RCLCPP_DEBUG(node->get_logger(), "Received teleop command: linear=(%f, %f, %f), angular=(%f, %f, %f)",
          msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
      }
    );

    // Add subscription for gait start command
    gait_start_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/quadruped/cmd/gait_start",
      rclcpp::QoS(1).reliable(),
      [this, node](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          gait_start_received_ = true;
          RCLCPP_INFO(node->get_logger(), "Received gait start command");
        }
      }
    );

    // Create regular publisher and realtime publisher
    auto gait_pub = node->create_publisher<quadruped_msgs::msg::GaitPattern>(
      "/quadruped/gait/gait_pattern", 10);
    
    // Create realtime publisher
    rt_gait_pub_ = std::make_unique<RTPublisher>(gait_pub);
    gait_msg_ = std::make_shared<quadruped_msgs::msg::GaitPattern>();

    RCLCPP_INFO(node->get_logger(), "Gait pattern generator configured successfully");
    RCLCPP_INFO(node->get_logger(), "Waiting for gait start command (SPACE key) to begin initialization sequence");
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
