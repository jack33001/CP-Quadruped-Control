#include "quadruped_mpc/controller_interfaces/gait_pattern_generator.hpp"
#include "quadruped_mpc/control_laws/GaitPatternGenerator.hpp"
#include <chrono>

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
  // Start high-resolution timer for measuring update duration
  auto start_time = std::chrono::high_resolution_clock::now();

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

  // Calculate elapsed time with microsecond precision
  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  //RCLCPP_INFO(
  //  get_node()->get_logger(),
  //  "Gait pattern generator update took %.6f ms", 
  //  elapsed_time.count() / 1000.0);

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

    // Initialize prediction data
    prediction_stages_ = node->declare_parameter<int>("prediction_stages", 50);
    prediction_horizon_ = node->declare_parameter<double>("prediction_horizon", 5.0);
    
    // Initialize prediction arrays
    for (int foot = 0; foot < 4; foot++) {
      prediction_data_.states[foot].resize(prediction_stages_, -2);  // Default to pre-init state
      prediction_data_.phases[foot].resize(prediction_stages_, 0.0); // Default phase 0.0
    }
    prediction_data_.timestep = prediction_horizon_ / prediction_stages_;
    
    // Calculate total cycle time (needed for logging)
    double total_cycle_time = stance_duration_ + swing_duration_;
    
    // Use the gait type to determine the phase offsets
    // First, figure out how many steps are in the sequence
    int step_count_ = *std::max_element(gait_type_.begin(), gait_type_.end());
    // Evenly space the foot phases over the cycle

    // Add a declaration for current_time
    auto current_time = get_node()->get_clock()->now();

    for (int i = 0; i < 4; i++) {
      // Set up the phase offset per-foot
      foot_info_[i].phase_offset = static_cast<double>(gait_type_[i]-1) * (total_cycle_time)/step_count_;
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
    RCLCPP_INFO(node->get_logger(), "  Total cycle time: %f", total_cycle_time);
    RCLCPP_INFO(node->get_logger(), "  Prediction stages: %d", prediction_stages_);
    RCLCPP_INFO(node->get_logger(), "  Prediction horizon: %f", prediction_horizon_);
    RCLCPP_INFO(node->get_logger(), "  Prediction timestep: %f", prediction_data_.timestep);
    
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
          gait_start_time_ = node->get_clock()->now().seconds();  // Store the current time
          RCLCPP_INFO(node->get_logger(), "Received gait start command at time: %f", gait_start_time_);
        }
      }
    );

    // Create regular publisher and realtime publisher
    auto gait_pub = node->create_publisher<quadruped_msgs::msg::GaitPattern>(
      "/quadruped/gait/gait_pattern", 10);
    
    // Create realtime publisher
    rt_gait_pub_ = std::make_unique<RTPublisher>(gait_pub);
    gait_msg_ = std::make_shared<quadruped_msgs::msg::GaitPattern>();

    // Initialize gait pattern message with prediction arrays
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
      
      // Initialize prediction arrays
      std::vector<int32_t> predicted_states;
      std::vector<float> predicted_phases;
      
      for (int foot = 0; foot < 4; foot++) {
        for (int step = 0; step < prediction_stages_; step++) {
          predicted_states.push_back(-2);  // Default to pre-init state
          predicted_phases.push_back(0.0f);  // Default phase 0.0
        }
      }
      
      msg.predicted_states = predicted_states;
      msg.predicted_phases = predicted_phases;
      msg.prediction_stages = prediction_stages_;
      msg.prediction_horizon = static_cast<float>(prediction_horizon_);
      msg.prediction_timestep = static_cast<float>(prediction_data_.timestep);
      
      rt_gait_pub_->unlockAndPublish();
    }
    
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
