#include "quadruped_mpc/controller_interfaces/state_estimator.hpp"
#include "quadruped_mpc/control_laws/StateEstimation.hpp"
#include <std_msgs/msg/string.hpp>
#include <sstream>

namespace quadruped_mpc
{

StateEstimator::StateEstimator() : controller_interface::ControllerInterface() {}

controller_interface::InterfaceConfiguration 
StateEstimator::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration 
StateEstimator::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Add state interfaces for actuated joints
  for (const auto & joint : joint_names_) {
    for (const auto & interface : state_interface_types_) {
      config.names.push_back(joint + "/" + interface);
    }
  }
  
  // Add IMU interfaces
  config.names.push_back("imu_sensor/orientation.x");
  config.names.push_back("imu_sensor/orientation.y");
  config.names.push_back("imu_sensor/orientation.z");
  config.names.push_back("imu_sensor/orientation.w");
  config.names.push_back("imu_sensor/angular_velocity.x");
  config.names.push_back("imu_sensor/angular_velocity.y");
  config.names.push_back("imu_sensor/angular_velocity.z");
  config.names.push_back("imu_sensor/linear_acceleration.x");
  config.names.push_back("imu_sensor/linear_acceleration.y");
  config.names.push_back("imu_sensor/linear_acceleration.z");
  
  return config;
}

controller_interface::return_type
StateEstimator::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!read_state_interfaces()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read state interfaces");
    return controller_interface::return_type::ERROR;
  }

  if (!update_model()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to update model");
    return controller_interface::return_type::ERROR;
  }

  if (!estimate_orientation()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to estimate orientation");
    return controller_interface::return_type::ERROR;
  }

  if (!pin_kinematics()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to compute kinematics");
    return controller_interface::return_type::ERROR;
  }

  if (!detect_contact()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to detect contact");
    return controller_interface::return_type::ERROR;
  }

  if (!estimate_base_position()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to estimate base position");
    return controller_interface::return_type::ERROR;
  }

  if (!foot_positions()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to compute foot positions");
    return controller_interface::return_type::ERROR;
  }

  if (!update_odometry()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to update odometry");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void StateEstimator::robot_description_callback(const std_msgs::msg::String::SharedPtr msg)
{
  urdf_string_ = msg->data;
  urdf_received_ = true;
}

auto StateEstimator::on_init() -> CallbackReturn
{
  try {
    // Get parameters from yaml
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("state_interfaces", std::vector<std::string>());

    // Setup robot description subscription
    urdf_received_ = false;
    robot_description_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&StateEstimator::robot_description_callback, this, std::placeholders::_1)
    );

    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
}

auto StateEstimator::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn
{
  try {
    // Initialize publishers
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());
    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // Get parameters
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
      return CallbackReturn::ERROR;
    }

    state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();
    if (state_interface_types_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces specified");
      return CallbackReturn::ERROR;
    }

    // Wait for robot description
    rclcpp::Time start_time = get_node()->now();
    while (!urdf_received_ && (get_node()->now() - start_time).seconds() < 5.0) {
      RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Waiting for robot description...");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (!urdf_received_) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot description after 5 seconds");
      return CallbackReturn::ERROR;
    }

    try {
      // Create Pinocchio model from URDF
      pinocchio::urdf::buildModelFromXML(urdf_string_, pinocchio::JointModelFreeFlyer(), model_);
      data_ = std::make_unique<pinocchio::Data>(model_);

      // Create mapping from state interface index to pinocchio joint index
      joint_mappings_.clear();
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::string pin_joint_name = joint_names_[i];
        
        // Find exact joint ID match
        size_t pin_idx = 0;
        bool found = false;
        for (size_t j = 0; j < model_.joints.size(); ++j) {
          if (model_.names[j] == pin_joint_name) {
            pin_idx = j;
            found = true;
            break;
          }
        }

        if (!found) {
          RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in model", pin_joint_name.c_str());
          return CallbackReturn::ERROR;
        }

        joint_mappings_.push_back({joint_names_[i], i, pin_idx});
      }

      // Pre-allocate vectors
      current_positions_.resize(model_.nq);
      current_velocities_.resize(model_.nv);
      current_positions_.setZero();
      current_velocities_.setZero();

      // Get frame IDs
      const std::array<std::string, 4> foot_frame_names = {
        "fl_foot", "fr_foot", "rl_foot", "rr_foot"
      };
      
      for (size_t i = 0; i < 4; ++i) {
        foot_frame_ids_[i] = model_.getFrameId(foot_frame_names[i]);
        if (foot_frame_ids_[i] >= static_cast<std::size_t>(model_.nframes)) {
          RCLCPP_ERROR(get_node()->get_logger(), "Frame %s not found in model", foot_frame_names[i].c_str());
          return CallbackReturn::ERROR;
        }
      }
      
      const std::array<std::string, 4> hip_frame_names = {
        "fl_hip", "fr_hip", "rl_hip", "rr_hip"
      };

      for (size_t i = 0; i < 4; ++i) {
        hip_frame_ids_[i] = model_.getFrameId(hip_frame_names[i]);
        if (hip_frame_ids_[i] >= static_cast<std::size_t>(model_.nframes)) {
          RCLCPP_ERROR(get_node()->get_logger(), "Hip frame %s not found in model", hip_frame_names[i].c_str());
          return CallbackReturn::ERROR;
        }
      }
      
      // Find body frame id
      body_frame_id_ = model_.getFrameId("body");
      if (body_frame_id_ >= static_cast<std::size_t>(model_.nframes)) {
        body_frame_id_ = model_.getFrameId("Body");
        if (body_frame_id_ >= static_cast<std::size_t>(model_.nframes)) {
          body_frame_id_ = model_.getFrameId("base_link");
          if (body_frame_id_ >= static_cast<std::size_t>(model_.nframes)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Body frame not found in model");
            return CallbackReturn::ERROR;
          }
        }
      }

      RCLCPP_INFO(get_node()->get_logger(), "State estimator configured successfully");

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to build model: %s", e.what());
      return CallbackReturn::ERROR;
    }

    // Set up subscriptions
    odom_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
      "/quadruped/state/ground_truth/odometry",
      rclcpp::QoS(1).reliable(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg;
      }
    );
    
    gait_sub_ = get_node()->create_subscription<quadruped_msgs::msg::GaitPattern>(
      "/quadruped/gait/gait_pattern", 
      rclcpp::SensorDataQoS(),
      std::bind(&StateEstimator::gaitPatternCallback, this, std::placeholders::_1)
    );
    
    // Create realtime state publisher
    auto state_pub = get_node()->create_publisher<quadruped_msgs::msg::QuadrupedState>(
      "/quadruped/state/state_estimate", 10);
    
    rt_state_pub_ = std::make_unique<RTPublisher>(state_pub);
    state_msg_ = std::make_shared<quadruped_msgs::msg::QuadrupedState>();

    return CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during configure: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

auto StateEstimator::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn
{
  return CallbackReturn::SUCCESS;
}

auto StateEstimator::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn
{
  return CallbackReturn::SUCCESS;
}

void StateEstimator::gaitPatternCallback(const quadruped_msgs::msg::GaitPattern::SharedPtr msg)
{
  gait_pattern_ = *msg;
}

}  // namespace quadruped_mpc

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  quadruped_mpc::StateEstimator, controller_interface::ControllerInterface)