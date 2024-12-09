#include "quadruped_mpc/controller_interfaces/state_estimator.hpp"
#include <std_msgs/msg/string.hpp>

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
  
  // Add state interfaces for each joint
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
  // Get current joint positions and velocities with proper sizes
  Eigen::VectorXd q(model_.nq);
  Eigen::VectorXd v(model_.nv);
  q.setZero();
  v.setZero();

  // Set floating base configuration (first 7 DoFs)
  q.head<7>().setZero();  // [x,y,z, quat_w,quat_x,quat_y,quat_z]

  // Update base orientation from IMU
  Eigen::Quaterniond base_orientation(
    state_interfaces_[3].get_value(),  // w
    state_interfaces_[0].get_value(),  // x
    state_interfaces_[1].get_value(),  // y
    state_interfaces_[2].get_value()   // z
  );
  
  // Store quaternion directly in configuration vector
  q[3] = base_orientation.w();
  q[4] = base_orientation.x();
  q[5] = base_orientation.y();
  q[6] = base_orientation.z();
  
  Eigen::Vector3d angular_velocity(
    state_interfaces_[4].get_value(),
    state_interfaces_[5].get_value(),
    state_interfaces_[6].get_value()
  );

  // Update joint states
  const size_t num_joints = joint_names_.size();
  const size_t imu_interfaces = 10;  // Total number of IMU interfaces
  const size_t interfaces_per_joint = state_interface_types_.size();

  for (size_t i = 0; i < num_joints; ++i) {
    size_t q_idx = 7 + i;  // Skip floating base (7 DoFs)
    size_t v_idx = i;      // Velocity starts at 0 for joints
    
    if (q_idx < static_cast<size_t>(model_.nq) && v_idx < static_cast<size_t>(model_.nv)) {
      size_t interface_idx = imu_interfaces + i * interfaces_per_joint;
      
      if (interface_idx < state_interfaces_.size()) {
        q[q_idx] = state_interfaces_[interface_idx].get_value();         // Position
        if (interface_idx + 1 < state_interfaces_.size()) {
          v[v_idx] = state_interfaces_[interface_idx + 1].get_value();   // Velocity
        }
      }
    }
  }

  try {
    // Update kinematics
    pinocchio::forwardKinematics(model_, *data_, q, v);
    pinocchio::updateFramePlacements(model_, *data_);

    // Update shared state with thread safety
    {
      auto& info = SharedQuadrupedInfo::getInstance();
      std::lock_guard<std::mutex> lock(info.mutex_);
      
      // Update base state
      info.state_.th_c_ = base_orientation;
      info.state_.om_c_ = angular_velocity;
      
      // Update foot positions
      for (size_t i = 0; i < 4; ++i) {
        if (foot_frame_ids_[i] < model_.frames.size()) {
          Eigen::Vector3d& foot_pos = (i == 0) ? info.state_.p_1_ :
                                     (i == 1) ? info.state_.p_2_ :
                                     (i == 2) ? info.state_.p_3_ : info.state_.p_4_;
          foot_pos = data_->oMf[foot_frame_ids_[i]].translation();
        }
      }
      info.is_initialized_ = true;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in kinematics update: %s", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void StateEstimator::robot_description_callback(const std_msgs::msg::String::SharedPtr msg)
{
  urdf_string_ = msg->data;
  urdf_received_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Received robot description from topic");
}

StateEstimator::CallbackReturn 
StateEstimator::on_init()
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

StateEstimator::CallbackReturn 
StateEstimator::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
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

  // Add timeout for robot description
  rclcpp::Time start_time = get_node()->now();
  while (!urdf_received_ && (get_node()->now() - start_time).seconds() < 5.0) {
    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,  // ms
      "Waiting for robot description...");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  if (!urdf_received_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot description after 5 seconds");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Got robot_description, building model...");

  try {
    // Create Pinocchio model from URDF
    pinocchio::urdf::buildModelFromXML(urdf_string_, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);

    // Get foot frame IDs
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
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to build model: %s", e.what());
    return CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

StateEstimator::CallbackReturn 
StateEstimator::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

StateEstimator::CallbackReturn 
StateEstimator::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

}  // namespace quadruped_mpc

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  quadruped_mpc::StateEstimator, controller_interface::ControllerInterface)
