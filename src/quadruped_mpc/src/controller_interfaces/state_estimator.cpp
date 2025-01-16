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
  if (!read_state_interfaces() || !update_model() || !forward_kinematics() || !detect_contact()) {
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
    // Removed hardware_height parameter
    
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

  try {
    // Create Pinocchio model from URDF
    pinocchio::urdf::buildModelFromXML(urdf_string_, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);

    // Initialize shared info
    auto& info = SharedQuadrupedInfo::getInstance();
    {
      std::lock_guard<std::mutex> lock(info.mutex_);
      info.default_model_ = model_;  // Copy the model
      info.is_initialized_ = true;
      RCLCPP_INFO(get_node()->get_logger(), "SharedQuadrupedInfo initialized");
    }

    // Create mapping from state interface index to pinocchio joint index
    joint_mappings_.clear();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      // Don't append "_joint" - use joint name directly
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

    // Pre-allocate vectors to correct sizes
    current_positions_.resize(model_.nq);
    current_velocities_.resize(model_.nv);
    current_positions_.setZero();
    current_velocities_.setZero();

    // Get foot frame IDs
    const std::array<std::string, 6> foot_frame_names = {
      "fl_foot", "fr_foot", "rl_foot", "rr_foot"
    };
    
    for (size_t i = 0; i < 4; ++i) {
      foot_frame_ids_[i] = model_.getFrameId(foot_frame_names[i]);
      if (foot_frame_ids_[i] >= static_cast<std::size_t>(model_.nframes)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Frame %s not found in model", foot_frame_names[i].c_str());
        return CallbackReturn::ERROR;
      }
    }

    // Get body frame ID
    body_frame_id_ = model_.getFrameId("Body");
    if (body_frame_id_ >= static_cast<std::size_t>(model_.nframes)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Frame 'Body' not found in model");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "State estimator configured successfully");

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to build model: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

auto StateEstimator::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn
{
  return CallbackReturn::SUCCESS;
}

auto StateEstimator::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn
{
  return CallbackReturn::SUCCESS;
}

bool StateEstimator::read_state_interfaces()
{
  try {
    // Resize joint states vector if needed
    if (joint_states_.size() != joint_names_.size()) {
      joint_states_.resize(joint_names_.size());
    }

    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);

    // Read all interfaces for each joint
    const size_t interfaces_per_joint = state_interface_types_.size();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      size_t base_idx = i * interfaces_per_joint;
      
      // Update local storage
      joint_states_[i].position = state_interfaces_[base_idx].get_value();      // position
      joint_states_[i].velocity = state_interfaces_[base_idx + 1].get_value();  // velocity
      joint_states_[i].effort = state_interfaces_[base_idx + 2].get_value();    // effort

      // Update shared state arrays
      quadruped_info.state_.joint_pos[i] = joint_states_[i].position;
      quadruped_info.state_.joint_vel[i] = joint_states_[i].velocity;
      quadruped_info.state_.joint_eff[i] = joint_states_[i].effort;
    }

    // Read IMU data - assuming they're after all joint interfaces
    const size_t imu_start_idx = joint_names_.size() * interfaces_per_joint;
    
    // Read orientation quaternion (convert to euler angles)
    double qx = state_interfaces_[imu_start_idx].get_value();
    double qy = state_interfaces_[imu_start_idx + 1].get_value();
    double qz = state_interfaces_[imu_start_idx + 2].get_value();
    double qw = state_interfaces_[imu_start_idx + 3].get_value();
    
    // Convert quaternion to euler angles (roll, pitch, yaw)
    double roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    double pitch = std::asin(2.0 * (qw * qy - qz * qx));
    double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    
    // Store euler angles
    quadruped_info.state_.orientation = Eigen::Vector3d(roll, pitch, yaw);

    // Read and store angular velocities directly
    quadruped_info.state_.angular_velocity = Eigen::Vector3d(
      state_interfaces_[imu_start_idx + 4].get_value(),  // wx
      state_interfaces_[imu_start_idx + 5].get_value(),  // wy
      state_interfaces_[imu_start_idx + 6].get_value()   // wz
    );

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error reading state interfaces: %s", e.what());
    return false;
  }
}

bool StateEstimator::update_model()
{
  try {
    // Clear position and velocity vectors
    current_positions_.setZero();
    current_velocities_.setZero();

    // Map joint states to Pinocchio model states
    for (size_t i = 0; i < joint_mappings_.size(); ++i) {
      const auto& mapping = joint_mappings_[i];
      const auto& joint = model_.joints[mapping.pinocchio_idx];
      
      // Get joint state
      const auto& state = joint_states_[i];

      // Map to Pinocchio state vectors
      if (joint.nq() == 2) {  // Revolute joints in Pinocchio use sin/cos
        current_positions_[joint.idx_q()] = std::cos(state.position);
        current_positions_[joint.idx_q() + 1] = std::sin(state.position);
      } else {
        current_positions_[joint.idx_q()] = state.position;
      }
      current_velocities_[joint.idx_v()] = state.velocity;
    }

    // Update Pinocchio model with new state
    pinocchio::forwardKinematics(model_, *data_, current_positions_, current_velocities_);
    pinocchio::updateFramePlacements(model_, *data_);
    
    // Compute Jacobians for each foot - getting only the translation part (3x2 matrices)
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);
    
    Eigen::MatrixXd J_temp(6,model_.nv);  // Temporary full Jacobian
    
    // For each foot, compute the full Jacobian then extract translation part
    // FL
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_, 
                                   foot_frame_ids_[0], pinocchio::LOCAL_WORLD_ALIGNED, 
                                   J_temp);
    quadruped_info.state_.J1 = J_temp.topRows(3).leftCols(2);  // Extract 3x2 translation part

    // FR
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_, 
                                   foot_frame_ids_[1], pinocchio::LOCAL_WORLD_ALIGNED, 
                                   J_temp);
    quadruped_info.state_.J2 = J_temp.topRows(3).leftCols(2);

    // RL
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_, 
                                   foot_frame_ids_[2], pinocchio::LOCAL_WORLD_ALIGNED, 
                                   J_temp);
    quadruped_info.state_.J3 = J_temp.topRows(3).leftCols(2);

    // RR
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_, 
                                   foot_frame_ids_[3], pinocchio::LOCAL_WORLD_ALIGNED, 
                                   J_temp);
    quadruped_info.state_.J4 = J_temp.topRows(3).leftCols(2);
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error updating model: %s", e.what());
    return false;
  }
}

bool StateEstimator::forward_kinematics()
{
  try {
    // Lock the shared data structure while updating
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);
    
    // Get body position
    quadruped_info.state_.pc = data_->oMf[body_frame_id_].translation();

    // Get foot positions in correct order: FL, FR, RL, RR
    quadruped_info.state_.p1 = data_->oMf[foot_frame_ids_[0]].translation();  // FL
    quadruped_info.state_.p2 = data_->oMf[foot_frame_ids_[1]].translation();  // FR
    quadruped_info.state_.p3 = data_->oMf[foot_frame_ids_[2]].translation();  // RL
    quadruped_info.state_.p4 = data_->oMf[foot_frame_ids_[3]].translation();  // RR

    // Copy joint states to shared info
    for (size_t i = 0; i < joint_states_.size(); ++i) {
      quadruped_info.state_.joint_pos[i] = joint_states_[i].position;
      quadruped_info.state_.joint_vel[i] = joint_states_[i].velocity;
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in inverse kinematics: %s", e.what());
    return false;
  }
}

bool StateEstimator::detect_contact()
{
  try {
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);
    
    // For now, just set all contacts to true
    quadruped_info.state_.contact_1_ = true;  // FL
    quadruped_info.state_.contact_2_ = true;  // FR
    quadruped_info.state_.contact_3_ = true;  // RL
    quadruped_info.state_.contact_4_ = true;  // RR

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in contact detection: %s", e.what());
    return false;
  }
}

}  // namespace quadruped_mpc

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  quadruped_mpc::StateEstimator, controller_interface::ControllerInterface)
