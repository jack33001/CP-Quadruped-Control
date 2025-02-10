#include "quadruped_mpc/controller_interfaces/state_estimator.hpp"
#include <std_msgs/msg/string.hpp>
#include <sstream>  // Add this header

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
  
  // Remove floating base interfaces - these don't exist in hardware
  
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
  if (!read_state_interfaces() 
      || !update_model()
      || !estimate_orientation()
      || !pin_kinematics()
      || !detect_contact()
      || !estimate_base_position()
      || !foot_positions()
      || !update_odometry()
      ) {
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
    
    // Set up sim clock before anything else
    get_node()->set_parameter(rclcpp::Parameter("use_sim_time", true));
    sim_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    clock_connected_ = false;
    
    // Setup clock subscription with reliable QoS
    clock_sub_ = get_node()->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", 
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
        // Simply mark as connected when we receive clock messages
        if (!clock_connected_) {
          RCLCPP_INFO(get_node()->get_logger(), "Connected to /clock topic");
          clock_connected_ = true;
        }
      }
    );

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
    // First explicitly set use_sim_time before anything else
    bool use_sim_time = false;
    if (!get_node()->get_parameter("use_sim_time", use_sim_time)) {
      RCLCPP_INFO(get_node()->get_logger(), "Setting use_sim_time parameter");
      get_node()->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }

    // Wait for clock without using spin_some
    rclcpp::Rate rate(10);  // 10 Hz check rate
    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !clock_connected_) {
      RCLCPP_INFO(get_node()->get_logger(), "Waiting for /clock...");
      
      // Check for timeout after 5 seconds
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() > 5) {
        RCLCPP_ERROR(get_node()->get_logger(), "Timeout waiting for clock connection");
        return CallbackReturn::ERROR;
      }
      
      rate.sleep();
    }

    // Set up clock subscription with reliable AND transient local QoS
    clock_sub_ = get_node()->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", 
      rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .transient_local(),
      [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
        if (!clock_connected_) {
          RCLCPP_INFO(get_node()->get_logger(), "Connected to /clock topic");
          clock_connected_ = true;
        }
      }
    );

    RCLCPP_INFO(get_node()->get_logger(), "Clock subscription confirmed");

    // Initialize tf broadcaster and odom publisher
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());
    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
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
      pinocchio::urdf::buildModelFromXML(urdf_string_, pinocchio::JointModelFreeFlyer(), model_);
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
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during configure stage with message: %s", e.what());
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

bool StateEstimator::read_state_interfaces()
{
  try {
    // Resize joint states vector if needed
    if (joint_states_.size() != joint_names_.size()) {
      joint_states_.resize(joint_names_.size());
    }

    auto& quadruped_info = SharedQuadrupedInfo::getInstance();
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
    
    // Store quaternion in Pinocchio's order [x,y,z,w]
    quadruped_info.state_.orientation_quat = Eigen::Vector4d(
      state_interfaces_[imu_start_idx].get_value(),      // x
      state_interfaces_[imu_start_idx + 1].get_value(),  // y
      state_interfaces_[imu_start_idx + 2].get_value(),  // z
      state_interfaces_[imu_start_idx + 3].get_value()   // w
    );

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
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error updating model: %s", e.what());
    return false;
  }
}

bool StateEstimator::foot_positions()
{
  try {
    auto& quadruped_info = SharedQuadrupedInfo::getInstance();
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);

    // Get foot positions in correct order: FL, FR, RL,
    quadruped_info.state_.p1 = data_->oMf[foot_frame_ids_[0]].translation() - quadruped_info.state_.pc;  // FL
    quadruped_info.state_.p2 = data_->oMf[foot_frame_ids_[1]].translation() - quadruped_info.state_.pc;  // FR
    quadruped_info.state_.p3 = data_->oMf[foot_frame_ids_[2]].translation() - quadruped_info.state_.pc;  // RL
    quadruped_info.state_.p4 = data_->oMf[foot_frame_ids_[3]].translation() - quadruped_info.state_.pc;  // RR

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
    auto& quadruped_info = SharedQuadrupedInfo::getInstance();
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

bool StateEstimator::pin_kinematics()
{
  try {
    auto& quadruped_info = SharedQuadrupedInfo::getInstance();
    pinocchio::forwardKinematics(model_, *data_, current_positions_, current_velocities_);
    pinocchio::updateFramePlacements(model_, *data_);
    
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);

    // Store foot positions as before
    quadruped_info.state_.p1 = data_->oMf[foot_frame_ids_[0]].translation();  // FL
    quadruped_info.state_.p2 = data_->oMf[foot_frame_ids_[1]].translation();  // FR
    quadruped_info.state_.p3 = data_->oMf[foot_frame_ids_[2]].translation();  // RL
    quadruped_info.state_.p4 = data_->oMf[foot_frame_ids_[3]].translation();  // RR

    // Compute full Jacobians in world frame
    Eigen::MatrixXd J_temp(6, model_.nv);

    // For each leg, get the correct joint indices from the model
    // FL
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_,
                                   foot_frame_ids_[0], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                   J_temp);
    // Extract just the position Jacobian (top 3 rows) for the leg's joints
    quadruped_info.state_.J1 = J_temp.block<3,2>(0, model_.joints[joint_mappings_[0].pinocchio_idx].idx_v());

    // FR
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_,
                                   foot_frame_ids_[1], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                   J_temp);
    quadruped_info.state_.J2 = J_temp.block<3,2>(0, model_.joints[joint_mappings_[2].pinocchio_idx].idx_v());

    // RL
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_,
                                   foot_frame_ids_[2], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                   J_temp);
    quadruped_info.state_.J3 = J_temp.block<3,2>(0, model_.joints[joint_mappings_[4].pinocchio_idx].idx_v());

    // RR
    pinocchio::computeFrameJacobian(model_, *data_, current_positions_,
                                   foot_frame_ids_[3], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                   J_temp);
    quadruped_info.state_.J4 = J_temp.block<3,2>(0, model_.joints[joint_mappings_[6].pinocchio_idx].idx_v());

    // Debug print to check Jacobian values
    RCLCPP_DEBUG(get_node()->get_logger(), 
                 "FL Jacobian:\n%.3f %.3f\n%.3f %.3f\n%.3f %.3f",
                 quadruped_info.state_.J1(0,0), quadruped_info.state_.J1(0,1),
                 quadruped_info.state_.J1(1,0), quadruped_info.state_.J1(1,1),
                 quadruped_info.state_.J1(2,0), quadruped_info.state_.J1(2,1));

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in pin kinematics: %s", e.what());
    return false;
  }
}

bool StateEstimator::estimate_base_position()
{
  try {
    auto& quadruped_info = SharedQuadrupedInfo::getInstance();
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);

    // Define foot sphere radius
    constexpr double foot_radius = 0.015;  // meters

    // Get foot positions and contact states
    std::vector<Eigen::Vector3d> foot_positions = {
      quadruped_info.state_.p1,  // FL
      quadruped_info.state_.p2,  // FR
      quadruped_info.state_.p3,  // RL
      quadruped_info.state_.p4   // RR
    };
    std::vector<bool> contacts = {
      quadruped_info.state_.contact_1_,
      quadruped_info.state_.contact_2_,
      quadruped_info.state_.contact_3_,
      quadruped_info.state_.contact_4_
    };

    // Calculate average Z height only, using the average of the contacting footp positions
    double world_z = 0.0;
    int contact_count = 0;

    for (size_t i = 0; i < 4; ++i) {
      if (contacts[i]) {
        world_z += foot_positions[i].z();
        contact_count++;
      }
    }

    if (contact_count > 0) {
      world_z /= contact_count;
      // Remember to negate the z-position to match the orientation of the world frame vs the body frame!
      world_z = world_z - foot_radius;  // Subtract foot radius to get body center


      // Set X and Y to 0, only update Z
      quadruped_info.state_.pc.x() = 0.0;
      quadruped_info.state_.pc.y() = 0.0;
      quadruped_info.state_.pc.z() = -world_z;
      
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Estimated body position: [%.3f, %.3f, %.3f] m",
        quadruped_info.state_.pc.x(),
        quadruped_info.state_.pc.y(),
        quadruped_info.state_.pc.z()
      );
    } else {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "No feet in contact with ground"
      );
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in base position estimation: %s", e.what());
    return false;
  }
}

bool StateEstimator::estimate_orientation() 
{
  try {
    auto& quadruped_info = SharedQuadrupedInfo::getInstance();
    std::lock_guard<std::mutex> lock(quadruped_info.mutex_);

    // Copy quaternion from Pinocchio state (already in [x,y,z,w] order)
    current_positions_[3] = quadruped_info.state_.orientation_quat[0];  // x
    current_positions_[4] = quadruped_info.state_.orientation_quat[1];  // y
    current_positions_[5] = quadruped_info.state_.orientation_quat[2];  // z
    current_positions_[6] = quadruped_info.state_.orientation_quat[3];  // w

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error in orientation estimation: %s", e.what());
    return false;
  }
}

bool StateEstimator::update_odometry()
{
  try {
    auto time_now = get_node()->get_clock()->now();
    
    geometry_msgs::msg::TransformStamped transform;
    nav_msgs::msg::Odometry odom;
    
    transform.header.stamp = time_now;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    
    odom.header = transform.header;
    odom.child_frame_id = transform.child_frame_id;

    Eigen::Vector3d pc;
    Eigen::Vector3d angular_velocity;
    
    {
      auto& quadruped_info = SharedQuadrupedInfo::getInstance();
      std::lock_guard<std::mutex> lock(quadruped_info.mutex_);
      pc = quadruped_info.state_.pc;
      angular_velocity = quadruped_info.state_.angular_velocity;
    }
    
    // Get the transform from base_link to Body from the model
    const auto& body_placement = model_.frames[body_frame_id_].placement;
    
    // Apply the body offset to our computed position
    Eigen::Vector3d base_position = pc - body_placement.translation();
    
    // Set translation with the base_link offset
    transform.transform.translation.x = current_positions_[0];
    transform.transform.translation.y = current_positions_[1];
    transform.transform.translation.z = current_positions_[2];
    
    // Set rotation (quaternion stays the same as it represents the orientation of the whole robot)
    transform.transform.rotation.w = current_positions_[3];
    transform.transform.rotation.x = current_positions_[4];
    transform.transform.rotation.y = current_positions_[5];
    transform.transform.rotation.z = current_positions_[6];

    // Log frame IDs
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "TF frames: parent='%s', child='%s'",
      transform.header.frame_id.c_str(),
      transform.child_frame_id.c_str()
    );

    // Copy transform to odometry message
    odom.pose.pose.position.x = transform.transform.translation.x;
    odom.pose.pose.position.y = transform.transform.translation.y;
    odom.pose.pose.position.z = transform.transform.translation.z;
    odom.pose.pose.orientation = transform.transform.rotation;

    // Set velocities
    odom.twist.twist.linear = geometry_msgs::msg::Vector3();  // Zero for now
    odom.twist.twist.angular.x = angular_velocity.x();
    odom.twist.twist.angular.y = angular_velocity.y();
    odom.twist.twist.angular.z = angular_velocity.z();

    // Simple debug logging
    RCLCPP_DEBUG(get_node()->get_logger(), 
      "Transform - Position: [%.3f, %.3f, %.3f], Rotation: [%.3f, %.3f, %.3f, %.3f]",
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z,
      transform.transform.rotation.w,
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z);

    // Publish everything
    tf_broadcaster_->sendTransform(transform);
    odom_pub_->publish(odom);

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error updating odometry: %s", e.what());
    return false;
  }
}

}  // namespace quadruped_mpc

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  quadruped_mpc::StateEstimator, controller_interface::ControllerInterface)
