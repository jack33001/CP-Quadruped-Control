#include "quadruped_mpc/controller_interfaces/quadruped_broadcaster.hpp"

namespace quadruped_mpc
{

QuadrupedBroadcaster::QuadrupedBroadcaster()
: controller_interface::ControllerInterface()
{}

controller_interface::CallbackReturn QuadrupedBroadcaster::on_init()
{
  try {
    // Declare parameters
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::string>("imu_name", "");
    
    // Declare new parameters
    auto_declare<double>("gravity", 0.0);  // Default to 0 to ensure params are set
    auto_declare<double>("hardware_height", 0.0);
    auto_declare<double>("hardware_width", 0.0);
    auto_declare<double>("hardware_length", 0.0);
    auto_declare<double>("balance_m", 0.0);
    auto_declare<double>("balance_kp_pos", 0.0);
    auto_declare<double>("balance_kd_pos", 0.0);
    auto_declare<double>("balance_kp_rot", 0.0);
    auto_declare<double>("balance_kd_rot", 0.0);
    
    // Declare desired state parameters
    auto_declare<std::vector<double>>("p_c_des", {0.0, 0.0, 0.0});
    auto_declare<std::vector<double>>("v_c_des", {0.0, 0.0, 0.0});
    auto_declare<std::vector<double>>("th_c_des", {1.0, 0.0, 0.0, 0.0});  // Default to identity quaternion (w,x,y,z)
    auto_declare<std::vector<double>>("om_c_des", {0.0, 0.0, 0.0});
    
    // Declare new weight and constraint parameters
    auto_declare<std::vector<double>>("S", std::vector<double>(36, 0.0));  // 6x6 matrix flattened
    auto_declare<double>("alpha", 0.0);
    auto_declare<double>("beta", 0.0);
    auto_declare<std::vector<double>>("C_row_support", std::vector<double>(3, 0.0));
    auto_declare<std::vector<double>>("C_row_swing", std::vector<double>(3, 0.0));
    auto_declare<std::vector<double>>("d_row_support", std::vector<double>(3, 0.0));
    auto_declare<std::vector<double>>("d_row_swing", std::vector<double>(3, 0.0));
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  imu_name_ = get_node()->get_parameter("imu_name").as_string();
  
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints provided");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get parameters
  gravity_ = get_node()->get_parameter("gravity").as_double();
  hardware_height_ = get_node()->get_parameter("hardware_height").as_double();
  hardware_width_ = get_node()->get_parameter("hardware_width").as_double();
  hardware_length_ = get_node()->get_parameter("hardware_length").as_double();
  balance_m_ = get_node()->get_parameter("balance_m").as_double();
  balance_kp_pos_ = get_node()->get_parameter("balance_kp_pos").as_double();
  balance_kd_pos_ = get_node()->get_parameter("balance_kd_pos").as_double();
  balance_kp_rot_ = get_node()->get_parameter("balance_kp_rot").as_double();
  balance_kd_rot_ = get_node()->get_parameter("balance_kd_rot").as_double();
  
  // Get desired state parameters
  p_c_des_ = get_node()->get_parameter("p_c_des").as_double_array();
  v_c_des_ = get_node()->get_parameter("v_c_des").as_double_array();
  th_c_des_ = get_node()->get_parameter("th_c_des").as_double_array();
  om_c_des_ = get_node()->get_parameter("om_c_des").as_double_array();

  // Get new parameters
  std::vector<double> S_vec = get_node()->get_parameter("S").as_double_array();
  double alpha = get_node()->get_parameter("alpha").as_double();
  double beta = get_node()->get_parameter("beta").as_double();
  std::vector<double> C_row_support = get_node()->get_parameter("C_row_support").as_double_array();
  std::vector<double> C_row_swing = get_node()->get_parameter("C_row_swing").as_double_array();
  std::vector<double> d_row_support = get_node()->get_parameter("d_row_support").as_double_array();
  std::vector<double> d_row_swing = get_node()->get_parameter("d_row_swing").as_double_array();

  // Validate vector sizes
  if (p_c_des_.size() != 3 || v_c_des_.size() != 3 || 
      th_c_des_.size() != 4 || om_c_des_.size() != 3) {  // Changed to check for 4 quaternion components
    RCLCPP_ERROR(get_node()->get_logger(), 
      "Invalid vector sizes. p_c_des, v_c_des, om_c_des need 3 elements, th_c_des needs 4 elements (w,x,y,z)");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Validate sizes
  if (S_vec.size() != 36 || C_row_support.size() != 3 || C_row_swing.size() != 3 ||
      d_row_support.size() != 3 || d_row_swing.size() != 3) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid parameter vector sizes");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Pack shared_quadruped_info
  auto& info = quadruped_info;
  {
    std::lock_guard<std::mutex> lock(info.mutex_);
    
    // Hardware info
    info.hardware_.height = hardware_height_;
    info.hardware_.width = hardware_width_;
    info.hardware_.length = hardware_length_;
    
    // Balance control info
    info.balance_.g = gravity_;
    info.balance_.m = balance_m_;
    info.balance_.Kp_pos = balance_kp_pos_;
    info.balance_.Kd_pos = balance_kd_pos_;
    info.balance_.Kp_rot = balance_kp_rot_;
    info.balance_.Kd_rot = balance_kd_rot_;
    
    // Pack desired states
    info.balance_.p_c_des = Eigen::Vector3d(p_c_des_[0], p_c_des_[1], p_c_des_[2]);
    info.balance_.v_c_des = Eigen::Vector3d(v_c_des_[0], v_c_des_[1], v_c_des_[2]);
    info.balance_.th_b_des = Eigen::Quaterniond(th_c_des_[0], th_c_des_[1], th_c_des_[2], th_c_des_[3]);  // w,x,y,z
    info.balance_.om_b_des = Eigen::Vector3d(om_c_des_[0], om_c_des_[1], om_c_des_[2]);
    
    // Recalculate inertia matrix with new dimensions
    double width2 = std::pow(info.hardware_.width, 2);
    double height2 = std::pow(info.hardware_.height, 2);
    double length2 = std::pow(info.hardware_.length, 2);
    
    info.balance_.Ig << 
      (1.0/12.0) * balance_m_ * (height2 + length2), 0.0, 0.0,
      0.0, (1.0/12.0) * balance_m_ * (width2 + height2), 0.0,
      0.0, 0.0, (1.0/12.0) * balance_m_ * (width2 + length2);
    
    // Pack new parameters
    // Convert S vector to 6x6 matrix
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        info.balance_.S(i,j) = S_vec[i*6 + j];
      }
    }
    
    info.balance_.alpha = alpha;
    info.balance_.beta = beta;

    // Pack constraint matrices
    for (int i = 0; i < 3; i++) {
      info.balance_.C.block<1,1>(i,0) = C_row_support[i] * Eigen::Matrix<double,1,1>::Ones();
      info.balance_.C.block<1,1>(i,1) = C_row_swing[i] * Eigen::Matrix<double,1,1>::Ones();
      info.balance_.d(i,0) = d_row_support[i];
      info.balance_.d(i,1) = d_row_swing[i];
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type QuadrupedBroadcaster::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration QuadrupedBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration QuadrupedBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

} // namespace quadruped_mpc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  quadruped_mpc::QuadrupedBroadcaster,
  controller_interface::ControllerInterface)
