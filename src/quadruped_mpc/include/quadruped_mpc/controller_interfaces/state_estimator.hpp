#ifndef QUADRUPED_MPC_STATE_ESTIMATOR_HPP_
#define QUADRUPED_MPC_STATE_ESTIMATOR_HPP_

#include <string>
#include <vector>
#include <memory>
#include <array>
#include <map>

// ROS2 headers
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "quadruped_msgs/msg/gait_pattern.hpp"

// Pinocchio headers
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

// TF2 includes
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "quadruped_msgs/msg/quadruped_state.hpp"

// Realtime publisher includes
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace quadruped_mpc
{

class StateEstimator : public controller_interface::ControllerInterface
{
public:
  using RTPublisher = realtime_tools::RealtimePublisher<quadruped_msgs::msg::QuadrupedState>;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  StateEstimator();

  ~StateEstimator()
  {
    data_.reset();
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> state_interface_types_;
  
  // IMU data storage
  Eigen::Vector4d imu_orientation_;  // [x,y,z,w]
  Eigen::Vector3d imu_angular_velocity_;
  Eigen::Vector3d imu_linear_acceleration_;
  
  // Foot state storage
  struct FootState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    bool in_contact;
  };
  std::array<FootState, 4> foot_states_;  // FL, FR, RL, RR
  
  // Hip positions storage
  std::array<Eigen::Vector3d, 4> hip_positions_;  // FL, FR, RL, RR

  // Position differencing variables for velocity calculation
  std::vector<Eigen::Vector3d> prev_foot_positions_;
  rclcpp::Time prev_update_time_{0};

  // Jacobian storage
  using JacobianMatrix = Eigen::Matrix<double, 3, 2>;
  struct LegJacobians {
    JacobianMatrix J1, J2, J3, J4;  // FL, FR, RL, RR
  };
  LegJacobians leg_jacobians_;

  // Pinocchio model and data
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  
  // Frame IDs for feet and body
  std::array<pinocchio::FrameIndex, 4> foot_frame_ids_;
  std::array<pinocchio::FrameIndex, 4> hip_frame_ids_;
  pinocchio::FrameIndex body_frame_id_;
  
  // Robot description subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  std::string urdf_string_;
  bool urdf_received_{false};
  
  // Callback for robot description
  void robot_description_callback(const std_msgs::msg::String::SharedPtr msg);
  
  // Debug vectors
  Eigen::VectorXd current_positions_;
  Eigen::VectorXd current_velocities_;

  struct JointMapping {
    std::string name;
    size_t state_interface_idx;
    size_t pinocchio_idx;
  };
  std::vector<JointMapping> joint_mappings_;

  // Joint state storage
  struct JointState {
    double position;
    double velocity;
    double effort;
  };
  std::vector<JointState> joint_states_;

  bool setup_frames();

  // State estimation functions
  bool read_state_interfaces();
  bool update_model();
  bool foot_positions();
  bool detect_contact();
  bool pin_kinematics();
  bool estimate_base_position();
  bool estimate_orientation();
  bool update_odometry();

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  bool clock_connected_{false};
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Clock::SharedPtr sim_clock_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  
  rclcpp::Subscription<quadruped_msgs::msg::GaitPattern>::SharedPtr gait_sub_;
  quadruped_msgs::msg::GaitPattern gait_pattern_;
  void gaitPatternCallback(const quadruped_msgs::msg::GaitPattern::SharedPtr msg);
  
  std::unique_ptr<RTPublisher> rt_state_pub_;
  std::shared_ptr<quadruped_msgs::msg::QuadrupedState> state_msg_;
};

}  // namespace quadruped_mpc

#endif  // QUADRUPED_MPC_STATE_ESTIMATOR_HPP_
