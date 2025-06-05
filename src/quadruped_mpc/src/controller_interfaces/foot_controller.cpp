#include "quadruped_mpc/controller_interfaces/foot_controller.hpp"
#include "quadruped_mpc/control_laws/FootControl.hpp"

#include <algorithm>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <sstream>
#include <chrono>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace quadruped_mpc
{

  FootController::FootController()
      : controller_interface::ControllerInterface()
  {
    // Initialize foot forces
    foot_forces_.has_data = false;
    
    // Initialize previous foot states
    previous_foot_states_.foot1_state = 0;
    previous_foot_states_.foot2_state = 0;
    previous_foot_states_.foot3_state = 0;
    previous_foot_states_.foot4_state = 0;
    previous_foot_states_.initialized = false;
  }

  controller_interface::CallbackReturn FootController::on_init()
  {
    try
    {
      // Get parameters from node
      auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
      auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>());

      controller_name_ = get_node()->get_name();

      return controller_interface::CallbackReturn::SUCCESS;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(
          get_node()->get_logger(), "Exception thrown during on_init stage with message: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  controller_interface::CallbackReturn FootController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    num_joints_ = joint_names_.size();

    if (num_joints_ == 0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "No joints provided");
      return controller_interface::CallbackReturn::ERROR;
    }

    command_interface_types_ = get_node()->get_parameter("command_interfaces").as_string_array();
    if (command_interface_types_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "No command interfaces provided");
      return controller_interface::CallbackReturn::ERROR;
    }

    // Initialize the realtime buffers
    foot_forces_buffer_ = std::make_shared<realtime_tools::RealtimeBuffer<quadruped_msgs::msg::FootForces>>();
    state_buffer_ = std::make_shared<realtime_tools::RealtimeBuffer<quadruped_msgs::msg::QuadrupedState>>();
    gait_buffer_ = std::make_shared<realtime_tools::RealtimeBuffer<quadruped_msgs::msg::GaitPattern>>();

    RCLCPP_INFO(get_node()->get_logger(), "Configured FootController with %zu joints", num_joints_);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration FootController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &joint : joint_names_)
    {
      for (const auto &interface_type : command_interface_types_)
      {
        command_interfaces_config.names.push_back(joint + "/" + interface_type);
      }
    }

    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration FootController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn FootController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Resize vectors first (clear and reserve instead of resize since we can't default construct these interfaces)
    joint_command_interfaces_.clear();

    // Reserve space to avoid reallocations
    joint_command_interfaces_.reserve(num_joints_ * command_interface_types_.size());

    // Log the command interfaces for debugging
    RCLCPP_INFO(get_node()->get_logger(), "Command interfaces being activated:");
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      RCLCPP_INFO(get_node()->get_logger(), "  [%zu] %s", i, command_interfaces_[i].get_name().c_str());
    }

    // Initialize interfaces by moving them into our vectors
    for (size_t i = 0; i < num_joints_; ++i)
    {
      for (size_t j = 0; j < command_interface_types_.size(); ++j)
      {
        joint_command_interfaces_.push_back(std::move(command_interfaces_[i * command_interface_types_.size() + j]));
      }
    }

    // Log the joint command interfaces after setup with more detail
    RCLCPP_INFO(get_node()->get_logger(), "Joint command interfaces after setup (this is the order that will be used):");
    for (size_t i = 0; i < joint_command_interfaces_.size(); ++i)
    {
      // Extract the joint name from the interface name (format is "joint_name/interface_type")
      std::string full_name = joint_command_interfaces_[i].get_name();
      size_t separator_pos = full_name.find('/');
      std::string joint_name = full_name.substr(0, separator_pos);
      
      RCLCPP_INFO(get_node()->get_logger(), "  [%zu] %s (joint: %s)", 
                  i, full_name.c_str(), joint_name.c_str());
    }

    // Parse the interface names to determine which interfaces correspond to which leg
    std::map<std::string, std::vector<size_t>> leg_to_interfaces;
    std::vector<std::string> leg_prefixes = {"fl_", "fr_", "rl_", "rr_"};
    
    for (size_t i = 0; i < joint_command_interfaces_.size(); ++i)
    {
      std::string name = joint_command_interfaces_[i].get_name();
      for (const auto& prefix : leg_prefixes)
      {
        if (name.find(prefix) != std::string::npos)
        {
          leg_to_interfaces[prefix].push_back(i);
          break;
        }
      }
    }
    
    // Log which interfaces belong to which leg
    RCLCPP_INFO(get_node()->get_logger(), "Interface to leg mapping:");
    for (const auto& leg : leg_prefixes)
    {
      std::stringstream ss;
      ss << leg << ": ";
      for (size_t idx : leg_to_interfaces[leg])
      {
        ss << idx << " ";
      }
      RCLCPP_INFO(get_node()->get_logger(), "  %s", ss.str().c_str());
    }

    // Setup subscriber for foot forces
    foot_forces_subscription_ = get_node()->create_subscription<quadruped_msgs::msg::FootForces>(
        "/quadruped/commands/forces", rclcpp::SensorDataQoS(),
        [this](const quadruped_msgs::msg::FootForces::SharedPtr msg) -> void
        {
          this->footForcesCallback(msg);
        });

    // Setup subscriber for state estimation
    state_subscription_ = get_node()->create_subscription<quadruped_msgs::msg::QuadrupedState>(
        "/quadruped/state/state_estimate", rclcpp::SensorDataQoS(),
        [this](const quadruped_msgs::msg::QuadrupedState::SharedPtr msg) -> void
        {
          this->stateCallback(msg);
        });
    RCLCPP_INFO(get_node()->get_logger(), "Subscribed to state estimation topic");

    // Setup subscriber for gait pattern
    gait_subscription_ = get_node()->create_subscription<quadruped_msgs::msg::GaitPattern>(
        "/quadruped/gait/gait_pattern", rclcpp::SensorDataQoS(),
        [this](const quadruped_msgs::msg::GaitPattern::SharedPtr msg) -> void
        {
          this->gaitCallback(msg);
        });
    RCLCPP_INFO(get_node()->get_logger(), "Subscribed to gait pattern topic");

    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "FootController activated successfully");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn FootController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Unsubscribe from topics
    foot_forces_subscription_.reset();
    state_subscription_.reset();
    gait_subscription_.reset();
    
    // Clear interfaces
    joint_command_interfaces_.clear();

    RCLCPP_INFO(get_node()->get_logger(), "FootController deactivated successfully");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type FootController::update(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    // Start high-resolution timer for measuring update duration
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!read_topics(*this))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to read topics");
      return controller_interface::return_type::ERROR;
    }

    if (!swing_trajectory(*this, time))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to generate swing trajectory");
      return controller_interface::return_type::ERROR;
    }

    if (!determine_forces(*this))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to determine forces");
      return controller_interface::return_type::ERROR;
    }
    
    if (!apply_jacobians(*this))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to apply Jacobians");
      return controller_interface::return_type::ERROR;
    }

      // Calculate elapsed time with microsecond precision
      auto end_time = std::chrono::high_resolution_clock::now();
      auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Foot controller update took %.6f ms", 
        elapsed_time.count() / 1000.0);

    // If we got here, everything was successful
    return controller_interface::return_type::OK;
  }

  void FootController::footForcesCallback(const quadruped_msgs::msg::FootForces::SharedPtr msg)
  {
    foot_forces_buffer_->writeFromNonRT(*msg);
  }

  void FootController::stateCallback(const quadruped_msgs::msg::QuadrupedState::SharedPtr msg)
  {
    state_buffer_->writeFromNonRT(*msg);
  }

  void FootController::gaitCallback(const quadruped_msgs::msg::GaitPattern::SharedPtr msg)
  {
    gait_buffer_->writeFromNonRT(*msg);
  }

  bool FootController::reset()
  {
    // Reset foot forces commands to all joints
    foot_forces_.has_data = false;
    
    // Reset previous foot states
    previous_foot_states_.foot1_state = 0;
    previous_foot_states_.foot2_state = 0;
    previous_foot_states_.foot3_state = 0;
    previous_foot_states_.foot4_state = 0;
    previous_foot_states_.initialized = false;
    
    // Send zero commands to all joints
    for (auto &interface : joint_command_interfaces_)
    {
      // Check the return value to avoid the warning
      if (!interface.set_value(0.0))
      {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set command interface value during reset");
        return false;
      }
    }
    
    return true;
  }

} // namespace quadruped_mpc

// Register the controller as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_mpc::FootController, controller_interface::ControllerInterface)
