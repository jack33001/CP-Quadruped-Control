#ifndef ZEROJOINTSCONTROLLER_HPP
#define ZEROJOINTSCONTROLLER_HPP


#pragma once


#include <vector>
#include <string>
#include <memory>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include "controller_interface/controller_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp" 


namespace quadruped_utils
{
class ZeroJointController : public controller_interface::ControllerInterface
{
public:
    ZeroJointController();
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:


    
    std::vector<std::string> joint_names_;
    std::vector<std::string> command_interface_types_;
    std::vector<std::string> state_interface_types_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_position_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_velocity_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_velocity_state_interface_;




    std::unordered_map<
        std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
        command_interface_map_ = {
        {"position", &joint_position_command_interface_},
        {"velocity", &joint_velocity_command_interface_}};

    std::unordered_map<
        std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
        state_interface_map_ = {
        {"position", &joint_position_state_interface_},
        {"velocity", &joint_velocity_state_interface_}};
        
    struct JointState {
        double position;
        double velocity;
        double effort;
        };

    std::vector<JointState> joint_states_;


    struct MotorHandle
    {
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
    };
    
    // const char * feedback_type() const;
    
    // controller_interface::CallbackReturn configure_side(
    //   const std::string & side, const std::vector<std::string> & wheel_names,
    //   std::vector<WheelHandle> & registered_handles);
    
    std::vector<MotorHandle> registered_motor_handles_;

private:
    // subscriptions
    // rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
    



    
};
} // namespace quadruped_utils

#endif // ZEROJOINTSCONTROLLER_HPP