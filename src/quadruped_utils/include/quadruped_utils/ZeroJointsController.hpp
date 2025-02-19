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

namespace quadruped_utils
{
class ZeroJointController : public controller_interface::ControllerInterface
    {
    public:

        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        ZeroJointController();
        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    protected:
        std::vector<std::string> joint_names_;

    private:
        std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
        std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;
    };
} // namespace quadruped_utils

#endif // ZEROJOINTSCONTROLLER_HPP