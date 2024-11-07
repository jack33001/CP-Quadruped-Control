#include "quadruped/QuadrupedHardwareInterface.hpp"
#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace quadruped
{

QuadrupedHardwareInterface::QuadrupedHardwareInterface()
{
    node_ = std::make_shared<rclcpp::Node>("quadruped_hardware_interface");
}

hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    RCLCPP_INFO(node_->get_logger(), "Initializing...");

    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize state and command interfaces
    for (const auto& joint : info_.joints)
    {
        // Resize the joint values vectors
        joint_position_.push_back(0.0);
        joint_velocity_.push_back(0.0);
        joint_effort_.push_back(0.0);
        joint_position_command_.push_back(0.0);
        joint_effort_command_.push_back(0.0);

        // Create state interfaces
        state_interfaces_.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &joint_position_.back()));
        state_interfaces_.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_.back()));
        state_interfaces_.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_EFFORT, &joint_effort_.back()));

        // Create command interfaces
        command_interfaces_.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_POSITION, &joint_position_command_.back()));
        command_interfaces_.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_EFFORT, &joint_effort_command_.back()));
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(node_->get_logger(), "Configuring...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(node_->get_logger(), "Activating...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(node_->get_logger(), "Deactivating...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> QuadrupedHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(node_->get_logger(), "Exporting state interfaces");
    return state_interfaces_;
}

std::vector<hardware_interface::CommandInterface> QuadrupedHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(node_->get_logger(), "Exporting command interfaces");
    return command_interfaces_;
}

hardware_interface::return_type QuadrupedHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    RCLCPP_INFO(node_->get_logger(), "Reading...");

    // Use EntityComponentManager to get the world and model
    gz::sim::EntityComponentManager ecm;
    auto world_entity = ecm.EntityByComponents(gz::sim::components::World(), gz::sim::components::Name("default"));

    if (!world_entity)
    {
        RCLCPP_ERROR(node_->get_logger(), "World not found in Gazebo");
        return hardware_interface::return_type::ERROR;
    }

    auto model_entity = ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name("your_model_name")); // Replace with your model's name
    if (!model_entity)
    {
        RCLCPP_ERROR(node_->get_logger(), "Model not found in Gazebo");
        return hardware_interface::return_type::ERROR;
    }

    // Loop through your joints and get their states
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        auto joint = model_entity.JointByName(info_.joints[i].name);
        if (joint)
        {
            // Read the joint position, velocity, and effort
            joint_position_[i] = joint.Position(0); // Assuming a single degree of freedom
            joint_velocity_[i] = joint.Velocity(0);
            joint_effort_[i] = joint.Force(0);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Joint %s not found in Gazebo", info_.joints[i].name.c_str());
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuadrupedHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    RCLCPP_INFO(node_->get_logger(), "Writing...");

    gz::sim::EntityComponentManager ecm;
    auto world_entity = ecm.EntityByComponents(gz::sim::components::World(), gz::sim::components::Name("default"));

    if (!world_entity)
    {
        RCLCPP_ERROR(node_->get_logger(), "World not found in Gazebo");
        return hardware_interface::return_type::ERROR;
    }

    auto model_entity = ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name("your_model_name")); // Replace with your model's name
    if (!model_entity)
    {
        RCLCPP_ERROR(node_->get_logger(), "Model not found in Gazebo");
        return hardware_interface::return_type::ERROR;
    }

    // Loop through your joints and apply commands
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        auto joint = model_entity.JointByName(info_.joints[i].name);
        if (joint)
        {
            // Send command to the joint, e.g., to position or effort
            joint.SetPosition(0, joint_position_command_[i]);
            joint.SetForce(0, joint_effort_command_[i]);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Joint %s not found in Gazebo", info_.joints[i].name.c_str());
        }
    }

    return hardware_interface::return_type::OK;
}

QuadrupedHardwareInterface::~QuadrupedHardwareInterface() {
    RCLCPP_INFO(node_->get_logger(), "Destroying QuadrupedHardwareInterface");
}

}  // namespace quadruped

PLUGINLIB_EXPORT_CLASS(
  quadruped::QuadrupedHardwareInterface,
  hardware_interface::SystemInterface
)
