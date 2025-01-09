from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': '<robot_description>',
            'robot_description_semantic': '<robot_description_semantic>',
            'robot_description_kinematics': '<robot_description_kinematics>',
            'robot_description_planning': '<robot_description_planning>',
            'robot_description_transmissions': '<robot_description_transmissions>',
            'robot_description_controllers': '<robot_description_controllers>',
            'hardware_plugin': 'quadruped_hardware/QuadrupedHardwareInterface'
        }],
        output='screen'
    )

    return LaunchDescription([
        controller_manager_node
    ])