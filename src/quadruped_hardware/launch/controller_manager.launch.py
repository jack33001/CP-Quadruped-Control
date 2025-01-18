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

    # Include the hardware interface launch file
    hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('quadruped_hardware'), 'launch', 'real_quadruped_hardware.launch.py')])
    )


    return LaunchDescription([
        controller_manager_node
    ])