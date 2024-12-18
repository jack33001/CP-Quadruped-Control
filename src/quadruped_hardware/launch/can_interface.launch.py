from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface to use (e.g., can0, can1, vcan0)'
    )

    # CAN Interface Node
    can_interface_node = Node(
        package='quadruped_hardware',
        executable='ros2_can_interface',  # Changed from 'can_interface' to 'ros2_can_interface'
        name='ros2_can_interface',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface')
        }],
        output='screen'
    )

    return LaunchDescription([
            can_interface_arg,
            can_interface_node
        ])