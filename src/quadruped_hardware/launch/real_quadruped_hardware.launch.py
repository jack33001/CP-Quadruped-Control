#launch  2 can interfaces on can0 and can1
#launch  4 motor drivers  each on can0 and can1

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Get paths
    config_path = os.path.join(get_package_share_directory('quadruped_bringup'), 
                              'config', 'quadruped_hardware.yaml')

    # Declare config file
    config_file = DeclareLaunchArgument(
        'config_file',
        default_value=config_path,
        description='Path to config file'
    )
    
    return LaunchDescription([
        # Launch CAN interface nodes
        Node(
            package='quadruped_hardware',
            executable='can_interface_node',
            name='can_interface_node_0',
            parameters=[{'can_interface': 'can0'}]
        ),
        Node(
            package='quadruped_hardware',
            executable='can_interface_node',
            name='can_interface_node_1',
            parameters=[{'can_interface': 'can1'}]
        ),

        # Launch Motor Driver nodes
        Node(
            package='quadruped_hardware',
            executable='motor_driver_node',
            name='motor_driver_node_0',
            parameters=[config_path, {
        'motor_ids': [
            LaunchConfiguration('motor0_can_id'),
            LaunchConfiguration('motor1_can_id'),
            LaunchConfiguration('motor2_can_id'),
            LaunchConfiguration('motor3_can_id')
        ],
        'can_interface': LaunchConfiguration('can_bus_0')
    }]),
        Node(
            package='quadruped_hardware',
            executable='motor_driver_node',
            name='motor_driver_node_1',
            parameters=[{'motor_ids': [LaunchConfiguration('motor4_can_id'),
                                       LaunchConfiguration('motor4_can_id'),
                                       LaunchConfiguration('motor4_can_id'),
                                       LaunchConfiguration('motor4_can_id')
                                       ], 
                         'can_interface': LaunchConfiguration('can_bus_1')}]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()