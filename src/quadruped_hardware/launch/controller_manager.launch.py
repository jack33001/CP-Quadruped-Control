from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():


    # Define the controller manager node

    # controller_manager_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[{
    #         'robot_description': '<robot_description>',
    #         'robot_description_semantic': '<robot_description_semantic>',
    #         'robot_description_kinematics': '<robot_description_kinematics>',
    #         'robot_description_planning': '<robot_description_planning>',
    #         'robot_description_transmissions': '<robot_description_transmissions>',
    #         'robot_description_controllers': '<robot_description_controllers>',
    #         'hardware_plugin': 'src/quadruped_hardware/src/ROS2_CAN_Interface.cpp'
    #     }],
    #     output='screen'
    # )

    # # Include the hardware interface launch file
    # hardware_interface_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('quadruped_hardware'), 'launch', 'real_quadruped_hardware.launch.py')])
    # )

    robot_description_path = '/home/ws/src/quadruped_urdf/urdf/QuadrupedURDF_real.urdf.xacro'  # Path to your URDF file

    # Path to your controller YAML file


    quadruped_hardware_config_path = os.path.join(
        get_package_share_directory('quadruped_hardware'), 'config', 'quadruped_hardware.yaml')



    return LaunchDescription([

        # Start the controller manager (ros2_control_node)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[
                quadruped_hardware_config_path  # Load URDF
   
            ],


            # control_node = Node(
            # package="controller_manager",
            # executable="ros2_control_node",
            # parameters=[robot_controllers],
            # output="both",

        )



    ])



