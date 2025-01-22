from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    robot_description_path = '/home/ws/src/quadruped_urdf/urdf/QuadrupedURDF_real.urdf.xacro'  # Path to your URDF file

    # Path to your controller YAML file

    config_path = os.path.join(
        get_package_share_directory('quadruped_hardware'), 'config', 'quadruped_hardware.yaml')



    return LaunchDescription([

        # Start the controller manager (ros2_control_node)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            # namespace= "quadruped_hardware",
            parameters=[
                config_path  # Load URDF
            ],

        )



    ])



