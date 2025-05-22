from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    robot_description_path = '/home/ws/src/quadruped_urdf/urdf/QuadrupedURDF_real.urdf.xacro'  # Path to your URDF file
    yaml_path = '/home/ws/src/quadruped_utils/config/controller.yaml'

    # Path to your controller YAML file

    config_path = os.path.join(
        get_package_share_directory('quadruped_utils'), 'config', 'controller.yaml')



    return LaunchDescription([
        
        # Debugger use prefix
           Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            # prefix=['gdbserver localhost:3001'],
            # namespace= "quadruped_hardware",
            parameters=[
                yaml_path , # Load yaml
            ],
            arguments=["controller_manager"],

        )
   


    ])



