from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import xacro

def generate_launch_description():
    # Package and path definitions
    pkg_quadruped = FindPackageShare('quadruped_urdf').find('quadruped_urdf')
    xacro_file = os.path.join(pkg_quadruped, 'urdf', 'QuadrupedURDF_sim.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description
            }]
        )
    ])
