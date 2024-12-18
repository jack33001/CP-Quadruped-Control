from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro

def generate_launch_description():
    # Package and path definitions
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')

    # Spawn robot node
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot_urdf',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '1.0', 
                   '-entity', 'Edward'],
        parameters=[{'use_sim_time': True, 'update_rate': 1000}]
    )

    return LaunchDescription([
        # Include Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'paused': 'true',
                'use_sim_time': 'true',
                'gui': 'true',
                'recording': 'false',
                'debug': 'false',
                'verbose': 'true',
                'gz_args': '-r empty.sdf',
            }.items()
        ),
        # Add robot spawning node
        spawn_robot,
    ])
