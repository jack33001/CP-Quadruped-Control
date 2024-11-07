from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():

    return LaunchDescription([
        # Delete the existing entity in Gazebo
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/srv/DeleteEntity', '{name: "Edward"}'],
            output='screen'
        ),

        # Reset the simulation
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/reset_simulation', 'std_srvs/srv/Empty'],
            output='screen'
        ),

        # Pause simulation (optional, but sometimes useful for respawn stability)
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/world/default/control', 'gazebo_msgs/srv/SetPaused', '{paused: true}'],
            output='screen'
        ),

        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_robot_urdf',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '1.0', '-entity', 'Edward'
            ],
            parameters=[{
                'use_sim_time': True,
                'update_rate': 1000
            }]
        ),

        # Unpause simulation (after spawning the robot)
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/world/default/control', 'gazebo_msgs/srv/SetPaused', '{paused: false}'],
            output='screen'
        ),

        # Start up a leg control node for each leg
        Node(
            package='quadruped',
            executable='LegController.py',
            output='screen',
            parameters=[{'which_leg': 'fl'}]
        ),
        Node(
            package='quadruped',
            executable='LegController.py',
            output='screen',
            parameters=[{'which_leg': 'fr'}]
        ),
        Node(
            package='quadruped',
            executable='LegController.py',
            output='screen',
            parameters=[{'which_leg': 'rl'}]
        ),
        Node(
            package='quadruped',
            executable='LegController.py',
            output='screen',
            parameters=[{'which_leg': 'rr'}]
        )
    ])
