from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from lifecycle_msgs.msg import Transition
import os
import xacro

def generate_launch_description():
    active_controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[{'use_sim_time': True}],
        arguments=[
            'joint_state_broadcaster',
            'imu_sensor_controller',
            'balance_controller'
        ],
        output='screen',
    )

    return LaunchDescription([
        active_controllers_spawner,
    ])
