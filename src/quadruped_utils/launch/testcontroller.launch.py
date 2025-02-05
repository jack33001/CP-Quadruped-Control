from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=['config/controller_config.yaml'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_controller'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_position_controller'],
            output='screen'
        ),
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_group_position_controller'],
        #     output='screen'
        # ),
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_velocity_controller'],
        #     output='screen'
        # ),
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_group_velocity_controller'],
        #     output='screen'
        # ),
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_effort_controller'],
        #     output='screen'
        # ),
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_group_effort_controller'],
        #     output='screen'
        # ),
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_trajectory_controller'],
        #     output='screen'
        # ),
    ])