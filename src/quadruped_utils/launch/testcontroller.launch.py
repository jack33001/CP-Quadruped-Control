from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    
    
    # joint_state_controller = Node(
    #         package='controller_manager',
    #         executable='spawner',
    #         arguments=['joint_state_controller'],
    #         output='screen'
    #     )
    
    # joint_position_controller = Node(
    #         package='controller_manager',
    #         executable='spawner',
    #         arguments=['joint_position_controller'],
    #         output='screen'
    #     )

    
    joint_group_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_group_position_controller'],
            output='screen'
        )
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
        
    joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output='screen'
        )
        

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action= joint_group_position_controller,
                on_exit=[joint_state_broadcaster],
            )
        )
    
    nodes = [
            # joint_state_controller,
            joint_group_position_controller,

            delay_joint_state_broadcaster_after_robot_controller_spawner,
        ]
        
    
    
    return LaunchDescription(declared_arguments + nodes)
 
        
