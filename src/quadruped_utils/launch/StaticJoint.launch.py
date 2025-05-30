from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

 
    controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'static_joints_controller',

        ],
        parameters=[{
            'use_sim_time': False#LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )
    
    return LaunchDescription([
        controllers_spawner
    ])