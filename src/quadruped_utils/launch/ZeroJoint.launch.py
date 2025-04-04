from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    controllers_spawner = Node(
        package='quadruped_utils',
        executable='spawner',
        arguments=[
            'zero_cont'
        ],
        output='screen',
    )

    return LaunchDescription([
        controllers_spawner,
    ])