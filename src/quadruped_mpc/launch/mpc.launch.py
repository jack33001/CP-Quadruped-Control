from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'imu_sensor_controller',
            'quadruped_broadcaster',
            'state_estimator',
            'balance_controller',
        ],
        output='screen',
    )

    return LaunchDescription([
        controllers_spawner,
    ])