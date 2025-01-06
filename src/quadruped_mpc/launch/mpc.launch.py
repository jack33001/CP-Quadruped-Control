from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get package directory
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Create action to run generate_controller script
    generate_controller = ExecuteProcess(
        cmd=['python3', os.path.join(package_dir, 'scripts/acados/generate_controller.py')],
        output='screen',
        shell=False
    )

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
        generate_controller,  # Run this first
        controllers_spawner,
    ])