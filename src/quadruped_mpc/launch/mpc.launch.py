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
            'state_estimator',
            'gait_pattern_generator',
            'balance_controller',
            'foot_controller'
        ],
        output='screen',
    )

    # Add teleop node
    teleop_node = Node(
        package='quadruped_mpc',
        executable='quadruped_teleop.py',
        name='quadruped_teleop',
        output='screen'
    )

    return LaunchDescription([
        controllers_spawner,
        teleop_node,
    ])                          