from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the package and launch file paths
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('quadruped_urdf'),
                'launch',
                'sim_urdf.launch.py'
            ])
        ])
    )

    # Wrap gazebo launch in TimerAction for delayed start
    gazebo_launch = TimerAction(
        period=5.0,  # Wait for 5 seconds after URDF launch
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('quadruped_sim'),
                        'launch',
                        'gazebo_sim.launch.py'
                    ])
                ])
            )
        ]
    )

    # Wrap MPC launch in TimerAction for delayed start
    mpc_launch = TimerAction(
        period=10.0,  # Wait for 10 seconds after launch start
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('quadruped_mpc'),
                        'launch',
                        'mpc.launch.py'
                    ])
                ])
            )
        ]
    )

    # Return launch description with timed execution
    return LaunchDescription([
        urdf_launch,
        gazebo_launch,
        mpc_launch
    ])