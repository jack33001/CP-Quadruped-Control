from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the RViz2 node   
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('quadruped_mpc'),
            'rviz',
            'mpc.rviz'
        ])],
        output='screen'
    )

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
        period=2.0,  # Wait for 5 seconds after URDF launch
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
        period = 7.0,  # Wait for 10 seconds after launch start
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
        rviz_node,
        gazebo_launch,
        mpc_launch,
    ])