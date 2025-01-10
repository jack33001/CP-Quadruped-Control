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
        output='screen'
    )

    print('rvis_node complete')
    
    # Define the package and launch file paths
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('quadruped_urdf'),
                'launch',
                'real_urdf.launch.py'
            ])
        ])
    )

    print('urdf_launch complete')
    
    # Define the package and launch file paths
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('quadruped_hardware'),
                'launch',
                'controller_manager.launch.py'
            ])
        ])
    )
    
    print('hardware_launch complete')
    
    
    # Return launch description with timed execution
    return LaunchDescription([
        urdf_launch,
        # rviz_node,
        # hardware_launch
        
    ])