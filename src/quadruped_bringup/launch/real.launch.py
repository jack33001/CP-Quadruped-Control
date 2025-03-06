from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():



    # # Define the RViz2 node   
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen'
    # )


   
    
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
    
    # Define the joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # # Define the joint_state_publisher node
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen'
    # )
    
    # # Define the package and launch file paths
    # hardware_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('quadruped_utils'),
    #             'launch',
    #             'testcontroller.launch.py'
    #         ])
    #     ])
    # )
    
    
    
    
    print('hardware_launch complete')
    
    # Return launch description with timed execution
    return LaunchDescription([
        urdf_launch,
        
        hardware_launch,
        
        joint_state_broadcaster_spawner
        # joint_state_publisher_node
        
    ])