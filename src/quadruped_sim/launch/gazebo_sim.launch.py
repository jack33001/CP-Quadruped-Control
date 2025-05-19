from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro
import datetime
import shutil  # Add this import
import uuid  # Add this import

def generate_launch_description():
    # Package and path definitions
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')
    pkg_ros_gz_bridge = FindPackageShare('ros_gz_bridge').find('ros_gz_bridge')

    # Clock bridge node
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'qos_overrides./clock.publisher.durability': 'transient_local',
            'qos_overrides./clock.publisher.reliability': 'reliable',
        }],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'] 
    )

    # Odometry bridge node
    state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='state_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # Bridge Gazebo odometry to ROS topic
            '/model/quadruped/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',  
        ],
        remappings=[
            ('/model/quadruped/odometry' , '/quadruped/state/ground_truth/odometry')  # remap Gazebo publication
        ]
    )

    # Spawn robot node
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot_urdf',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '.3',
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                   '-entity', 'Edward'],
        parameters=[{'use_sim_time': True, 'update_rate': 1000}]
    )

    # PlotJuggler node with only auto-subscribe
    plotjuggler = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        parameters=[{'use_sim_time': True}]
    )

    # Configure storage location for ROS2 bags with unique timestamp
    base_path = os.path.join(os.getcwd(), 'src', 'quadruped_sim', 'logs')
    if os.path.exists(base_path):
        shutil.rmtree(base_path)
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    unique_id = str(uuid.uuid4())[:8]
    storage_path = os.path.join(base_path, f"{timestamp}-{unique_id}")
    os.makedirs(storage_path)

    # ROS2 bag recording node with only pose data
    bag_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '--output', storage_path,
             '/world/empty/pose/info'],
        output='screen'
    )

    return LaunchDescription([
        # Include Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'paused': 'true',
                'gui': 'true',
                'recording': 'false',
                'debug': 'false',
                'verbose': 'true',
                'gz_args': '-r empty.sdf',
            }.items()
        ),
        clock_bridge,
        # Spawn robot first
        spawn_robot,
        # Then bridge its pose and contacts
        state_bridge,
        bag_recorder,
    ])
