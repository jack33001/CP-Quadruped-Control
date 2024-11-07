from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from lifecycle_msgs.msg import Transition
import os
import xacro

def generate_launch_description():
    # Package and path definitions
    pkg_quadruped = FindPackageShare('quadruped').find('quadruped')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')
    xacro_file = os.path.join(pkg_quadruped, 'urdf', 'QuadrupedURDF.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    controller_config_path = PathJoinSubstitution(
        [pkg_quadruped, 'config', 'quadruped_controllers.yaml'])

    # Environment variable setting
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=[FindPackageShare("quadruped").find("quadruped"), ":", 
               os.environ.get('GAZEBO_MODEL_PATH', '')])

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot_urdf',
        output='screen',
        arguments=['-topic', 'robot_description',
                  '-x', '0.0', '-y', '0.0', '-z', '1.0', 
                  '-entity', 'Edward'],
        parameters=[{'use_sim_time': True, 'update_rate': 1000}]
    )

    active_controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[{'use_sim_time': True}],
        arguments=[
            'joint_state_broadcaster'
            'imu_sensor_controller',
        ],
        output='screen',
    )

    '''inactive_controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[{'use_sim_time': True}],
        arguments=[
            'position_trajectory_controller_fr_leg',
            'position_trajectory_controller_fl_leg',
            'position_trajectory_controller_rr_leg',
            'position_trajectory_controller_rl_leg',
            'velocity_controller_fr_leg',
            'velocity_controller_fl_leg',
            'velocity_controller_rr_leg',
            'velocity_controller_rl_leg',
            'effort_controller_fr_leg',
            'effort_controller_fl_leg',
            'effort_controller_rr_leg',
            'effort_controller_rl_leg',
            'effort_trajectory_controller_fr_leg',
            'effort_trajectory_controller_fl_leg',
            'effort_trajectory_controller_rr_leg',
            'effort_trajectory_controller_rl_leg',
            '--inactive'
        ],
        output='screen',
    )

    # Define the lifecycle nodes for each leg
    leg_controllers = {
        'fl': 'Front_Left_Leg_Controller',
        'fr': 'Front_Right_Leg_Controller',
        'rl': 'Rear_Left_Leg_Controller',
        'rr': 'Rear_Right_Leg_Controller'
    }

    lifecycle_nodes = []
    configure_events = []
    activate_events = []

    for leg_id, node_name in leg_controllers.items():
        # Create lifecycle node with namespace
        lifecycle_node = LifecycleNode(
            package='quadruped',
            executable='LegController.py',
            name=node_name,
            namespace='',  # Set an explicit namespace
            output='screen',
            parameters=[{'which_leg': leg_id, 'use_sim_time': True}]
        )
        lifecycle_nodes.append(lifecycle_node)

        # Create configure event
        configure_event = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(lifecycle_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        )
        configure_events.append(configure_event)

        # Create activate event
        activate_event = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(lifecycle_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            )
        )
        activate_events.append(activate_event)

        # Register handler to activate after configuration
        lifecycle_nodes.append(
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=lifecycle_node,
                    goal_state='inactive',
                    entities=[activate_event],
                )
            )
        )

    # GaitPatternGenerator as a lifecycle node
    gait_pattern_generator_node = LifecycleNode(
        package='quadruped',
        executable='GaitPatternGenerator.py',
        name='Gait_Pattern_Generator',
        namespace='',  # Set an explicit namespace
        output='screen'
    )

    # Gait pattern generator lifecycle events
    gait_pattern_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(gait_pattern_generator_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # This event will activate the gait pattern generator
    gait_pattern_activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(gait_pattern_generator_node),
            transition_id=Transition.TRANSITION_ACTIVATE
        )
    )

    # Add gait pattern generator to the exit handlers
    gait_pattern_generator_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=inactive_controllers_spawner,
            on_exit=[gait_pattern_generator_node, gait_pattern_configure_event]
        )
    )

    gait_pattern_generator_active_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=active_controllers_spawner,
            on_exit=[gait_pattern_activate_event]
        )
    )

    # Ensure LegControllers wait for both active and inactive controllers
    for leg_controller in lifecycle_nodes:
        leg_controller_wait_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=active_controllers_spawner,
                on_exit=[leg_controller],
            )
        )

        # Ensure activation happens after both controllers are available
        leg_controller_wait_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=inactive_controllers_spawner,
                on_exit=[leg_controller],
            )
        )'''

    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description
            }]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'paused': 'true',
                'use_sim_time': 'true',
                'gui': 'true',
                'recording': 'false',
                'debug': 'false',
                'verbose': 'true',
                'gz_args': '-r empty.sdf',
            }.items()
        ),
        spawn_robot,
        #joint_state_broadcaster_spawner,
        active_controllers_spawner,
        #inactive_controllers_spawner,
        #gait_pattern_generator_handler,
        #gait_pattern_generator_active_handler,
        #*lifecycle_nodes,
        #*configure_events
    ])
