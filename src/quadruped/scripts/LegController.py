#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn, Publisher
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController
from quadruped.msg import LegCommand
from action_msgs.msg import GoalStatus
import leg_class

# Define lifecycle state constants for readability
PRIMARY_STATE_UNKNOWN = 0
PRIMARY_STATE_UNCONFIGURED = 1
PRIMARY_STATE_INACTIVE = 2
PRIMARY_STATE_ACTIVE = 3
PRIMARY_STATE_FINALIZED = 4

class LifecycleLegController(LifecycleNode):
    def __init__(self):
        super().__init__('leg_controller')
        
        # Initialize variables that will be set up in configure
        self.leg_name = None
        self.leg = None
        self.active_controller = None
        self.pos_publisher = None
        self.vel_publisher = None
        self.eff_publisher = None
        self.postraj_client = None
        self.efftraj_client = None
        self.switch_client = None
        self.gait_pattern_subscription = None
        
        # State tracking attribute
        self._current_state = PRIMARY_STATE_UNCONFIGURED
        
        # Create callback groups
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.action_group = ReentrantCallbackGroup()
        
        self.get_logger().info('Node initialized')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Lifecycle configure callback"""
        
        self.get_logger().info('Configuring...')
        
        try:
            self._current_state = PRIMARY_STATE_INACTIVE
            
            # Parameter declaration
            self.declare_parameter('which_leg', 'fl')
            self.leg_name = self.get_parameter('which_leg').get_parameter_value().string_value
            self.leg = leg_class.leg()
            
            self.active_controller = "position_controller"
            
            # Set up topic names
            pos_topic = f'/position_controller_{self.leg_name}_leg/commands'
            vel_topic = f'/velocity_controller_{self.leg_name}_leg/commands'
            eff_topic = f'/effort_controller_{self.leg_name}_leg/commands'
            postraj_topic = f'/position_trajectory_controller_{self.leg_name}_leg/follow_joint_trajectory'
            efftraj_topic = f'/effort_trajectory_controller_{self.leg_name}_leg/follow_joint_trajectory'
            
            # Create lifecycle publishers
            self.pos_publisher = self.create_lifecycle_publisher(Float64MultiArray, pos_topic, 10)
            self.vel_publisher = self.create_lifecycle_publisher(Float64MultiArray, vel_topic, 10)
            self.eff_publisher = self.create_lifecycle_publisher(Float64MultiArray, eff_topic, 10)
            
            # Create action clients
            self.postraj_client = ActionClient(
                self, 
                FollowJointTrajectory, 
                postraj_topic,
                callback_group=self.action_group
            )
            
            self.efftraj_client = ActionClient(
                self, 
                FollowJointTrajectory, 
                efftraj_topic,
                callback_group=self.action_group
            )
            
            # Create service client
            self.switch_client = self.create_client(
                SwitchController, 
                '/controller_manager/switch_controller',
                callback_group=self.service_group
            )
            
            # Set up subscriber
            self.gait_pattern_subscription = self.create_subscription(
                LegCommand,
                f'gait_pattern/{self.leg_name}',
                self.parse_cmd,
                10,
                callback_group=self.action_group
            )
            
            self.get_logger().info('Configured successfully!')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Error during configuration: {str(e)}')
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Lifecycle activate callback"""
        try:
            self._current_state = PRIMARY_STATE_ACTIVE
            
            # Activate all lifecycle publishers with state parameter
            self.pos_publisher.on_activate(state)
            self.vel_publisher.on_activate(state)
            self.eff_publisher.on_activate(state)
            
            # Wait for services and action servers
            timeout = 2.0
            while not (self.switch_client.wait_for_service(timeout_sec=timeout) and 
                self.efftraj_client.wait_for_server(timeout_sec=timeout) and 
                self.postraj_client.wait_for_server(timeout_sec=timeout)):
                self.get_logger().warn('Waiting for services and action servers...')
                
            self.get_logger().info('Activated successfully!')
            return TransitionCallbackReturn.SUCCESS
                
        except Exception as e:
            self.get_logger().error(f'Error during activation: {str(e)}')
            return TransitionCallbackReturn.ERROR

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Lifecycle deactivate callback"""
        try:
            self._current_state = PRIMARY_STATE_INACTIVE
            
            # Deactivate all lifecycle publishers with state parameter
            self.pos_publisher.on_deactivate(state)
            self.vel_publisher.on_deactivate(state)
            self.eff_publisher.on_deactivate(state)
            
            self.get_logger().info('Deactivated successfully!')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error during deactivation: {str(e)}')
            return TransitionCallbackReturn.ERROR

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Lifecycle cleanup callback"""
        try:
            self._current_state = PRIMARY_STATE_UNCONFIGURED
            self.get_logger().info('Cleaned up successfully!')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {str(e)}')
            return TransitionCallbackReturn.ERROR

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Lifecycle shutdown callback"""
        try:
            self._current_state = PRIMARY_STATE_FINALIZED
            self.get_logger().info('Shutting down...')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {str(e)}')
            return TransitionCallbackReturn.ERROR

    def get_current_state(self):
        """Custom function to retrieve the current state."""
        return self._current_state

    async def _switch_controller(self, desired_controller: str) -> bool:
        """
        Asynchronous version of controller switching
        """
        if not self.switch_client:
            self.get_logger().error('Switch client not initialized')
            return False

        if self.active_controller == desired_controller:
            return True
            
        request = SwitchController.Request()
        request.activate_controllers = [f"{desired_controller}_{self.leg_name}_leg"]
        request.deactivate_controllers = [f"{self.active_controller}_{self.leg_name}_leg"]
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = Duration(sec=5)
        
        try:
            response = await self.switch_client.call_async(request)
            if response is not None and response.ok:
                self.active_controller = desired_controller
                return True
            else:
                self.get_logger().error(f"Failed to switch to {desired_controller}")
                return False
        except Exception as e:
            self.get_logger().error(f"Exception while switching controllers: {str(e)}")
            return False

    async def parse_cmd(self, msg):
        """Handle incoming commands asynchronously"""
        try:
            if self.get_current_state() != PRIMARY_STATE_ACTIVE:
                self.get_logger().warn('Node not active, ignoring command')
                return

            if msg.cmd_type == "position":
                if await self._switch_controller("position_controller"):
                    position = self.leg.cartesian_to_joint(msg.position)
                    self._publish_position_command(position)
                else:
                    self.get_logger().error("Failed to switch to position controller")
                    
            elif msg.cmd_type == "position_trajectory":
                if await self._switch_controller("position_trajectory_controller"):
                    trajectory = self.leg.generate_joint_trajectory(msg.end_position, msg.position_end_time)
                    await self._publish_postraj_command(trajectory)
                else:
                    self.get_logger().error("Failed to switch to position trajectory controller")
                    
        except Exception as e:
            self.get_logger().error(f"Error in parse_cmd: {str(e)}")

    def _publish_position_command(self, cmd):
        """Publish position command asynchronously"""
        if not self.pos_publisher:
            self.get_logger().error('Position publisher not initialized')
            return
        
        if not self.pos_publisher:
            self.get_logger().error('Position publisher not initialized')
            return
        
        # Add these checks
        if not self.pos_publisher.is_activated:
            self.get_logger().warn('Position publisher is not activated!')
            return

        msg = Float64MultiArray()
        msg.data = cmd
        self.pos_publisher.publish(msg)

    async def _publish_postraj_command(self, trajectory_points):
        """Publish trajectory command asynchronously"""
        if not self.postraj_client:
            self.get_logger().error('Position trajectory client not initialized')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [f'{self.leg_name}_hip', f'{self.leg_name}_knee']

        for keypoint in trajectory_points:
            point = JointTrajectoryPoint()
            point.positions = keypoint[0]
            seconds = int(keypoint[1])
            nanoseconds = int((keypoint[1] - seconds) * 1e9)
            point.time_from_start = Duration(sec=seconds, nanosec=nanoseconds)
            goal_msg.trajectory.points.append(point)
            
        send_goal_future = await self.postraj_client.send_goal_async(goal_msg)
        if not send_goal_future.accepted:
            self.get_logger().error('Failed to send trajectory goal')
            return
        result = await send_goal_future.get_result_async()
        
        # Use GoalStatus.STATUS_SUCCEEDED to check for success
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('Trajectory goal did not succeed')

def main(args=None):
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        node = LifecycleLegController()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
