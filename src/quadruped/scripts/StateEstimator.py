#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from quadruped.msg import LegCommand
from gait_pattern_class import gait_pattern_generator
from rclpy.duration import Duration

class StateEstimator(LifecycleNode):
    def __init__(self):
        super().__init__('gait_pattern_generator')
        
        # Set up lifecycle publishers, subscribers, and timers as None initially
        self.state_publisher = None
        self.state_subsciption = None
        self.timer = None

        # Gait pattern generator initialization
        self.pattern_generator = gait_pattern_generator()

        self.get_logger().info("Node initialized")

    def on_configure(self, state):
        """Lifecycle transition callback for the configuring state"""
        
        self.get_logger().info("Configuring...")
        
        # Set up publishers
        self.state_publisher = self.create_publisher(LegCommand, 'state_estimation', 10)

        # Set up timer for generating the pattern
        self.timer = self.create_timer(0.01, self.generate_pattern)

        # Set up state subscription
        self.state_subscription = self.create_subscription(
            LegCommand, 'joint_states', self.estimate_state, 10
        )

        self.get_logger().info("Configured successfully!")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """Lifecycle transition callback for the activating state"""
        self.get_logger().info("Activating...")

        # Check if publishers are created before activation
        if not self.state_publisher:
            self.get_logger().error("One or more publishers are not initialized. Cannot activate.")
            return TransitionCallbackReturn.FAILURE
        
        self.get_logger().info("Activated successfully!")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Lifecycle transition callback for the deactivating state"""
        self.get_logger().info("Deactivating...")

        self.get_logger().info("Deactivated successfully!")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Lifecycle transition callback for the cleaning up state"""
        self.get_logger().info("Cleaning up...")

        # Destroy publishers, timer, and subscription
        self.destroy_publisher(self.state_publisher)
        self.destroy_timer(self.timer)
        self.destroy_subscription(self.state_subscription)

        self.get_logger().info("Cleaned up successfully!")
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)

    state_estimator = StateEstimator()

    # Run the node's lifecycle
    rclpy.spin(state_estimator)

    gait_pattern_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
