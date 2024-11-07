#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from quadruped.msg import LegCommand
from gait_pattern_class import gait_pattern_generator
from rclpy.duration import Duration

class GaitPatternGenerator(LifecycleNode):
    def __init__(self):
        super().__init__('gait_pattern_generator')
        
        # Set up lifecycle publishers, subscribers, and timers as None initially
        self.gait_publisher_fl = None
        self.gait_publisher_fr = None
        self.gait_publisher_rl = None
        self.gait_publisher_rr = None
        self.timer = None
        self.state_subscription = None

        # Gait pattern generator initialization
        self.pattern_generator = gait_pattern_generator()

        self.get_logger().info("Node initialized")

    def on_configure(self, state):
        """Lifecycle transition callback for the configuring state"""
        
        self.get_logger().info("Configuring...")
        
        # Set up publishers
        self.gait_publisher_fl = self.create_publisher(LegCommand, 'gait_pattern/fl', 10)
        self.gait_publisher_fr = self.create_publisher(LegCommand, 'gait_pattern/fr', 10)
        self.gait_publisher_rl = self.create_publisher(LegCommand, 'gait_pattern/rl', 10)
        self.gait_publisher_rr = self.create_publisher(LegCommand, 'gait_pattern/rr', 10)

        # Set up timer for generating the pattern
        self.timer = self.create_timer(0.01, self.generate_pattern)

        # Set up state subscription
        self.state_subscription = self.create_subscription(
            LegCommand, 'state_estimation', self.generate_pattern, 10
        )

        # Initialize useful variables for the pattern generator
        self.pattern_generator.t0 = self.get_clock().now().seconds_nanoseconds()[0] + \
            self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        self.get_logger().info("Configured successfully!")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """Lifecycle transition callback for the activating state"""
        self.get_logger().info("Activating...")

        # Check if publishers are created before activation
        if not self.gait_publisher_fl or not self.gait_publisher_fr or not self.gait_publisher_rl or not self.gait_publisher_rr:
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
        self.destroy_publisher(self.gait_publisher_fl)
        self.destroy_publisher(self.gait_publisher_fr)
        self.destroy_publisher(self.gait_publisher_rl)
        self.destroy_publisher(self.gait_publisher_rr)
        self.destroy_timer(self.timer)
        self.destroy_subscription(self.state_subscription)

        self.get_logger().info("Cleaned up successfully!")
        return TransitionCallbackReturn.SUCCESS

    def generate_pattern(self, msg=None):
        """Generate the gait pattern and publish it to each leg's topic"""
        # Update state if called by subscriber
        if msg is not None:
            # Update state if necessary
            pass

        # Generate the pattern
        self.pattern_generator.t = (
            self.get_clock().now().seconds_nanoseconds()[0]
            + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
            - self.pattern_generator.t0
        )
        pattern = self.pattern_generator.ftw_position_generator()

        # Pack the messages
        outgoing_msgs = [LegCommand() for _ in range(len(pattern))]
        for i, pattern_data in enumerate(pattern):
            if pattern_data['cmd_type'] == "position":
                outgoing_msgs[i].cmd_type = pattern_data['cmd_type']
                outgoing_msgs[i].position = pattern_data['position']
            elif pattern_data['cmd_type'] == "position_trajectory":
                outgoing_msgs[i].cmd_type = pattern_data['cmd_type']
                outgoing_msgs[i].end_position = pattern_data['end_position']
                outgoing_msgs[i].position_end_time = pattern_data['position_end_time']
                outgoing_msgs[i].position_kp = pattern_data['position_kp']
                outgoing_msgs[i].position_kd = pattern_data['position_kd']
            elif pattern_data['cmd_type'] == "impulse":
                outgoing_msgs[i].cmd_type = pattern_data['cmd_type']
                outgoing_msgs[i].total_impulse = pattern_data['impulse']
                outgoing_msgs[i].impulse_time = pattern_data['impulse_time']

        # Publish leg commands
        if outgoing_msgs[0].cmd_type != "none":
            self.gait_publisher_fl.publish(outgoing_msgs[0])
        if outgoing_msgs[1].cmd_type != "none":
            self.gait_publisher_fr.publish(outgoing_msgs[1])
        if outgoing_msgs[2].cmd_type != "none":
            self.gait_publisher_rl.publish(outgoing_msgs[2])
        if outgoing_msgs[3].cmd_type != "none":
            self.gait_publisher_rr.publish(outgoing_msgs[3])


def main(args=None):
    rclpy.init(args=args)

    gait_pattern_generator = GaitPatternGenerator()

    # Run the node's lifecycle
    rclpy.spin(gait_pattern_generator)

    gait_pattern_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
