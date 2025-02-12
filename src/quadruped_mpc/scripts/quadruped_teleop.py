#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from pynput import keyboard

class QuadrupedTeleop(Node):
    def __init__(self):
        super().__init__('quadruped_teleop')
        self.publisher = self.create_publisher(Pose, 'quadruped_cmd', 10)
        self.pose = Pose()
        
        # Movement parameters
        self.position_step = 0.01  # m
        self.rotation_step = 0.01  # rad
        
        # Initialize keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()
        
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Log control scheme
        self.get_logger().info('Quadruped Teleop node started, publishing to topic: quadruped_cmd')
        self.get_logger().info('Control Scheme:')
        self.get_logger().info('W/S: +/- X position')
        self.get_logger().info('A/D: +/- Y position')
        self.get_logger().info('Q/E: +/- Yaw')
        self.get_logger().info('Up/Down: +/- Z position')
        self.get_logger().info('Left/Right: +/- Roll')
        self.get_logger().info('ESC: Exit')
        
    def timer_callback(self):
        self.publisher.publish(self.pose)
        
    def on_press(self, key):
        try:
            key_char = key.char
        except AttributeError:
            key_char = str(key)
        
        try:
            if key.char == 'w':
                self.pose.position.x += self.position_step
                self.get_logger().info(f'X position: {self.pose.position.x:.3f}')
            elif key.char == 's':
                self.pose.position.x -= self.position_step
                self.get_logger().info(f'X position: {self.pose.position.x:.3f}')
            elif key.char == 'a':
                self.pose.position.y += self.position_step
                self.get_logger().info(f'Y position: {self.pose.position.y:.3f}')
            elif key.char == 'd':
                self.pose.position.y -= self.position_step
                self.get_logger().info(f'Y position: {self.pose.position.y:.3f}')
            elif key.char == 'q':
            # Note: This is a simplification. For proper orientation control,
            # we should use quaternion operations
                self.pose.orientation.z += self.rotation_step
                self.get_logger().info(f'Yaw: {self.pose.orientation.z:.3f}')
            elif key.char == 'e':
                self.pose.orientation.z -= self.rotation_step
                self.get_logger().info(f'Yaw: {self.pose.orientation.z:.3f}')
        except AttributeError:
            if key == keyboard.Key.up:
                self.pose.position.z += self.position_step
            elif key == keyboard.Key.down:
                self.pose.position.z -= self.position_step
            elif key == keyboard.Key.left:
                self.pose.orientation.x += self.rotation_step
            elif key == keyboard.Key.right:
                self.pose.orientation.x -= self.rotation_step
                
    def on_release(self, key):
        if key == keyboard.Key.esc:
            return False

def main(args=None):
    rclpy.init(args=args)
    teleop = QuadrupedTeleop()
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
