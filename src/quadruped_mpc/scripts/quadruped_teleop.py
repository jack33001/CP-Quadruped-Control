#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class QuadrupedTeleop(Node):
    def __init__(self):
        super().__init__('quadruped_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        
        # Movement parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # Initialize keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()
        
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def timer_callback(self):
        self.publisher.publish(self.twist)
        
    def on_press(self, key):
        try:
            if key.char == 'w':
                self.twist.linear.x = self.linear_speed
            elif key.char == 's':
                self.twist.linear.x = -self.linear_speed
            elif key.char == 'a':
                self.twist.linear.y = self.linear_speed
            elif key.char == 'd':
                self.twist.linear.y = -self.linear_speed
            elif key.char == 'q':
                self.twist.angular.z = self.angular_speed
            elif key.char == 'e':
                self.twist.angular.z = -self.angular_speed
        except AttributeError:
            if key == keyboard.Key.up:
                self.twist.linear.z = self.linear_speed
            elif key == keyboard.Key.down:
                self.twist.linear.z = -self.linear_speed
            elif key == keyboard.Key.left:
                self.twist.angular.x = self.angular_speed
            elif key == keyboard.Key.right:
                self.twist.angular.x = -self.angular_speed
                
    def on_release(self, key):
        try:
            if key.char in ['w', 's']:
                self.twist.linear.x = 0.0
            elif key.char in ['a', 'd']:
                self.twist.linear.y = 0.0
            elif key.char in ['q', 'e']:
                self.twist.angular.z = 0.0
        except AttributeError:
            if key in [keyboard.Key.up, keyboard.Key.down]:
                self.twist.linear.z = 0.0
            elif key in [keyboard.Key.left, keyboard.Key.right]:
                self.twist.angular.x = 0.0
                
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
