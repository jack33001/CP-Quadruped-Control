#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool
from pynput import keyboard
import math

class QuadrupedTeleop(Node):
    def __init__(self):
        super().__init__('quadruped_teleop')
        # Create publishers for separate pose and twist topics
        self.pose_publisher = self.create_publisher(Pose, '/quadruped/cmd/pose_cmd', 10)
        self.twist_publisher = self.create_publisher(Twist, '/quadruped/cmd/twist_cmd', 10)
        # Add publisher for gait start command
        self.gait_start_publisher = self.create_publisher(Bool, '/quadruped/cmd/gait_start', 10)
        
        # Initialize pose and twist messages
        self.pose = Pose()
        self.twist = Twist()
        
        # Initialize pose with position [0, 0, 0.2] and quaternion [1, 0, 0, 0] (identity)
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        self.pose.position.z = 0.1  # Set initial height to 0.2m
        self.pose.orientation.w = 1.0
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = 0.0
        
        # Initialize twist with zero velocities
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        
        # Movement parameters
        self.position_step = 0.01  # m
        self.rotation_step = 0.01  # rad
        self.velocity_step = 0.01  # m/s or rad/s
        
        # Track shift key state
        self.shift_pressed = False
        
        # Track gait start state
        self.gait_started = False
        
        # Initialize keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()
        
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Log control scheme
        self.get_logger().info('Quadruped Teleop node started, publishing to topics:')
        self.get_logger().info('- /quadruped/cmd/pose_cmd (Pose)')
        self.get_logger().info('- /quadruped/cmd/twist_cmd (Twist)')
        self.get_logger().info('- /quadruped/cmd/gait_start (Bool)')
        self.get_logger().info('Control Scheme:')
        self.get_logger().info('W/S: +/- X position')
        self.get_logger().info('A/D: +/- Y position')
        self.get_logger().info('Q/E: +/- Yaw')
        self.get_logger().info('Up/Down: +/- Z position')
        self.get_logger().info('Left/Right: +/- Roll')
        self.get_logger().info('Shift + W/S: +/- X velocity')
        self.get_logger().info('Shift + A/D: +/- Y velocity')
        self.get_logger().info('Shift + Q/E: +/- Yaw rate')
        self.get_logger().info('SPACE: Start gait pattern')
        self.get_logger().info('ESC: Exit')
        
    def timer_callback(self):
        # Publish to separate topics
        self.pose_publisher.publish(self.pose)
        self.twist_publisher.publish(self.twist)
        
    def on_press(self, key):
        # Check if it's the shift key
        if key == keyboard.Key.shift:
            self.shift_pressed = True
            return
            
        # Check if it's the space key (for starting gait)
        if key == keyboard.Key.space:
            self.gait_started = True
            gait_start_msg = Bool()
            gait_start_msg.data = True
            self.gait_start_publisher.publish(gait_start_msg)
            self.get_logger().info('Gait start command sent')
            return
            
        # Handle character keys
        if hasattr(key, 'char'):
            key_char = key.char
            if key_char is None:
                return
                
            # Make sure key_char is lowercase for consistent comparisons
            key_char = key_char.lower()
            
            if self.shift_pressed:
                # Velocity control (Twist)
                if key_char == 'w':
                    self.twist.linear.x += self.velocity_step
                    self.get_logger().info(f'X velocity: {self.twist.linear.x:.3f}')
                elif key_char == 's':
                    self.twist.linear.x -= self.velocity_step
                    self.get_logger().info(f'X velocity: {self.twist.linear.x:.3f}')
                elif key_char == 'a':
                    self.twist.linear.y += self.velocity_step
                    self.get_logger().info(f'Y velocity: {self.twist.linear.y:.3f}')
                elif key_char == 'd':
                    self.twist.linear.y -= self.velocity_step
                    self.get_logger().info(f'Y velocity: {self.twist.linear.y:.3f}')
                elif key_char == 'q':
                    self.twist.angular.z += self.velocity_step
                    self.get_logger().info(f'Yaw rate: {self.twist.angular.z:.3f}')
                elif key_char == 'e':
                    self.twist.angular.z -= self.velocity_step
                    self.get_logger().info(f'Yaw rate: {self.twist.angular.z:.3f}')
            else:
                # Position control (Pose)
                if key_char == 'w':
                    self.pose.position.x += self.position_step
                    self.get_logger().info(f'X position: {self.pose.position.x:.3f}')
                elif key_char == 's':
                    self.pose.position.x -= self.position_step
                    self.get_logger().info(f'X position: {self.pose.position.x:.3f}')
                elif key_char == 'a':
                    self.pose.position.y += self.position_step
                    self.get_logger().info(f'Y position: {self.pose.position.y:.3f}')
                elif key_char == 'd':
                    self.pose.position.y -= self.position_step
                    self.get_logger().info(f'Y position: {self.pose.position.y:.3f}')
                elif key_char == 'q':
                    self.apply_yaw_rotation(self.rotation_step)
                    self.get_logger().info(f'Yaw: increased by {self.rotation_step:.3f}')
                elif key_char == 'e':
                    self.apply_yaw_rotation(-self.rotation_step)
                    self.get_logger().info(f'Yaw: decreased by {self.rotation_step:.3f}')
        # Handle special keys
        else:
            if key == keyboard.Key.up:
                self.pose.position.z += self.position_step
                self.get_logger().info(f'Z position: {self.pose.position.z:.3f}')
            elif key == keyboard.Key.down:
                self.pose.position.z -= self.position_step
                self.get_logger().info(f'Z position: {self.pose.position.z:.3f}')
            elif key == keyboard.Key.left:
                self.apply_roll_rotation(self.rotation_step)
                self.get_logger().info(f'Roll: increased by {self.rotation_step:.3f}')
            elif key == keyboard.Key.right:
                self.apply_roll_rotation(-self.rotation_step)
                self.get_logger().info(f'Roll: decreased by {self.rotation_step:.3f}')
                
    def on_release(self, key):
        if key == keyboard.Key.shift:
            self.shift_pressed = False
        elif key == keyboard.Key.esc:
            # Stop the listener
            return False

    def apply_yaw_rotation(self, angle):
        """Apply a yaw rotation by updating the quaternion properly"""
        # Extract current quaternion
        q_w = self.pose.orientation.w
        q_x = self.pose.orientation.x
        q_y = self.pose.orientation.y
        q_z = self.pose.orientation.z
        
        # Create quaternion for incremental yaw rotation
        half_angle = angle / 2.0
        cos_half = math.cos(half_angle)
        sin_half = math.sin(half_angle)
        
        # Incremental rotation quaternion (about Z axis)
        dq_w = cos_half
        dq_x = 0.0
        dq_y = 0.0
        dq_z = sin_half
        
        # Quaternion multiplication
        new_q_w = q_w * dq_w - q_x * dq_x - q_y * dq_y - q_z * dq_z
        new_q_x = q_w * dq_x + q_x * dq_w + q_y * dq_z - q_z * dq_y
        new_q_y = q_w * dq_y - q_x * dq_z + q_y * dq_w + q_z * dq_x
        new_q_z = q_w * dq_z + q_x * dq_y - q_y * dq_x + q_z * dq_w
        
        # Normalize quaternion
        norm = math.sqrt(new_q_w**2 + new_q_x**2 + new_q_y**2 + new_q_z**2)
        self.pose.orientation.w = new_q_w / norm
        self.pose.orientation.x = new_q_x / norm
        self.pose.orientation.y = new_q_y / norm
        self.pose.orientation.z = new_q_z / norm

    def apply_roll_rotation(self, angle):
        """Apply a roll rotation by updating the quaternion properly"""
        # Extract current quaternion
        q_w = self.pose.orientation.w
        q_x = self.pose.orientation.x
        q_y = self.pose.orientation.y
        q_z = self.pose.orientation.z
        
        # Create quaternion for incremental roll rotation
        half_angle = angle / 2.0
        cos_half = math.cos(half_angle)
        sin_half = math.sin(half_angle)
        
        # Incremental rotation quaternion (about X axis)
        dq_w = cos_half
        dq_x = sin_half
        dq_y = 0.0
        dq_z = 0.0
        
        # Quaternion multiplication
        new_q_w = q_w * dq_w - q_x * dq_x - q_y * dq_y - q_z * dq_z
        new_q_x = q_w * dq_x + q_x * dq_w + q_y * dq_z - q_z * dq_y
        new_q_y = q_w * dq_y - q_x * dq_z + q_y * dq_w + q_z * dq_x
        new_q_z = q_w * dq_z + q_x * dq_y - q_y * dq_x + q_z * dq_w
        
        # Normalize quaternion
        norm = math.sqrt(new_q_w**2 + new_q_x**2 + new_q_y**2 + new_q_z**2)
        self.pose.orientation.w = new_q_w / norm
        self.pose.orientation.x = new_q_x / norm
        self.pose.orientation.y = new_q_y / norm
        self.pose.orientation.z = new_q_z / norm

def main(args=None):
    rclpy.init(args=args)
    teleop = QuadrupedTeleop()
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
