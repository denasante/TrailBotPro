#!/usr/bin/env python3
"""
Simple Keyboard Teleop for Parrot Drone
Group 16 - Trail Guardian - 41068 Robotics Studio 1

Use arrow keys to fly the drone around and test object detection.

Controls:
    Arrow Up/Down: Forward/Backward
    Arrow Left/Right: Strafe Left/Right
    W/S: Up/Down (altitude)
    A/D: Yaw Left/Right
    Space: Stop/Hover
    Q: Quit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


class DroneKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('drone_keyboard_teleop')
        
        # Publisher for drone velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/parrot/cmd_vel',
            10
        )
        
        # Movement parameters
        self.linear_speed = 1.0   # m/s
        self.angular_speed = 0.5  # rad/s
        self.vertical_speed = 0.5 # m/s
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Drone Keyboard Teleop Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Controls:')
        self.get_logger().info('  Arrow Up/Down:    Forward/Backward')
        self.get_logger().info('  Arrow Left/Right: Strafe Left/Right')
        self.get_logger().info('  W/S:              Up/Down (altitude)')
        self.get_logger().info('  A/D:              Yaw Left/Right')
        self.get_logger().info('  Space:            Stop/Hover')
        self.get_logger().info('  Q:                Quit')
        self.get_logger().info('=' * 60)
        
        # Timer to publish at steady rate
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_twist = Twist()
        
        # Get original terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
    
    def get_key(self):
        """Get a single keypress from terminal"""
        tty.setraw(sys.stdin.fileno())
        
        # Check if key is available (non-blocking)
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1)
        else:
            key = ''
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def timer_callback(self):
        """Continuously publish the current twist command"""
        key = self.get_key()
        
        if key:
            self.process_key(key)
        
        # Publish current command
        self.cmd_vel_pub.publish(self.current_twist)
    
    def process_key(self, key):
        """Process keyboard input and update twist"""
        twist = Twist()
        
        # Arrow keys (special sequences)
        if key == '\x1b':  # ESC sequence for arrow keys
            next1 = sys.stdin.read(1)
            next2 = sys.stdin.read(1)
            
            if next1 == '[':
                if next2 == 'A':  # Up arrow
                    twist.linear.x = self.linear_speed
                    self.get_logger().info('Forward')
                elif next2 == 'B':  # Down arrow
                    twist.linear.x = -self.linear_speed
                    self.get_logger().info('Backward')
                elif next2 == 'D':  # Left arrow
                    twist.linear.y = self.linear_speed
                    self.get_logger().info('Strafe Left')
                elif next2 == 'C':  # Right arrow
                    twist.linear.y = -self.linear_speed
                    self.get_logger().info('Strafe Right')
        
        # Letter keys
        elif key == 'w' or key == 'W':
            twist.linear.z = self.vertical_speed
            self.get_logger().info('Ascending')
        elif key == 's' or key == 'S':
            twist.linear.z = -self.vertical_speed
            self.get_logger().info('Descending')
        elif key == 'a' or key == 'A':
            twist.angular.z = self.angular_speed
            self.get_logger().info('Yaw Left')
        elif key == 'd' or key == 'D':
            twist.angular.z = -self.angular_speed
            self.get_logger().info('Yaw Right')
        elif key == ' ':  # Spacebar
            twist = Twist()  # All zeros = stop
            self.get_logger().info('STOP - Hovering')
        elif key == 'q' or key == 'Q':
            self.get_logger().info('Quitting...')
            raise KeyboardInterrupt
        
        self.current_twist = twist
    
    def cleanup(self):
        """Restore terminal settings"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DroneKeyboardTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down teleop...")
    finally:
        # Restore terminal
        if 'node' in locals():
            node.cleanup()
        
        # Send stop command
        stop_node = rclpy.create_node('stop_drone')
        stop_pub = stop_node.create_publisher(Twist, '/parrot/cmd_vel', 10)
        stop_pub.publish(Twist())  # Send zero velocities
        stop_node.destroy_node()
        
        rclpy.shutdown()


if __name__ == '__main__':
    main()