#!/usr/bin/env python3
"""
Simple Square Pattern Flight for Detection Testing
Group 16 - Trail Guardian - 41068 Robotics Studio 1

Flies the drone in a square pattern to automatically test object detection.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math


class SquarePatternFlight(Node):
    def __init__(self):
        super().__init__('square_pattern_flight')
        
        # Parameters
        self.declare_parameter('square_size', 10.0)  # meters
        self.declare_parameter('flight_height', 3.0)  # meters
        self.declare_parameter('speed', 1.0)  # m/s
        
        self.square_size = self.get_parameter('square_size').value
        self.flight_height = self.get_parameter('flight_height').value
        self.speed = self.get_parameter('speed').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/parrot/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/parrot/goal_pose', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/parrot/odometry',
            self.odom_callback,
            10
        )
        
        # State
        self.current_position = None
        self.waypoint_index = 0
        self.waypoints = self.generate_square_waypoints()
        self.at_waypoint = False
        
        # Timer to check and publish goals
        self.timer = self.create_timer(1.0, self.navigate_to_next_waypoint)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Square Pattern Flight Initialized')
        self.get_logger().info(f'Square size: {self.square_size}m')
        self.get_logger().info(f'Flight height: {self.flight_height}m')
        self.get_logger().info(f'Speed: {self.speed}m/s')
        self.get_logger().info(f'Waypoints: {len(self.waypoints)}')
        self.get_logger().info('=' * 60)
    
    def generate_square_waypoints(self):
        """Generate waypoints for a square pattern centered at origin"""
        half_size = self.square_size / 2.0
        z = self.flight_height
        
        # Square corners (clockwise)
        waypoints = [
            (half_size, half_size, z),      # Top-right
            (half_size, -half_size, z),     # Bottom-right
            (-half_size, -half_size, z),    # Bottom-left
            (-half_size, half_size, z),     # Top-left
            (half_size, half_size, z),      # Back to start
        ]
        
        return waypoints
    
    def odom_callback(self, msg):
        """Update current position from odometry"""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
    
    def navigate_to_next_waypoint(self):
        """Navigate to the next waypoint in sequence"""
        if self.current_position is None:
            self.get_logger().warn('Waiting for odometry...')
            return
        
        # Get current waypoint
        target = self.waypoints[self.waypoint_index]
        
        # Calculate distance to waypoint
        distance = math.sqrt(
            (target[0] - self.current_position[0])**2 +
            (target[1] - self.current_position[1])**2 +
            (target[2] - self.current_position[2])**2
        )
        
        # Check if we've reached the waypoint
        if distance < 1.0:  # 1 meter threshold
            self.get_logger().info(
                f'Reached waypoint {self.waypoint_index + 1}/{len(self.waypoints)}'
            )
            
            # Move to next waypoint
            self.waypoint_index = (self.waypoint_index + 1) % len(self.waypoints)
            target = self.waypoints[self.waypoint_index]
            
            if self.waypoint_index == 0:
                self.get_logger().info('Completed one lap! Starting again...')
        
        # Publish goal
        goal = PoseStamped()
        goal.header.frame_id = 'parrot/odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = target[0]
        goal.pose.position.y = target[1]
        goal.pose.position.z = target[2]
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        
        self.get_logger().info(
            f'Flying to waypoint {self.waypoint_index + 1}: '
            f'({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}) - '
            f'Distance: {distance:.2f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SquarePatternFlight()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()