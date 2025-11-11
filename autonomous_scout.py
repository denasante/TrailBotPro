#!/usr/bin/env python3
"""
Trail Guardian Autonomous Scout Controller
Group 16 - 41068 Robotics Studio 1

Autonomous exploration system that:
- Plans systematic coverage paths (lawnmower pattern)
- Scouts the 15m x 15m world bounds
- Identifies trees and obstacles
- Maintains safe distances
- Reports discoveries to mission coordinator
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
import math


class AutonomousScout(Node):
    def __init__(self):
        super().__init__('autonomous_scout')
        
        # =====================================================================
        # PARAMETERS
        # =====================================================================
        self.declare_parameter('cruise_altitude', 3.0)  # meters
        self.declare_parameter('cruise_speed', 0.5)  # m/s
        self.declare_parameter('scan_spacing', 3.0)  # spacing between scan lines
        self.declare_parameter('world_bounds', 7.5)  # world is ±7.5m from origin
        self.declare_parameter('safety_distance', 1.5)  # minimum distance to obstacles
        self.declare_parameter('takeoff_height', 2.5)  # initial takeoff height
        
        self.cruise_altitude = self.get_parameter('cruise_altitude').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.scan_spacing = self.get_parameter('scan_spacing').value
        self.world_bounds = self.get_parameter('world_bounds').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.takeoff_height = self.get_parameter('takeoff_height').value
        
        # =====================================================================
        # STATE MACHINE
        # =====================================================================
        self.state = 'IDLE'  # IDLE, TAKEOFF, SCOUTING, INVESTIGATING, RETURNING, LANDED
        self.current_waypoint = 0
        self.waypoints = []
        
        # =====================================================================
        # ROBOT STATE
        # =====================================================================
        self.current_pose = None
        self.current_velocity = None
        self.obstacles_nearby = False
        self.trees_detected = []
        
        # =====================================================================
        # PUBLISHERS
        # =====================================================================
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/parrot/cmd_vel',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/trail_guardian/scout_status',
            10
        )
        
        self.mission_pub = self.create_publisher(
            PoseStamped,
            '/trail_guardian/poi_report',
            10
        )
        
        # =====================================================================
        # SUBSCRIBERS
        # =====================================================================
        self.odom_sub = self.create_subscription(
            Odometry,
            '/parrot/odometry',
            self.odometry_callback,
            10
        )
        
        self.tree_sub = self.create_subscription(
            PoseStamped,
            '/trail_guardian/tree_location',
            self.tree_callback,
            10
        )
        
        # =====================================================================
        # CONTROL LOOP
        # =====================================================================
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        # =====================================================================
        # PARAMETERS
        # =====================================================================
        # Position tolerance (m)
        self.position_tolerance = 0.3
        
        # PID gains for position control
        self.kp_xy = 0.8
        self.kp_z = 0.5
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Trail Guardian Autonomous Scout Initialized')
        self.get_logger().info(f'World bounds: ±{self.world_bounds}m')
        self.get_logger().info(f'Cruise altitude: {self.cruise_altitude}m')
        self.get_logger().info(f'Scan spacing: {self.scan_spacing}m')
        self.get_logger().info('Waiting for odometry...')
        self.get_logger().info('=' * 60)

    def odometry_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
        
        # Auto-start scouting once we have odometry
        if self.state == 'IDLE' and self.current_pose is not None:
            self.start_mission()

    def tree_callback(self, msg):
        """Record detected trees"""
        tree_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        
        # Check if this is a new tree (not already in list)
        is_new = True
        for known_tree in self.trees_detected:
            dist = math.sqrt(
                (tree_pos[0] - known_tree[0])**2 +
                (tree_pos[1] - known_tree[1])**2
            )
            if dist < 2.0:  # Within 2m = same tree
                is_new = False
                break
        
        if is_new:
            self.trees_detected.append(tree_pos)
            self.get_logger().info(f'New tree logged: {len(self.trees_detected)} total')
            
            # Report POI to mission coordinator
            poi_msg = PoseStamped()
            poi_msg.header.stamp = self.get_clock().now().to_msg()
            poi_msg.header.frame_id = 'parrot/base_link'
            poi_msg.pose = msg.pose
            self.mission_pub.publish(poi_msg)

    def start_mission(self):
        """Initialize and start the scouting mission"""
        self.get_logger().info('🚁 STARTING AUTONOMOUS SCOUTING MISSION')
        
        # Generate coverage path (lawnmower pattern)
        self.waypoints = self.generate_coverage_path()
        
        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints')
        self.current_waypoint = 0
        self.state = 'SCOUTING'
        
        self.publish_status('Mission started - beginning coverage pattern')

    def generate_coverage_path(self):
        """Generate lawnmower pattern for systematic coverage"""
        waypoints = []
        
        # Start position (current position or default)
        if self.current_pose:
            start_x = self.current_pose.position.x
            start_y = self.current_pose.position.y
        else:
            start_x, start_y = 0.0, 0.0
        
        # Coverage bounds (stay within world limits with safety margin)
        x_min = -self.world_bounds + 1.0
        x_max = self.world_bounds - 1.0
        y_min = -self.world_bounds + 1.0
        y_max = self.world_bounds - 1.0
        
        # Lawnmower pattern
        y = y_min
        direction = 1  # 1 for right, -1 for left
        
        while y <= y_max:
            if direction == 1:
                # Move right
                waypoints.append((x_min, y, self.cruise_altitude))
                waypoints.append((x_max, y, self.cruise_altitude))
            else:
                # Move left
                waypoints.append((x_max, y, self.cruise_altitude))
                waypoints.append((x_min, y, self.cruise_altitude))
            
            # Move to next scan line
            y += self.scan_spacing
            direction *= -1
        
        # Add return to start
        waypoints.append((start_x, start_y, self.cruise_altitude))
        waypoints.append((start_x, start_y, 0.5))  # Descend for landing
        
        return waypoints

    def control_loop(self):
        """Main control loop - runs at 10 Hz"""
        
        if self.current_pose is None:
            return  # Waiting for odometry
        
        if self.state == 'IDLE':
            # Do nothing, waiting to start
            pass
        
        elif self.state == 'SCOUTING':
            self.execute_scouting()
        
        elif self.state == 'RETURNING':
            self.execute_return()
        
        elif self.state == 'LANDED':
            # Mission complete
            self.stop_drone()

    def execute_scouting(self):
        """Execute the coverage path waypoint navigation"""
        
        if self.current_waypoint >= len(self.waypoints):
            # Mission complete
            self.get_logger().info('🎉 SCOUTING MISSION COMPLETE!')
            self.publish_status(f'Mission complete - {len(self.trees_detected)} trees found')
            self.state = 'LANDED'
            return
        
        # Get current waypoint
        target_x, target_y, target_z = self.waypoints[self.current_waypoint]
        
        # Calculate error
        error_x = target_x - self.current_pose.position.x
        error_y = target_y - self.current_pose.position.y
        error_z = target_z - self.current_pose.position.z
        
        distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        # Check if waypoint reached
        if distance < self.position_tolerance:
            self.current_waypoint += 1
            self.get_logger().info(
                f'Waypoint {self.current_waypoint}/{len(self.waypoints)} reached'
            )
            
            if self.current_waypoint < len(self.waypoints):
                next_wp = self.waypoints[self.current_waypoint]
                self.get_logger().info(f'Next target: ({next_wp[0]:.1f}, {next_wp[1]:.1f}, {next_wp[2]:.1f})')
            
            return
        
        # Proportional control
        cmd = Twist()
        # NOTE: Parrot drone has non-standard frame convention:
        # - cmd.linear.x = forward/backward (correct)
        # - cmd.linear.y = UP/DOWN (not left/right!)
        # - cmd.linear.z = left/right (not up/down!)
        cmd.linear.x = self.kp_xy * error_x
        cmd.linear.y = self.kp_z * error_z   # SWAPPED: altitude control via Y
        cmd.linear.z = self.kp_xy * error_y  # SWAPPED: lateral control via Z
        
        # Limit speeds
        max_speed = self.cruise_speed
        cmd.linear.x = np.clip(cmd.linear.x, -max_speed, max_speed)
        cmd.linear.y = np.clip(cmd.linear.y, -max_speed*0.5, max_speed*0.5)  # Altitude
        cmd.linear.z = np.clip(cmd.linear.z, -max_speed, max_speed)  # Lateral
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Log progress periodically
        if self.current_waypoint % 5 == 0:
            self.publish_status(
                f'Scouting... WP {self.current_waypoint}/{len(self.waypoints)}, '
                f'Trees: {len(self.trees_detected)}'
            )

    def execute_return(self):
        """Return to home position"""
        # Navigate to origin
        target_x, target_y, target_z = 0.0, 0.0, 0.5
        
        error_x = target_x - self.current_pose.position.x
        error_y = target_y - self.current_pose.position.y
        error_z = target_z - self.current_pose.position.z
        
        distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        if distance < self.position_tolerance:
            self.get_logger().info('Returned to home')
            self.state = 'LANDED'
            self.stop_drone()
            return
        
        # Simple proportional control
        cmd = Twist()
        cmd.linear.x = 0.5 * error_x
        cmd.linear.y = 0.5 * error_y
        cmd.linear.z = 0.3 * error_z
        
        self.cmd_vel_pub.publish(cmd)

    def stop_drone(self):
        """Send stop command"""
        cmd = Twist()  # All zeros
        self.cmd_vel_pub.publish(cmd)

    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {message}')


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousScout()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down autonomous scout...')
        node.stop_drone()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()