#!/usr/bin/env python3
"""
Frontier Explorer Node
Detects frontiers between known free space and unknown space in the map,
then publishes navigation goals to explore these frontiers systematically.

Group 16 - Trail Guardian - 41068 Robotics Studio 1
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import numpy as np
from scipy.ndimage import label, binary_dilation
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import math


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # Parameters
        self.declare_parameter('map_topic', '/parrot/map')
        self.declare_parameter('goal_topic', '/parrot/goal_pose')
        self.declare_parameter('min_frontier_size', 15)  # cells
        self.declare_parameter('exploration_radius', 15.0)  # meters
        self.declare_parameter('goal_timeout', 30.0)  # seconds
        self.declare_parameter('robot_frame', 'parrot/base_link')
        self.declare_parameter('map_frame', 'parrot/map')
        
        self.map_topic = self.get_parameter('map_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # State
        self.map_data = None
        self.current_goal = None
        self.goal_start_time = None
        self.robot_position = None
        self.explored_area_percent = 0.0
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            self.map_topic, 
            self.map_callback, 
            10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.frontier_markers_pub = self.create_publisher(
            MarkerArray, 
            '/frontier_markers', 
            10
        )
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer for goal management
        self.timer = self.create_timer(2.0, self.check_goal_status)
        
        self.get_logger().info('Frontier Explorer initialized')
        self.get_logger().info(f'Listening to map: {self.map_topic}')
        self.get_logger().info(f'Publishing goals to: {self.goal_topic}')
    
    def get_robot_position(self):
        """Get robot's current position from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
        except Exception as e:
            self.get_logger().warn(f'Could not get robot position: {e}')
            return None
    
    def map_callback(self, msg):
        """Process incoming map and detect frontiers"""
        self.map_data = msg
        self.robot_position = self.get_robot_position()
        
        # Calculate exploration progress
        self.calculate_exploration_progress(msg)
        
        # Only look for new frontiers if we don't have an active goal
        if self.current_goal is None:
            frontiers = self.detect_frontiers(msg)
            
            if frontiers:
                self.publish_frontier_markers(frontiers)
                best_frontier = self.select_best_frontier(frontiers)
                
                if best_frontier:
                    self.publish_goal(best_frontier)
            else:
                self.get_logger().info(
                    f'No more frontiers found! Exploration {self.explored_area_percent:.1f}% complete'
                )
    
    def calculate_exploration_progress(self, map_msg):
        """Calculate percentage of map that has been explored"""
        data = np.array(map_msg.data)
        unknown = np.sum(data == -1)
        total = len(data)
        known = total - unknown
        self.explored_area_percent = (known / total) * 100.0
    
    def detect_frontiers(self, map_msg):
        """
        Find boundaries between known free space and unknown space.
        Returns list of frontier regions with centroids and sizes.
        """
        width = map_msg.info.width
        height = map_msg.info.height
        data = np.array(map_msg.data).reshape((height, width))
        
        # Identify different regions
        # Free space = 0, Unknown = -1, Occupied > 50
        free_space = (data == 0)
        unknown = (data == -1)
        
        # Dilate free space by 1 cell to find adjacent unknown cells
        frontier_mask = binary_dilation(free_space, iterations=1) & unknown
        
        # Find connected frontier regions
        labeled, num_features = label(frontier_mask)
        
        frontiers = []
        for region_id in range(1, num_features + 1):
            region = (labeled == region_id)
            size = np.sum(region)
            
            # Filter by minimum size
            if size >= self.min_frontier_size:
                # Get centroid in grid coordinates
                y_coords, x_coords = np.where(region)
                centroid_x = np.mean(x_coords)
                centroid_y = np.mean(y_coords)
                
                # Convert to world coordinates
                world_x = (centroid_x * map_msg.info.resolution + 
                          map_msg.info.origin.position.x)
                world_y = (centroid_y * map_msg.info.resolution + 
                          map_msg.info.origin.position.y)
                
                # Calculate distance from robot
                if self.robot_position:
                    distance = math.sqrt(
                        (world_x - self.robot_position[0])**2 + 
                        (world_y - self.robot_position[1])**2
                    )
                else:
                    distance = float('inf')
                
                frontiers.append({
                    'position': (world_x, world_y),
                    'size': size,
                    'distance': distance
                })
        
        self.get_logger().info(f'Detected {len(frontiers)} frontiers')
        return frontiers
    
    def select_best_frontier(self, frontiers):
        """
        Select the best frontier based on:
        1. Within exploration radius
        2. Size (information gain)
        3. Distance (prefer closer frontiers)
        """
        if not frontiers:
            return None
        
        # Filter by exploration radius
        valid_frontiers = [
            f for f in frontiers 
            if f['distance'] <= self.exploration_radius
        ]
        
        if not valid_frontiers:
            # If no frontiers in radius, take the closest one
            valid_frontiers = [min(frontiers, key=lambda f: f['distance'])]
        
        # Score: balance size and distance
        # Higher score is better
        for frontier in valid_frontiers:
            # Normalize size and distance
            size_score = frontier['size'] / 100.0  # Arbitrary scaling
            distance_score = 1.0 / (1.0 + frontier['distance'])
            
            # Weight: prefer information gain but consider distance
            frontier['score'] = 0.6 * size_score + 0.4 * distance_score
        
        # Select highest scoring frontier
        best = max(valid_frontiers, key=lambda f: f['score'])
        
        self.get_logger().info(
            f'Selected frontier: size={best["size"]}, '
            f'dist={best["distance"]:.2f}m, score={best["score"]:.3f}'
        )
        
        return best
    
    def publish_goal(self, frontier):
        """Publish a navigation goal for the selected frontier"""
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = frontier['position'][0]
        goal.pose.position.y = frontier['position'][1]
        goal.pose.position.z = 2.0  # Hover height for drone
        
        # Orientation facing forward (can be improved to face frontier)
        goal.pose.orientation.w = 1.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        
        self.goal_pub.publish(goal)
        self.current_goal = goal
        self.goal_start_time = self.get_clock().now()
        
        self.get_logger().info(
            f'Published goal: ({goal.pose.position.x:.2f}, '
            f'{goal.pose.position.y:.2f}, {goal.pose.position.z:.2f})'
        )
    
    def check_goal_status(self):
        """Check if current goal should be abandoned and new one selected"""
        if self.current_goal is None:
            return
        
        # Check timeout
        if self.goal_start_time:
            elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
            
            if elapsed > self.goal_timeout:
                self.get_logger().warn(
                    f'Goal timeout ({elapsed:.1f}s), selecting new frontier'
                )
                self.current_goal = None
                self.goal_start_time = None
                return
        
        # Check if goal reached (within 1.0m)
        if self.robot_position:
            goal_pos = (
                self.current_goal.pose.position.x,
                self.current_goal.pose.position.y
            )
            distance = math.sqrt(
                (goal_pos[0] - self.robot_position[0])**2 + 
                (goal_pos[1] - self.robot_position[1])**2
            )
            
            if distance < 1.0:
                self.get_logger().info(f'Goal reached! Distance: {distance:.2f}m')
                self.current_goal = None
                self.goal_start_time = None
    
    def publish_frontier_markers(self, frontiers):
        """Visualize frontiers in RViz"""
        marker_array = MarkerArray()
        
        for i, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = frontier['position'][0]
            marker.pose.position.y = frontier['position'][1]
            marker.pose.position.z = 1.0
            marker.pose.orientation.w = 1.0
            
            # Size based on frontier size
            scale = min(frontier['size'] / 50.0, 2.0)
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            
            # Color: green for valid, yellow for distant
            if frontier['distance'] <= self.exploration_radius:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)
            else:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)
            
            marker.lifetime.sec = 5
            marker_array.markers.append(marker)
        
        self.frontier_markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()