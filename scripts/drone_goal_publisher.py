#!/usr/bin/env python3
"""
Simple Nav2 Goal Publisher for Drone
Group 16 - Trail Guardian - 41068 Robotics Studio 1

Publishes navigation goals to test Nav2 autonomous navigation.
Can be used to send the drone to specific waypoints.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import sys


class DroneGoalPublisher(Node):
    def __init__(self):
        super().__init__('drone_goal_publisher')
        
        # Parameters
        self.declare_parameter('namespace', 'parrot')
        self.namespace = self.get_parameter('namespace').value
        
        # Action client for Nav2
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            f'/{self.namespace}/navigate_to_pose'
        )
        
        self.get_logger().info(f'Drone Goal Publisher initialized for namespace: {self.namespace}')
        self.get_logger().info('Waiting for Nav2 action server...')
        
        # Wait for action server
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Nav2 action server connected!')
    
    def send_goal(self, x, y, z=2.0, yaw=0.0):
        """
        Send a navigation goal to Nav2
        
        Args:
            x: X position in meters
            y: Y position in meters
            z: Z position (altitude) in meters (default: 2.0)
            yaw: Orientation in radians (default: 0.0)
        """
        goal_msg = NavigateToPose.Goal()
        
        # Create pose
        goal_msg.pose.header.frame_id = f'{self.namespace}/map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        
        # Convert yaw to quaternion (simplified for z-axis rotation)
        import math
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        
        self.get_logger().info(
            f'Sending goal: ({x:.2f}, {y:.2f}, {z:.2f}) yaw={yaw:.2f} rad'
        )
        
        # Send goal
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return
        
        self.get_logger().info('Goal accepted by Nav2!')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle goal result"""
        result = future.result().result
        self.get_logger().info(f'Goal completed! Result: {result}')
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}m',
            throttle_duration_sec=2.0  # Log every 2 seconds
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = DroneGoalPublisher()
    
    # Check if goal coordinates provided via command line
    if len(sys.argv) >= 3:
        try:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            z = float(sys.argv[3]) if len(sys.argv) >= 4 else 2.0
            yaw = float(sys.argv[4]) if len(sys.argv) >= 5 else 0.0
            
            node.send_goal(x, y, z, yaw)
            
            # Spin to handle callbacks
            rclpy.spin(node)
            
        except ValueError:
            node.get_logger().error('Invalid coordinates. Usage: x y [z] [yaw]')
    else:
        # Interactive mode
        node.get_logger().info('')
        node.get_logger().info('='*60)
        node.get_logger().info('INTERACTIVE MODE - Send navigation goals to the drone')
        node.get_logger().info('='*60)
        node.get_logger().info('')
        node.get_logger().info('Usage: Enter coordinates as: x y [z] [yaw]')
        node.get_logger().info('Example: 5.0 3.0 2.5 1.57')
        node.get_logger().info('Type "quit" to exit')
        node.get_logger().info('')
        
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        
        try:
            while rclpy.ok():
                # Get user input
                try:
                    user_input = input('Enter goal (x y [z] [yaw]): ').strip()
                    
                    if user_input.lower() in ['quit', 'exit', 'q']:
                        break
                    
                    # Parse coordinates
                    coords = user_input.split()
                    if len(coords) < 2:
                        node.get_logger().warn('Need at least x and y coordinates')
                        continue
                    
                    x = float(coords[0])
                    y = float(coords[1])
                    z = float(coords[2]) if len(coords) >= 3 else 2.0
                    yaw = float(coords[3]) if len(coords) >= 4 else 0.0
                    
                    node.send_goal(x, y, z, yaw)
                    
                    # Process callbacks
                    executor.spin_once(timeout_sec=0.1)
                    
                except ValueError:
                    node.get_logger().error('Invalid input. Use numbers for coordinates.')
                except EOFError:
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            node.get_logger().info('Shutting down...')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()