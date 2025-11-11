#!/usr/bin/env python3
"""
Trail Guardian Enhanced Tree Detection Node
Group 16 - 41068 Robotics Studio 1

Specifically detects Oak and Pine trees in the simple_trees world.
Uses multiple visual cues:
- Brown/green color detection for tree trunks and canopy
- Vertical shape detection (trees are tall and narrow)
- Size filtering (trees are larger than grass patches)
- Depth validation (trees are substantial 3D objects)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


class TreeDetector(Node):
    def __init__(self):
        super().__init__('tree_detector')
        
        # =====================================================================
        # PARAMETERS
        # =====================================================================
        self.declare_parameter('detection_rate', 1.0)  # Slower for more thorough processing
        self.declare_parameter('min_tree_area', 3000)  # Trees are large
        self.declare_parameter('max_tree_area', 150000)
        self.declare_parameter('min_tree_height_ratio', 1.5)  # Trees are taller than wide
        self.declare_parameter('visualization', True)
        
        self.detection_rate = self.get_parameter('detection_rate').value
        self.min_area = self.get_parameter('min_tree_area').value
        self.max_area = self.get_parameter('max_tree_area').value
        self.min_height_ratio = self.get_parameter('min_tree_height_ratio').value
        self.enable_viz = self.get_parameter('visualization').value
        
        # =====================================================================
        # CV BRIDGE & TF
        # =====================================================================
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # =====================================================================
        # SUBSCRIBERS
        # =====================================================================
        self.image_sub = self.create_subscription(
            Image,
            '/parrot/camera/image',
            self.image_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/parrot/camera/depth_image',
            self.depth_callback,
            10
        )
        
        # =====================================================================
        # PUBLISHERS
        # =====================================================================
        self.tree_detection_pub = self.create_publisher(
            String,
            '/trail_guardian/tree_detections',
            10
        )
        
        self.tree_location_pub = self.create_publisher(
            PoseStamped,
            '/trail_guardian/tree_location',
            10
        )
        
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/parrot/camera/tree_annotated',
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/trail_guardian/tree_markers',
            10
        )
        
        # =====================================================================
        # STATE
        # =====================================================================
        self.latest_depth = None
        self.tree_count = 0
        self.marker_id = 0
        self.last_detection_time = self.get_clock().now()
        
        # Camera intrinsics (matching your camera)
        self.fx = 554.25
        self.fy = 554.25
        self.ppx = 360.0
        self.ppy = 240.0
        
        # Known tree locations for validation (from world file)
        self.known_trees = {
            'oak': {'world_pos': (0.0, 3.0, 0.0), 'detected': False},
            'pine': {'world_pos': (5.0, 0.0, 0.0), 'detected': False}
        }
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Trail Guardian Tree Detector Initialized')
        self.get_logger().info(f'Detection rate: {self.detection_rate} Hz')
        self.get_logger().info(f'Min tree area: {self.min_area} px')
        self.get_logger().info(f'World bounds: 15m x 15m (±7.5m from origin)')
        self.get_logger().info(f'Known trees: Oak at (0, 3), Pine at (5, 0)')
        self.get_logger().info('=' * 60)

    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    def image_callback(self, msg):
        """Process incoming camera images for tree detection"""
        
        # Rate limiting
        current_time = self.get_clock().now()
        time_since_last = (current_time - self.last_detection_time).nanoseconds / 1e9
        if time_since_last < (1.0 / self.detection_rate):
            return
        
        self.last_detection_time = current_time
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect trees
            tree_detections = self.detect_trees(cv_image)
            
            if tree_detections:
                annotated = self.draw_detections(cv_image, tree_detections)
                self.publish_detections(tree_detections, msg.header)
                
                if self.enable_viz:
                    annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                    annotated_msg.header = msg.header
                    self.annotated_image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def detect_trees(self, image):
        """
        Multi-stage tree detection:
        1. Color-based detection (brown trunks, green canopy)
        2. Shape analysis (vertical objects)
        3. Size filtering (large objects)
        4. Depth validation (substantial 3D presence)
        """
        detections = []
        height, width = image.shape[:2]
        
        # Convert color spaces
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # ===================================================================
        # STAGE 1: BROWN TRUNK DETECTION
        # ===================================================================
        # Brown range for tree trunks (wider range than before)
        lower_brown = np.array([5, 30, 20])
        upper_brown = np.array([30, 255, 180])
        brown_mask = cv2.inRange(hsv, lower_brown, upper_brown)
        
        # Clean up
        kernel_large = np.ones((7, 7), np.uint8)
        brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_CLOSE, kernel_large)
        brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_OPEN, kernel_large)
        
        # ===================================================================
        # STAGE 2: GREEN CANOPY DETECTION
        # ===================================================================
        # Green range for tree canopy
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Clean up
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel_large)
        
        # ===================================================================
        # STAGE 3: COMBINE MASKS (tree = brown trunk OR green canopy)
        # ===================================================================
        combined_mask = cv2.bitwise_or(brown_mask, green_mask)
        
        # Further cleanup
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel_large)
        
        # Find contours
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by size
            if not (self.min_area < area < self.max_area):
                continue
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # ===================================================================
            # STAGE 4: SHAPE ANALYSIS
            # ===================================================================
            # Trees are typically taller than they are wide
            aspect_ratio = float(h) / w if w > 0 else 0
            
            # Skip horizontal objects (not trees)
            if aspect_ratio < self.min_height_ratio:
                continue
            
            # Calculate center
            cx, cy = x + w // 2, y + h // 2
            
            # ===================================================================
            # STAGE 5: POSITION ANALYSIS
            # ===================================================================
            # Trees in lower portion of image are more likely (looking down)
            y_position_ratio = cy / height
            
            # Confidence based on shape and position
            confidence = 0.5  # Base confidence
            
            # Boost confidence for tall, narrow objects
            if aspect_ratio > 2.0:
                confidence += 0.2
            
            # Boost confidence for objects in lower half of frame
            if y_position_ratio > 0.4:
                confidence += 0.15
            
            # Boost confidence for large objects
            if area > 10000:
                confidence += 0.15
            
            confidence = min(confidence, 0.95)
            
            # ===================================================================
            # STAGE 6: DEPTH VALIDATION
            # ===================================================================
            depth_valid = False
            avg_depth = 0.0
            
            if self.latest_depth is not None:
                # Sample depth in center region
                sample_y1 = max(0, cy - h//4)
                sample_y2 = min(height, cy + h//4)
                sample_x1 = max(0, cx - w//4)
                sample_x2 = min(width, cx + w//4)
                
                depth_region = self.latest_depth[sample_y1:sample_y2, sample_x1:sample_x2]
                valid_depths = depth_region[(~np.isnan(depth_region)) & 
                                            (depth_region > 0) & 
                                            (depth_region < 50)]
                
                if len(valid_depths) > 0:
                    avg_depth = float(np.median(valid_depths))
                    depth_valid = True
            
            # Skip if no valid depth
            if not depth_valid:
                continue
            
            # Classify tree type based on visual characteristics
            tree_type = self.classify_tree_type(image, x, y, w, h, brown_mask, green_mask)
            
            detection = {
                'class': tree_type,
                'confidence': confidence,
                'bbox': (x, y, w, h),
                'center': (cx, cy),
                'area': area,
                'aspect_ratio': aspect_ratio,
                'depth': avg_depth
            }
            detections.append(detection)
        
        return detections

    def classify_tree_type(self, image, x, y, w, h, brown_mask, green_mask):
        """Classify detected tree as Oak or Pine based on visual features"""
        
        # Extract ROI
        roi_brown = brown_mask[y:y+h, x:x+w]
        roi_green = green_mask[y:y+h, x:x+w]
        
        brown_pixels = np.sum(roi_brown > 0)
        green_pixels = np.sum(roi_green > 0)
        total_pixels = w * h
        
        brown_ratio = brown_pixels / total_pixels if total_pixels > 0 else 0
        green_ratio = green_pixels / total_pixels if total_pixels > 0 else 0
        
        # Pine trees: More greenish canopy, narrow
        # Oak trees: More brownish, wider canopy
        aspect_ratio = float(h) / w if w > 0 else 0
        
        if green_ratio > 0.3 and aspect_ratio > 2.5:
            return 'pine_tree'
        elif brown_ratio > 0.15:
            return 'oak_tree'
        else:
            return 'tree_unknown'

    def draw_detections(self, image, detections):
        """Draw bounding boxes and labels on image"""
        annotated = image.copy()
        
        for det in detections:
            x, y, w, h = det['bbox']
            cx, cy = det['center']
            
            # Color based on tree type
            if 'pine' in det['class']:
                color = (0, 255, 0)  # Green for pine
                display_name = "Pine Tree"
            elif 'oak' in det['class']:
                color = (0, 165, 255)  # Orange for oak
                display_name = "Oak Tree"
            else:
                color = (255, 0, 255)  # Magenta for unknown
                display_name = "Tree"
            
            # Draw bounding box
            cv2.rectangle(annotated, (x, y), (x+w, y+h), color, 3)
            
            # Draw center crosshair
            cv2.drawMarker(annotated, (cx, cy), color,
                          cv2.MARKER_CROSS, 30, 3)
            
            # Prepare label
            label = f"{display_name} {det['confidence']:.0%}"
            (label_w, label_h), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2
            )
            
            # Draw label background
            cv2.rectangle(annotated, (x, y - label_h - 15),
                         (x + label_w + 10, y), color, -1)
            
            # Draw label text
            cv2.putText(annotated, label, (x + 5, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Draw detailed info
            depth_text = f"Depth: {det['depth']:.1f}m"
            cv2.putText(annotated, depth_text, (x, y + h + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            ratio_text = f"H/W: {det['aspect_ratio']:.1f}"
            cv2.putText(annotated, ratio_text, (x, y + h + 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw header
        header = f"Trees Detected: {len(detections)} | Total: {self.tree_count}"
        cv2.putText(annotated, header, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        return annotated

    def publish_detections(self, detections, header):
        """Publish tree detection results"""
        
        for det in detections:
            # Text detection message
            detection_msg = String()
            detection_msg.data = (f"{det['class']},{det['confidence']:.2f},"
                                 f"{det['center'][0]},{det['center'][1]},"
                                 f"{det['depth']:.2f}")
            self.tree_detection_pub.publish(detection_msg)
            
            # 3D location
            pose_3d = self.calculate_3d_position(det, header)
            if pose_3d is not None:
                self.tree_location_pub.publish(pose_3d)
            
            self.tree_count += 1
        
        self.get_logger().info(
            f'Detected {len(detections)} trees (total: {self.tree_count})'
        )

    def calculate_3d_position(self, detection, header):
        """Calculate 3D position of detected tree"""
        cx, cy = detection['center']
        depth = detection['depth']
        
        if depth <= 0 or np.isnan(depth):
            return None
        
        # Convert pixel to 3D camera frame
        z = float(depth)
        x = float((cx - self.ppx) * z / self.fx)
        y = float((cy - self.ppy) * z / self.fy)
        
        # Create pose in camera optical frame
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = 'parrot/camera_rgb_optical_frame'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0
        
        # Transform to base_link
        try:
            transform = self.tf_buffer.lookup_transform(
                'parrot/base_link',
                pose_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            
            # Simple transform (for production use tf2_geometry_msgs)
            world_pose = PoseStamped()
            world_pose.header.stamp = self.get_clock().now().to_msg()
            world_pose.header.frame_id = 'parrot/base_link'
            world_pose.pose.position.x = transform.transform.translation.x + x
            world_pose.pose.position.y = transform.transform.translation.y + y
            world_pose.pose.position.z = transform.transform.translation.z + z
            world_pose.pose.orientation.w = 1.0
            
            self.get_logger().info(
                f'{detection["class"]} at base_link: '
                f'x={world_pose.pose.position.x:.2f}, '
                f'y={world_pose.pose.position.y:.2f}, '
                f'depth={depth:.2f}m'
            )
            
            return world_pose
            
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return pose_msg


def main(args=None):
    rclpy.init(args=args)
    node = TreeDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down tree detector...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()