#!/usr/bin/env python3
"""
Trail Guardian Drone Object Detection Node
Group 16 - 41068 Robotics Studio 1

Detects trail hazards from aerial surveillance:
- Fallen branches
- Erosion patches
- Trail obstacles
- Ground anomalies

Publishes both 2D (pixel) and 3D (world) coordinates for ground robot response.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


class DroneObjectDetector(Node):
    def __init__(self):
        super().__init__('drone_object_detector')
        
        # =====================================================================
        # PARAMETERS
        # =====================================================================
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('min_detection_area', 500)  # pixels
        self.declare_parameter('max_detection_area', 50000)  # pixels
        self.declare_parameter('visualization', True)
        self.declare_parameter('detection_rate', 2.0)  # Hz
        
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.min_area = self.get_parameter('min_detection_area').value
        self.max_area = self.get_parameter('max_detection_area').value
        self.enable_viz = self.get_parameter('visualization').value
        self.detection_rate = self.get_parameter('detection_rate').value
        
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
        # Detection messages (for mission coordinator)
        self.detection_pub = self.create_publisher(
            String,
            '/trail_guardian/detections',
            10
        )
        
        # 3D anomaly locations (for Husky navigation)
        self.anomaly_location_pub = self.create_publisher(
            PoseStamped,
            '/trail_guardian/anomaly_location',
            10
        )
        
        # Annotated image (for visualization)
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/parrot/camera/annotated',
            10
        )
        
        # Visualization markers (for RViz)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/trail_guardian/detection_markers',
            10
        )
        
        # =====================================================================
        # STATE
        # =====================================================================
        self.latest_depth = None
        self.detection_count = 0
        self.marker_id = 0
        self.last_detection_time = self.get_clock().now()
        
        # Camera intrinsics (adjust based on your camera model)
        self.fx = 554.25  # Focal length X
        self.fy = 554.25  # Focal length Y
        self.ppx = 360.0  # Principal point X (720/2)
        self.ppy = 240.0  # Principal point Y (480/2)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Trail Guardian Drone Object Detector Initialized')
        self.get_logger().info(f'Detection rate: {self.detection_rate} Hz')
        self.get_logger().info(f'Min area: {self.min_area} px, Max area: {self.max_area} px')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info('=' * 60)

    def depth_callback(self, msg):
        """Store latest depth image for 3D localization"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    def image_callback(self, msg):
        """Process incoming camera images for object detection"""
        
        # Rate limiting
        current_time = self.get_clock().now()
        time_since_last = (current_time - self.last_detection_time).nanoseconds / 1e9
        if time_since_last < (1.0 / self.detection_rate):
            return
        
        self.last_detection_time = current_time
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run detection
            detections = self.detect_objects(cv_image)
            
            # Process and publish results
            if detections:
                annotated = self.draw_detections(cv_image, detections)
                self.publish_detections(detections, msg.header)
                
                if self.enable_viz:
                    # Publish annotated image
                    annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                    annotated_msg.header = msg.header
                    self.annotated_image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def detect_objects(self, image):
        """
        Detect trail hazards using color and shape analysis.
        
        Detection strategy:
        1. Brown objects (fallen branches, logs)
        2. Dark patches (shadows, holes, erosion)
        3. Unusual shapes (obstacles)
        
        For production: Replace with YOLO or custom trained model
        """
        detections = []
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # ===================================================================
        # DETECTION METHOD 1: Brown objects (branches, logs)
        # ===================================================================
        lower_brown = np.array([10, 40, 20])
        upper_brown = np.array([30, 255, 200])
        brown_mask = cv2.inRange(hsv, lower_brown, upper_brown)
        
        # Clean up mask
        kernel = np.ones((5, 5), np.uint8)
        brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_CLOSE, kernel)
        brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(brown_mask, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by size
            if self.min_area < area < self.max_area:
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate aspect ratio
                aspect_ratio = float(w) / h if h > 0 else 0
                
                # Elongated objects more likely to be branches
                confidence = 0.7 if 2.0 < aspect_ratio < 10.0 else 0.5
                
                detection = {
                    'class': 'fallen_branch',
                    'confidence': confidence,
                    'bbox': (x, y, w, h),
                    'center': (x + w // 2, y + h // 2),
                    'area': area,
                    'aspect_ratio': aspect_ratio
                }
                detections.append(detection)
        
        # ===================================================================
        # DETECTION METHOD 2: Dark patches (erosion, holes)
        # ===================================================================
        # Detect very dark areas that could be erosion or holes
        _, dark_mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        
        # Clean up
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if self.min_area < area < self.max_area:
                x, y, w, h = cv2.boundingRect(contour)
                
                # Circular patches more likely to be holes/erosion
                circularity = 4 * np.pi * area / (cv2.arcLength(contour, True) ** 2) \
                    if cv2.arcLength(contour, True) > 0 else 0
                
                confidence = 0.65 if circularity > 0.7 else 0.45
                
                detection = {
                    'class': 'erosion_patch',
                    'confidence': confidence,
                    'bbox': (x, y, w, h),
                    'center': (x + w // 2, y + h // 2),
                    'area': area,
                    'circularity': circularity
                }
                detections.append(detection)
        
        # ===================================================================
        # FILTER & DEDUPLICATE
        # ===================================================================
        # Remove overlapping detections (keep higher confidence)
        filtered_detections = []
        for det in detections:
            is_duplicate = False
            for existing in filtered_detections:
                if self.is_overlapping(det['bbox'], existing['bbox']):
                    if det['confidence'] > existing['confidence']:
                        filtered_detections.remove(existing)
                    else:
                        is_duplicate = True
                    break
            
            if not is_duplicate:
                filtered_detections.append(det)
        
        return filtered_detections

    def is_overlapping(self, bbox1, bbox2, threshold=0.5):
        """Check if two bounding boxes overlap significantly"""
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        
        # Calculate intersection
        x_overlap = max(0, min(x1 + w1, x2 + w2) - max(x1, x2))
        y_overlap = max(0, min(y1 + h1, y2 + h2) - max(y1, y2))
        intersection = x_overlap * y_overlap
        
        # Calculate union
        area1 = w1 * h1
        area2 = w2 * h2
        union = area1 + area2 - intersection
        
        # IoU (Intersection over Union)
        iou = intersection / union if union > 0 else 0
        return iou > threshold

    def draw_detections(self, image, detections):
        """Draw bounding boxes and labels on image"""
        annotated = image.copy()
        
        for det in detections:
            x, y, w, h = det['bbox']
            cx, cy = det['center']
            
            # Color based on class
            if det['class'] == 'fallen_branch':
                color = (0, 165, 255)  # Orange
                display_name = "Branch"
            elif det['class'] == 'erosion_patch':
                color = (0, 0, 255)  # Red
                display_name = "Erosion"
            else:
                color = (0, 255, 0)  # Green
                display_name = det['class']
            
            # Draw bounding box
            cv2.rectangle(annotated, (x, y), (x+w, y+h), color, 2)
            
            # Draw center crosshair
            cv2.drawMarker(annotated, (cx, cy), color, 
                          cv2.MARKER_CROSS, 20, 2)
            
            # Prepare label
            label = f"{display_name}: {det['confidence']:.2f}"
            (label_w, label_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
            )
            
            # Draw label background
            cv2.rectangle(annotated, (x, y - label_h - 10), 
                         (x + label_w, y), color, -1)
            
            # Draw label text
            cv2.putText(annotated, label, (x, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Draw detection info
            info = f"Area: {det['area']:.0f}px"
            cv2.putText(annotated, info, (x, y + h + 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Draw header
        header = f"Detections: {len(detections)} | Total: {self.detection_count}"
        cv2.putText(annotated, header, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return annotated

    def publish_detections(self, detections, header):
        """Publish detection results to all topics"""
        
        markers = MarkerArray()
        
        for i, det in enumerate(detections):
            # Text detection message
            detection_msg = String()
            detection_msg.data = (f"{det['class']},{det['confidence']:.2f},"
                                 f"{det['center'][0]},{det['center'][1]},{det['area']}")
            self.detection_pub.publish(detection_msg)
            
            # 3D location with TF transform
            if self.latest_depth is not None:
                pose_3d = self.calculate_3d_position(det, header)
                if pose_3d is not None:
                    self.anomaly_location_pub.publish(pose_3d)
                    
                    # Create visualization marker
                    if self.enable_viz:
                        marker = self.create_marker(pose_3d, det, i)
                        markers.markers.append(marker)
            
            self.detection_count += 1
        
        # Publish all markers
        if self.enable_viz and markers.markers:
            self.marker_pub.publish(markers)
        
        self.get_logger().info(
            f'Published {len(detections)} detections (total: {self.detection_count})'
        )

    def calculate_3d_position(self, detection, header):
        """Calculate 3D world coordinates of detected object"""
        cx, cy = detection['center']
        
        # Get depth at detection center
        if (0 <= cy < self.latest_depth.shape[0] and 
            0 <= cx < self.latest_depth.shape[1]):
            
            depth = self.latest_depth[cy, cx]
            
            # Check for valid depth
            if np.isnan(depth) or depth <= 0 or depth > 100:
                self.get_logger().warn(f'Invalid depth at ({cx}, {cy}): {depth}')
                return None
            
            # Convert pixel coordinates to 3D camera frame
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
            
            # Try to transform to world frame (parrot/odom)
            try:
                # Wait for transform (with timeout)
                transform = self.tf_buffer.lookup_transform(
                    'parrot/base_link',
                    pose_msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5)
                )
                
                # Transform point
                # Simple implementation - for production use tf2_geometry_msgs
                world_pose = PoseStamped()
                world_pose.header.stamp = self.get_clock().now().to_msg()
                world_pose.header.frame_id = 'parrot/base_link'
                world_pose.pose.position.x = (
                    transform.transform.translation.x + x
                )
                world_pose.pose.position.y = (
                    transform.transform.translation.y + y
                )
                world_pose.pose.position.z = (
                    transform.transform.translation.z + z
                )
                world_pose.pose.orientation.w = 1.0
                
                self.get_logger().info(
                    f'{detection["class"]} detected at world coords: '
                    f'x={world_pose.pose.position.x:.2f}, '
                    f'y={world_pose.pose.position.y:.2f}, '
                    f'z={world_pose.pose.position.z:.2f}'
                )
                
                return world_pose
                
            except Exception as e:
                self.get_logger().warn(f'TF transform failed: {e}')
                # Return camera frame pose if transform fails
                return pose_msg
        
        return None

    def create_marker(self, pose, detection, marker_id):
        """Create RViz visualization marker for detection"""
        marker = Marker()
        marker.header = pose.header
        marker.ns = "trail_hazards"
        marker.id = self.marker_id + marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose = pose.pose
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        # Color based on class
        if detection['class'] == 'fallen_branch':
            marker.color.r = 1.0
            marker.color.g = 0.65
            marker.color.b = 0.0
        elif detection['class'] == 'erosion_patch':
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        
        marker.color.a = 0.8
        marker.lifetime = Duration(seconds=10.0).to_msg()
        
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = DroneObjectDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down drone object detector...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()