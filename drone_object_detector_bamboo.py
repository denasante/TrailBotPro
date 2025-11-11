#!/usr/bin/env python3
"""
Trail Guardian Drone Object Detection Node - Bamboo Cluster Detection
Group 16 - 41068 Robotics Studio 1

Detects colored bamboo clusters from aerial surveillance:
- Red Bamboo (bamboo_thicket_RED)
- Yellow Bamboo (bamboo_thicket_YELLOW)
- Purple Bamboo (bamboo_thicket_PURPLE)

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
        self.get_logger().info('Trail Guardian Bamboo Cluster Detector Initialized')
        self.get_logger().info(f'Detection rate: {self.detection_rate} Hz')
        self.get_logger().info(f'Min area: {self.min_area} px, Max area: {self.max_area} px')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info('Detecting: RED, YELLOW, PURPLE bamboo clusters')
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
        Detect colored bamboo clusters using HSV color space.
        
        Detects:
        - Red Bamboo Clusters
        - Yellow Bamboo Clusters
        - Purple Bamboo Clusters
        """
        detections = []
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Morphological kernel for cleaning up masks
        kernel = np.ones((5, 5), np.uint8)
        
        # ===================================================================
        # COLOR DETECTION DEFINITIONS
        # HSV ranges tuned for the bamboo cluster colors
        # ===================================================================
        color_ranges = {
            'red_bamboo': {
                'lower': np.array([0, 120, 70]),      # Red lower bound
                'upper': np.array([10, 255, 255]),    # Red upper bound
                'color': (0, 0, 255),                 # BGR: Red
                'display_name': 'Red Bamboo'
            },
            'red_bamboo_wrap': {  # Red wraps around in HSV (170-180)
                'lower': np.array([170, 120, 70]),
                'upper': np.array([180, 255, 255]),
                'color': (0, 0, 255),
                'display_name': 'Red Bamboo'
            },
            'yellow_bamboo': {
                'lower': np.array([20, 100, 100]),    # Yellow lower bound
                'upper': np.array([35, 255, 255]),    # Yellow upper bound
                'color': (0, 255, 255),               # BGR: Yellow
                'display_name': 'Yellow Bamboo'
            },
            'purple_bamboo': {
                # Purple: Hue 0.737 = 133° in OpenCV (0-180 scale)
                # Sat 0.981 = 250, Val 1.0 = 255
                'lower': np.array([120, 100, 100]),   # Purple lower bound
                'upper': np.array([150, 255, 255]),   # Purple upper bound
                'color': (255, 0, 255),               # BGR: Magenta/Purple
                'display_name': 'Purple Bamboo'
            }
        }
        
        # ===================================================================
        # DETECT EACH COLOR
        # ===================================================================
        for color_name, color_config in color_ranges.items():
            # Create mask for this color
            mask = cv2.inRange(hsv, color_config['lower'], color_config['upper'])
            
            # For red, we need to combine both ranges (0-10 and 170-180)
            if color_name == 'red_bamboo':
                mask_wrap = cv2.inRange(hsv, 
                                       color_ranges['red_bamboo_wrap']['lower'],
                                       color_ranges['red_bamboo_wrap']['upper'])
                mask = cv2.bitwise_or(mask, mask_wrap)
            elif color_name == 'red_bamboo_wrap':
                continue  # Skip, already handled above
            
            # Clean up mask with morphological operations
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Optional: Additional dilation to merge nearby regions
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                          cv2.CHAIN_APPROX_SIMPLE)
            
            # Process each contour
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by size
                if self.min_area < area < self.max_area:
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate center
                    cx = x + w // 2
                    cy = y + h // 2
                    
                    # Calculate confidence based on:
                    # 1. Size (larger objects = higher confidence)
                    # 2. Color purity (how many pixels match)
                    size_confidence = min(area / 5000.0, 1.0)  # Normalize by expected size
                    
                    # Count color pixels in the bounding box
                    roi_mask = mask[y:y+h, x:x+w]
                    color_pixels = cv2.countNonZero(roi_mask)
                    color_ratio = color_pixels / (w * h) if (w * h) > 0 else 0
                    
                    # Combined confidence
                    confidence = (size_confidence * 0.4 + color_ratio * 0.6)
                    
                    # Only accept if above threshold
                    if confidence >= self.confidence_threshold:
                        detection = {
                            'class': color_name,
                            'confidence': confidence,
                            'bbox': (x, y, w, h),
                            'center': (cx, cy),
                            'area': area,
                            'color_ratio': color_ratio,
                            'display_name': color_config['display_name'],
                            'color': color_config['color']
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
            color = det['color']
            display_name = det['display_name']
            
            # Draw bounding box
            cv2.rectangle(annotated, (x, y), (x+w, y+h), color, 3)
            
            # Draw center crosshair
            cv2.drawMarker(annotated, (cx, cy), color, 
                          cv2.MARKER_CROSS, 30, 3)
            
            # Prepare label
            label = f"{display_name}: {det['confidence']:.2f}"
            (label_w, label_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            
            # Draw label background
            cv2.rectangle(annotated, (x, y - label_h - 10), 
                         (x + label_w + 10, y), color, -1)
            
            # Draw label text
            cv2.putText(annotated, label, (x + 5, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Draw detection info
            info = f"Area: {det['area']:.0f}px | Purity: {det['color_ratio']:.2f}"
            cv2.putText(annotated, info, (x, y + h + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw header with detection summary
        header = f"Bamboo Clusters: {len(detections)} | Total: {self.detection_count}"
        cv2.rectangle(annotated, (5, 5), (500, 40), (0, 0, 0), -1)
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
            f'Published {len(detections)} bamboo detections (total: {self.detection_count})'
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
                    f'{detection["display_name"]} detected at world coords: '
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
        marker.ns = "bamboo_clusters"
        marker.id = self.marker_id + marker_id
        marker.type = Marker.CYLINDER  # Better representation for bamboo clusters
        marker.action = Marker.ADD
        
        marker.pose = pose.pose
        marker.scale.x = 1.0  # Diameter
        marker.scale.y = 1.0
        marker.scale.z = 2.0  # Height
        
        # Use the detection color but convert from BGR to RGB for RViz
        bgr_color = detection['color']
        marker.color.b = bgr_color[0] / 255.0
        marker.color.g = bgr_color[1] / 255.0
        marker.color.r = bgr_color[2] / 255.0
        marker.color.a = 0.7
        
        marker.lifetime = Duration(seconds=15.0).to_msg()
        
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