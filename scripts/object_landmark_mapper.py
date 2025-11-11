#!/usr/bin/env python3
"""
Object Landmark Mapper Node
Detects unique objects using RGB-D camera, deprojects to 3D using depth,
deduplicates landmarks, and persists them to disk in GeoJSON and CSV formats.

Group 16 - Trail Guardian - 41068 Robotics Studio 1
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import json
import csv
from datetime import datetime
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import os


class ObjectLandmarkMapper(Node):
    def __init__(self):
        super().__init__('object_landmark_mapper')
        
        # Parameters
        self.declare_parameter('rgb_topic', '/parrot/camera/image')
        self.declare_parameter('depth_topic', '/parrot/camera/depth_image')
        self.declare_parameter('camera_info_topic', '/parrot/camera/camera_info')
        self.declare_parameter('output_dir', '/home/claude')
        self.declare_parameter('dedup_radius', 0.75)  # meters
        self.declare_parameter('min_confidence', 0.6)
        self.declare_parameter('map_frame', 'parrot/map')
        self.declare_parameter('camera_frame', 'parrot/camera_link')
        
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.dedup_radius = self.get_parameter('dedup_radius').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # State
        self.bridge = CvBridge()
        self.landmarks = []
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        self.processing = False
        
        # Camera intrinsics (defaults, will be updated from camera_info)
        self.fx = 500.0
        self.fy = 500.0
        self.cx = 360.0
        self.cy = 240.0
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, 
            self.rgb_topic, 
            self.rgb_callback, 
            10
        )
        self.depth_sub = self.create_subscription(
            Image, 
            self.depth_topic, 
            self.depth_callback, 
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/unique_objects_viz', 
            10
        )
        
        # Timer for periodic processing
        self.timer = self.create_timer(1.0, self.process_frame)
        
        self.get_logger().info('Object Landmark Mapper initialized')
        self.get_logger().info(f'Output directory: {self.output_dir}')
    
    def camera_info_callback(self, msg):
        """Update camera intrinsics from camera_info"""
        if self.camera_info is None:
            self.camera_info = msg
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f'Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, '
                f'cx={self.cx:.1f}, cy={self.cy:.1f}'
            )
    
    def rgb_callback(self, msg):
        """Store latest RGB image"""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'RGB conversion error: {e}')
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            # Depth is usually in 32FC1 format (meters)
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def process_frame(self):
        """Process RGB-D frame to detect and map objects"""
        if self.processing:
            return
        
        if self.latest_rgb is None or self.latest_depth is None:
            return
        
        self.processing = True
        
        try:
            # Detect objects in RGB image
            detections = self.detect_objects(self.latest_rgb)
            
            # Process each detection
            for det in detections:
                if det['confidence'] < self.min_confidence:
                    continue
                
                # Deproject to 3D using depth
                point_3d_camera = self.deproject_to_3d(det, self.latest_depth)
                
                if point_3d_camera is None:
                    continue
                
                # Transform to map frame
                point_3d_map = self.transform_to_map(point_3d_camera)
                
                if point_3d_map is None:
                    continue
                
                # Check for duplicates
                if not self.is_duplicate(point_3d_map, det['class']):
                    landmark = {
                        'x': point_3d_map['x'],
                        'y': point_3d_map['y'],
                        'z': point_3d_map['z'],
                        'class': det['class'],
                        'confidence': det['confidence'],
                        'timestamp': datetime.now().isoformat()
                    }
                    
                    self.landmarks.append(landmark)
                    self.get_logger().info(
                        f'New landmark: {det["class"]} at '
                        f'({landmark["x"]:.2f}, {landmark["y"]:.2f}, {landmark["z"]:.2f})'
                    )
                    
                    # Save and visualize
                    self.save_landmarks()
                    self.publish_markers()
        
        finally:
            self.processing = False
    
    def detect_objects(self, image):
        """
        Detect objects in RGB image using color-based detection.
        Returns list of detections with bbox, class, and confidence.
        
        This is a simplified detector - replace with your existing tree detection
        or a proper ML model like YOLOv8.
        """
        detections = []
        
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for tree types (adjust based on your world)
        tree_types = {
            'oak': {
                'lower': np.array([15, 50, 50]),   # Brown-ish
                'upper': np.array([30, 255, 200])
            },
            'pine': {
                'lower': np.array([35, 40, 40]),   # Green-ish
                'upper': np.array([85, 255, 200])
            }
        }
        
        for tree_class, color_range in tree_types.items():
            # Create mask
            mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
            
            # Morphological operations to clean up mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(
                mask, 
                cv2.RETR_EXTERNAL, 
                cv2.CHAIN_APPROX_SIMPLE
            )
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by size
                if area < 500:  # Minimum area threshold
                    continue
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate confidence based on area and aspect ratio
                aspect_ratio = float(w) / h if h > 0 else 0
                confidence = min(area / 5000.0, 1.0)  # Simple confidence scoring
                
                # Filter unrealistic shapes
                if aspect_ratio > 3.0 or aspect_ratio < 0.3:
                    continue
                
                detections.append({
                    'bbox': (x, y, w, h),
                    'class': tree_class,
                    'confidence': confidence
                })
        
        return detections
    
    def deproject_to_3d(self, detection, depth_image):
        """
        Deproject 2D detection to 3D point using depth image.
        Returns point in camera frame coordinates.
        """
        x, y, w, h = detection['bbox']
        center_u = int(x + w / 2)
        center_v = int(y + h / 2)
        
        # Ensure within image bounds
        if (center_v >= depth_image.shape[0] or 
            center_u >= depth_image.shape[1] or
            center_v < 0 or center_u < 0):
            return None
        
        # Get depth value
        depth = depth_image[center_v, center_u]
        
        # Check for invalid depth
        if depth == 0 or np.isnan(depth) or np.isinf(depth):
            return None
        
        # Deproject using pinhole camera model
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = depth
        point_3d = {
            'x': (center_u - self.cx) * depth / self.fx,
            'y': (center_v - self.cy) * depth / self.fy,
            'z': depth
        }
        
        return point_3d
    
    def transform_to_map(self, point_camera):
        """Transform 3D point from camera frame to map frame"""
        try:
            # Create PointStamped in camera frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.camera_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = point_camera['x']
            point_stamped.point.y = point_camera['y']
            point_stamped.point.z = point_camera['z']
            
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Apply transform
            point_map = do_transform_point(point_stamped, transform)
            
            return {
                'x': point_map.point.x,
                'y': point_map.point.y,
                'z': point_map.point.z
            }
        
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {e}')
            return None
    
    def is_duplicate(self, new_point, object_class):
        """Check if landmark already exists within deduplication radius"""
        for landmark in self.landmarks:
            # Only compare same class
            if landmark['class'] != object_class:
                continue
            
            # Calculate Euclidean distance
            dist = np.sqrt(
                (landmark['x'] - new_point['x'])**2 +
                (landmark['y'] - new_point['y'])**2 +
                (landmark['z'] - new_point['z'])**2
            )
            
            if dist < self.dedup_radius:
                return True
        
        return False
    
    def save_landmarks(self):
        """Save landmarks to GeoJSON and CSV files"""
        # Save as JSON
        json_file = os.path.join(self.output_dir, 'objects.json')
        with open(json_file, 'w') as f:
            json.dump(self.landmarks, f, indent=2)
        
        # Save as GeoJSON (assuming planar coordinates, not true lat/lon)
        geojson_file = os.path.join(self.output_dir, 'objects.geojson')
        features = []
        for lm in self.landmarks:
            feature = {
                'type': 'Feature',
                'geometry': {
                    'type': 'Point',
                    'coordinates': [lm['x'], lm['y'], lm['z']]
                },
                'properties': {
                    'class': lm['class'],
                    'confidence': lm['confidence'],
                    'timestamp': lm['timestamp']
                }
            }
            features.append(feature)
        
        geojson = {
            'type': 'FeatureCollection',
            'features': features
        }
        
        with open(geojson_file, 'w') as f:
            json.dump(geojson, f, indent=2)
        
        # Save as CSV
        csv_file = os.path.join(self.output_dir, 'objects.csv')
        with open(csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(
                f, 
                fieldnames=['class', 'x', 'y', 'z', 'confidence', 'timestamp']
            )
            writer.writeheader()
            writer.writerows(self.landmarks)
    
    def publish_markers(self):
        """Visualize landmarks in RViz"""
        marker_array = MarkerArray()
        
        for i, lm in enumerate(self.landmarks):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'objects'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = lm['x']
            marker.pose.position.y = lm['y']
            marker.pose.position.z = lm['z']
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            # Color by class
            if lm['class'] == 'oak':
                marker.color = ColorRGBA(r=0.6, g=0.4, b=0.2, a=1.0)  # Brown
            elif lm['class'] == 'pine':
                marker.color = ColorRGBA(r=0.0, g=0.6, b=0.2, a=1.0)  # Green
            else:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)  # Magenta
            
            # Add text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'object_labels'
            text_marker.id = i + 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = marker.pose
            text_marker.pose.position.z += 0.5
            text_marker.scale.z = 0.3
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f'{lm["class"]}\n{lm["confidence"]:.2f}'
            
            marker_array.markers.append(marker)
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLandmarkMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()