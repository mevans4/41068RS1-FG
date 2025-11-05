#!/usr/bin/env python3
"""
Drone Tree Detector - Adapted from ForestGuard
HSV color-based tree detection optimized for aerial drone perspective.

Detects healthy (green) and unhealthy (red/brown) trees from downward-facing camera.
Publishes detection counts, visualization markers, and debug images.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


def _csv3(s, fb):
    """Parse comma-separated HSV values or return fallback."""
    try:
        v = [int(x.strip()) for x in str(s).split(',')]
        return np.array(v[:3], dtype=np.uint8) if len(v) >= 3 else np.array(fb, dtype=np.uint8)
    except Exception:
        return np.array(fb, dtype=np.uint8)


class DroneTreeDetector(Node):
    def __init__(self):
        super().__init__('drone_tree_detector')
        self.bridge = CvBridge()

        # ---- PARAMETERS ----
        # Camera topic
        self.declare_parameter('image_topic', '/camera/image')

        # HSV thresholds for healthy trees (green foliage)
        self.declare_parameter('green_low',  '40,80,40')      # Adjusted for aerial view
        self.declare_parameter('green_high', '90,255,255')

        # HSV thresholds for unhealthy trees (red/brown/dead)
        # Red wraps around in HSV (0-10 and 170-180)
        self.declare_parameter('red1_low',   '0,100,40')
        self.declare_parameter('red1_high',  '15,255,255')
        self.declare_parameter('red2_low',   '165,100,40')
        self.declare_parameter('red2_high',  '179,255,255')

        # Brown/dead trees (lower saturation, yellow-orange hues)
        self.declare_parameter('brown_low',  '10,40,40')
        self.declare_parameter('brown_high', '30,200,200')

        # Morphological operations
        self.declare_parameter('kernel', 7)                    # Larger for aerial noise
        self.declare_parameter('open_iters', 2)                # More aggressive noise removal
        self.declare_parameter('close_iters', 3)               # Fill gaps in tree canopy

        # Detection filters
        self.declare_parameter('roi_ymin', 0.1)                # Use most of image (aerial view)
        self.declare_parameter('min_area_px', 800)             # Smaller trees from altitude
        self.declare_parameter('aspect_min', 0.8)              # Trees appear more circular from above
        self.declare_parameter('aspect_max', 1.5)              # Reject elongated non-tree objects

        # Tracking/deduplication
        self.declare_parameter('track_timeout', 2.0)           # Longer timeout for slower drone movement
        self.declare_parameter('dist_thresh_px', 80.0)         # Distance threshold in pixels

        # Get parameter values
        img_topic      = self.get_parameter('image_topic').value
        self.lower_g   = _csv3(self.get_parameter('green_low').value,  (40,80,40))
        self.upper_g   = _csv3(self.get_parameter('green_high').value, (90,255,255))
        self.lower_r1  = _csv3(self.get_parameter('red1_low').value,   (0,100,40))
        self.upper_r1  = _csv3(self.get_parameter('red1_high').value,  (15,255,255))
        self.lower_r2  = _csv3(self.get_parameter('red2_low').value,   (165,100,40))
        self.upper_r2  = _csv3(self.get_parameter('red2_high').value,  (179,255,255))
        self.lower_b   = _csv3(self.get_parameter('brown_low').value,  (10,40,40))
        self.upper_b   = _csv3(self.get_parameter('brown_high').value, (30,200,200))

        ksize          = int(self.get_parameter('kernel').value)
        self.open_it   = int(self.get_parameter('open_iters').value)
        self.close_it  = int(self.get_parameter('close_iters').value)
        self.roi_ymin  = float(self.get_parameter('roi_ymin').value)
        self.min_area  = int(self.get_parameter('min_area_px').value)
        self.aspect_min= float(self.get_parameter('aspect_min').value)
        self.aspect_max= float(self.get_parameter('aspect_max').value)
        self.track_timeout = float(self.get_parameter('track_timeout').value)
        self.dist_thresh = float(self.get_parameter('dist_thresh_px').value)

        self.kernel = np.ones((max(1, ksize), max(1, ksize)), np.uint8)

        # ---- PUBLISHERS & SUBSCRIBERS ----
        self.sub = self.create_subscription(Image, img_topic, self.image_cb, 10)

        # Debug visualization (overlay image)
        self.pub_debug = self.create_publisher(Image, '/drone/tree_detection_debug', 10)

        # Tree counts [healthy_count, unhealthy_count]
        self.pub_counts = self.create_publisher(Int32MultiArray, '/drone/tree_counts', 10)

        # RViz markers for visualization
        self.pub_markers = self.create_publisher(MarkerArray, '/drone/tree_detections', 10)

        # Tree positions as PoseArray (for navigation/waypoint generation)
        self.pub_poses = self.create_publisher(PoseArray, '/drone/detected_tree_poses', 10)

        self.marker_seq = 0

        # ---- TRACKING MEMORY ----
        # Simple short-term memory to avoid counting same tree multiple times
        self.track_memory = []  # list of (cx, cy, timestamp, color_id)

        self.last_time = time.time()

        self.get_logger().info(
            f"Drone Tree Detector Started\n"
            f"  Subscribed: {img_topic}\n"
            f"  Green HSV: {self.lower_g.tolist()} to {self.upper_g.tolist()}\n"
            f"  Red1 HSV:  {self.lower_r1.tolist()} to {self.upper_r1.tolist()}\n"
            f"  Red2 HSV:  {self.lower_r2.tolist()} to {self.upper_r2.tolist()}\n"
            f"  Brown HSV: {self.lower_b.tolist()} to {self.upper_b.tolist()}\n"
            f"  ROI Y-min: {self.roi_ymin}, Min Area: {self.min_area}px\n"
            f"  Aspect ratio: {self.aspect_min} - {self.aspect_max}"
        )

    def _morph(self, mask):
        """Apply morphological operations to clean up binary mask."""
        if self.open_it > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=self.open_it)
        if self.close_it > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=self.close_it)
        return mask

    def _update_tracks(self, cx, cy, color_id):
        """
        Keep short-term memory of previously seen trees to avoid double counting.
        Returns True if this is a new detection, False if it's a duplicate.
        """
        now = time.time()
        # Remove old tracks that have timed out
        self.track_memory = [
            (x, y, t, c) for (x, y, t, c) in self.track_memory
            if now - t < self.track_timeout
        ]

        # Check if current detection matches any recent track
        for x, y, t, c in self.track_memory:
            dist = np.hypot(cx - x, cy - y)
            if dist < self.dist_thresh and c == color_id:
                return False  # Duplicate detection

        # New detection - add to memory
        self.track_memory.append((cx, cy, now, color_id))
        return True

    def _count_and_mark(self, frame, mask, color_id, color_name, header):
        """
        Find contours in mask, filter by size/aspect ratio, and create markers.

        Args:
            frame: Original image for drawing
            mask: Binary mask of detected color
            color_id: 0=healthy, 1=unhealthy
            color_name: 'healthy' or 'unhealthy'
            header: Image header for timestamp/frame_id

        Returns:
            count: Number of new detections
            markers: List of RViz markers
            poses: List of tree positions
        """
        n = 0
        markers = []
        poses = []

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(c)
            aspect = h / float(w + 1e-3)

            # From above, trees should be roughly circular (aspect ~ 1.0)
            if aspect < self.aspect_min or aspect > self.aspect_max:
                continue

            cx, cy = x + w / 2, y + h / 2

            # Check if this is a duplicate detection
            if not self._update_tracks(cx, cy, color_id):
                continue

            n += 1

            # Draw bounding box on debug image
            color = (0, 255, 0) if color_id == 0 else (0, 0, 255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

            # Draw label
            label = f"{color_name} #{n}"
            cv2.putText(frame, label, (x, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Create RViz marker
            m = Marker()
            m.header.frame_id = 'camera_link'  # Drone camera frame
            m.header.stamp = header.stamp
            m.ns = 'drone_trees'
            m.id = self.marker_seq
            self.marker_seq += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            # Position relative to camera (approximate 3D position)
            # For better accuracy, would need depth data or altitude info
            m.pose.position.x = 2.0  # Forward distance (placeholder)
            m.pose.position.y = (cx - frame.shape[1] / 2) / 300.0  # Horizontal offset
            m.pose.position.z = -(cy - frame.shape[0] / 2) / 300.0  # Vertical offset

            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.lifetime.sec = 5  # Markers expire after 5 seconds

            if color_id == 0:
                # Healthy tree - green
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.1, 0.9
            else:
                # Unhealthy tree - red
                m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.1, 0.1, 0.9

            markers.append(m)

            # Create pose for navigation
            p = Pose()
            p.position.x = m.pose.position.x
            p.position.y = m.pose.position.y
            p.position.z = m.pose.position.z
            p.orientation.w = 1.0
            poses.append(p)

        return n, markers, poses

    def image_cb(self, msg: Image):
        """Main image callback - processes camera feed and detects trees."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        H, W, _ = frame.shape

        # Apply ROI to focus on relevant part of image
        y0 = int(self.roi_ymin * H)
        roi = frame[y0:, :]

        # Convert to HSV color space
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Create masks for different tree types
        mask_green = cv2.inRange(hsv, self.lower_g, self.upper_g)
        mask_red1  = cv2.inRange(hsv, self.lower_r1, self.upper_r1)
        mask_red2  = cv2.inRange(hsv, self.lower_r2, self.upper_r2)
        mask_brown = cv2.inRange(hsv, self.lower_b, self.upper_b)

        # Combine red and brown for unhealthy trees
        mask_unhealthy = cv2.bitwise_or(mask_red1, mask_red2)
        mask_unhealthy = cv2.bitwise_or(mask_unhealthy, mask_brown)

        # Clean up masks with morphological operations
        mask_green = self._morph(mask_green)
        mask_unhealthy = self._morph(mask_unhealthy)

        # Detect and count trees
        num_healthy, markers_healthy, poses_healthy = self._count_and_mark(
            roi, mask_green, 0, 'healthy', msg.header
        )
        num_unhealthy, markers_unhealthy, poses_unhealthy = self._count_and_mark(
            roi, mask_unhealthy, 1, 'unhealthy', msg.header
        )

        # Publish counts
        counts = Int32MultiArray()
        counts.data = [num_healthy, num_unhealthy]
        self.pub_counts.publish(counts)

        # Publish markers
        ma = MarkerArray()
        ma.markers = markers_healthy + markers_unhealthy
        self.pub_markers.publish(ma)

        # Publish poses
        pa = PoseArray()
        pa.header.frame_id = 'camera_link'
        pa.header.stamp = msg.header.stamp
        pa.poses = poses_healthy + poses_unhealthy
        self.pub_poses.publish(pa)

        # Create debug visualization
        overlay = np.zeros_like(roi)
        overlay[mask_green > 0] = [0, 255, 0]        # Green for healthy
        overlay[mask_unhealthy > 0] = [0, 0, 255]    # Red for unhealthy

        debug = frame.copy()
        debug[y0:, :] = cv2.addWeighted(roi, 0.6, overlay, 0.4, 0)

        # Add info text
        info_text = f"Healthy: {num_healthy}  Unhealthy: {num_unhealthy}"
        cv2.putText(debug, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Publish debug image
        try:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DroneTreeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
