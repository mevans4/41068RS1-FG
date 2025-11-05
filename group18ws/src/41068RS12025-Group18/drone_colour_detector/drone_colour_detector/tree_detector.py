#!/usr/bin/env python3
"""
Drone Tree Detector - Adapted from ForestGuard
HSV color-based tree detection optimized for aerial drone perspective.

Detects healthy (green) and unhealthy (red/brown) trees from downward-facing camera.
Publishes detection counts, visualization markers, and debug images.
Logs detections to CSV/JSON with world coordinates for mission analysis.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
import csv
from pathlib import Path
from datetime import datetime
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import traceback


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

        # ---- TF2 SETUP FOR WORLD COORDINATES ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- PARAMETERS ----
        # Camera topic
        self.declare_parameter('image_topic', '/camera/image')

        # Data logging parameters
        self.declare_parameter('enable_logging', True)
        self.declare_parameter('log_directory', '~/drone_tree_logs')
        self.declare_parameter('mission_name', f'mission_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
        self.declare_parameter('target_frame', 'map')  # World coordinate frame (map, odom, or world)
        self.declare_parameter('save_format', 'csv')  # csv or json
        self.declare_parameter('enable_waypoints', True)  # Export waypoints for unhealthy trees

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
        self.enable_logging = self.get_parameter('enable_logging').value
        log_dir        = Path(self.get_parameter('log_directory').value).expanduser()
        self.mission_name = self.get_parameter('mission_name').value
        self.target_frame = self.get_parameter('target_frame').value
        self.save_format = self.get_parameter('save_format').value
        self.enable_waypoints = self.get_parameter('enable_waypoints').value

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

        # ---- DATA LOGGING SETUP ----
        self.detections = []  # List of all detections for mission report
        self.cumulative_healthy = 0
        self.cumulative_unhealthy = 0
        self.mission_start_time = datetime.now()

        if self.enable_logging:
            log_dir.mkdir(parents=True, exist_ok=True)
            self.log_file = log_dir / f"{self.mission_name}_detections.{self.save_format}"
            self.report_file = log_dir / f"{self.mission_name}_report.txt"
            self.waypoint_file = log_dir / f"{self.mission_name}_waypoints.txt"

            # Initialize CSV with headers if using CSV format
            if self.save_format == 'csv':
                with open(self.log_file, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=[
                        'timestamp', 'tree_id', 'health_status',
                        'camera_x', 'camera_y', 'camera_z',
                        'world_x', 'world_y', 'world_z',
                        'world_frame', 'area_px', 'aspect_ratio'
                    ])
                    writer.writeheader()

            self.get_logger().info(f"Logging enabled: {self.log_file}")
        else:
            self.log_file = None
            self.get_logger().info("Logging disabled")

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

    def _camera_to_world(self, camera_point, stamp):
        """
        Transform camera-relative coordinates to world coordinates using TF.

        Args:
            camera_point: tuple (x, y, z) in camera frame
            stamp: ROS timestamp for transform lookup

        Returns:
            tuple (world_x, world_y, world_z, frame_id) or (None, None, None, None) if transform fails
        """
        try:
            # Create PointStamped in camera frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = 'camera_link'
            point_stamped.header.stamp = stamp
            point_stamped.point.x = camera_point[0]
            point_stamped.point.y = camera_point[1]
            point_stamped.point.z = camera_point[2]

            # Transform to target frame (map, odom, or world)
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                'camera_link',
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            world_point = do_transform_point(point_stamped, transform)

            return (
                world_point.point.x,
                world_point.point.y,
                world_point.point.z,
                self.target_frame
            )

        except Exception as e:
            # TF lookup can fail if frames aren't available yet
            # This is common early in mission or in simulation
            return (None, None, None, None)

    def _log_detection(self, detection_data):
        """
        Log a tree detection to file.

        Args:
            detection_data: dict with detection information
        """
        if not self.enable_logging:
            return

        # Add to in-memory list for mission report
        self.detections.append(detection_data)

        # Write to file
        try:
            if self.save_format == 'csv':
                with open(self.log_file, 'a', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=[
                        'timestamp', 'tree_id', 'health_status',
                        'camera_x', 'camera_y', 'camera_z',
                        'world_x', 'world_y', 'world_z',
                        'world_frame', 'area_px', 'aspect_ratio'
                    ])
                    writer.writerow(detection_data)
            else:  # JSON
                with open(self.log_file, 'a') as f:
                    json.dump(detection_data, f)
                    f.write('\n')
        except Exception as e:
            self.get_logger().error(f"Failed to log detection: {e}")

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

            # Update cumulative counts
            if color_id == 0:
                self.cumulative_healthy += 1
                tree_id = f"H{self.cumulative_healthy}"
            else:
                self.cumulative_unhealthy += 1
                tree_id = f"U{self.cumulative_unhealthy}"

            # Draw bounding box on debug image
            color = (0, 255, 0) if color_id == 0 else (0, 0, 255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

            # Draw label
            label = f"{color_name} #{tree_id}"
            cv2.putText(frame, label, (x, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Estimate 3D position in camera frame
            # Better estimation: use tree size to estimate distance
            # Assumption: average tree canopy diameter is ~3-5 meters
            # If we see it as w pixels wide, we can estimate distance
            # This is rough but better than a fixed placeholder
            assumed_tree_diameter_m = 4.0  # meters
            focal_length_px = 500.0  # typical for drone cameras (adjust if known)
            estimated_distance = (assumed_tree_diameter_m * focal_length_px) / (w + 1e-3)
            estimated_distance = max(1.0, min(estimated_distance, 50.0))  # Clamp to reasonable range

            # Camera coordinates (downward-facing camera)
            cam_x = estimated_distance
            cam_y = (cx - frame.shape[1] / 2) * estimated_distance / focal_length_px
            cam_z = -(cy - frame.shape[0] / 2) * estimated_distance / focal_length_px

            # Transform to world coordinates
            world_x, world_y, world_z, world_frame = self._camera_to_world(
                (cam_x, cam_y, cam_z), header.stamp
            )

            # Create RViz marker
            m = Marker()
            m.header.frame_id = world_frame if world_frame else 'camera_link'
            m.header.stamp = header.stamp
            m.ns = 'drone_trees'
            m.id = self.marker_seq
            self.marker_seq += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            # Use world coordinates if available, otherwise camera coordinates
            if world_x is not None:
                m.pose.position.x = world_x
                m.pose.position.y = world_y
                m.pose.position.z = world_z
            else:
                m.pose.position.x = cam_x
                m.pose.position.y = cam_y
                m.pose.position.z = cam_z

            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.lifetime.sec = 30  # Markers now last 30 seconds instead of 5

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

            # Log detection to file
            detection_data = {
                'timestamp': datetime.now().isoformat(),
                'tree_id': tree_id,
                'health_status': color_name,
                'camera_x': round(cam_x, 2),
                'camera_y': round(cam_y, 2),
                'camera_z': round(cam_z, 2),
                'world_x': round(world_x, 2) if world_x is not None else None,
                'world_y': round(world_y, 2) if world_y is not None else None,
                'world_z': round(world_z, 2) if world_z is not None else None,
                'world_frame': world_frame,
                'area_px': int(area),
                'aspect_ratio': round(aspect, 2)
            }
            self._log_detection(detection_data)

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

        # Add info text with both current frame and cumulative counts
        info_text = f"Frame: H:{num_healthy} U:{num_unhealthy} | Total: H:{self.cumulative_healthy} U:{self.cumulative_unhealthy}"
        cv2.putText(debug, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Publish debug image
        try:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")

    def generate_mission_report(self):
        """Generate comprehensive mission report with statistics and findings."""
        if not self.enable_logging or not self.detections:
            return

        try:
            mission_duration = datetime.now() - self.mission_start_time

            report = []
            report.append("=" * 70)
            report.append("DRONE FOREST MONITORING MISSION REPORT")
            report.append("=" * 70)
            report.append(f"Mission Name: {self.mission_name}")
            report.append(f"Start Time:   {self.mission_start_time.strftime('%Y-%m-%d %H:%M:%S')}")
            report.append(f"End Time:     {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            report.append(f"Duration:     {mission_duration}")
            report.append("")
            report.append("-" * 70)
            report.append("DETECTION SUMMARY")
            report.append("-" * 70)
            report.append(f"Total Trees Detected:    {len(self.detections)}")
            report.append(f"  Healthy Trees:         {self.cumulative_healthy}")
            report.append(f"  Unhealthy Trees:       {self.cumulative_unhealthy}")

            if len(self.detections) > 0:
                health_percentage = (self.cumulative_healthy / len(self.detections)) * 100
                report.append(f"  Forest Health Index:   {health_percentage:.1f}%")

            report.append("")
            report.append("-" * 70)
            report.append("DATA FILES")
            report.append("-" * 70)
            report.append(f"Detection Log:  {self.log_file}")

            if self.enable_waypoints and self.cumulative_unhealthy > 0:
                report.append(f"Waypoint File:  {self.waypoint_file}")

            report.append("")

            # Statistics on detections with world coordinates
            world_coords_count = sum(1 for d in self.detections if d.get('world_x') is not None)
            report.append("-" * 70)
            report.append("COORDINATE TRANSFORM STATUS")
            report.append("-" * 70)
            report.append(f"Detections with world coordinates: {world_coords_count} / {len(self.detections)}")

            if world_coords_count < len(self.detections):
                report.append("NOTE: Some detections lack world coordinates (TF unavailable)")

            report.append("")
            report.append("-" * 70)
            report.append("UNHEALTHY TREE LOCATIONS (Priority for Inspection)")
            report.append("-" * 70)

            unhealthy_trees = [d for d in self.detections if d['health_status'] == 'unhealthy']

            if unhealthy_trees:
                for tree in unhealthy_trees:
                    if tree.get('world_x') is not None:
                        report.append(
                            f"  {tree['tree_id']}: ({tree['world_x']:.2f}, "
                            f"{tree['world_y']:.2f}, {tree['world_z']:.2f}) "
                            f"in {tree['world_frame']} frame"
                        )
                    else:
                        report.append(
                            f"  {tree['tree_id']}: Camera-relative position only "
                            f"({tree['camera_x']:.2f}, {tree['camera_y']:.2f}, {tree['camera_z']:.2f})"
                        )
            else:
                report.append("  No unhealthy trees detected - forest appears healthy!")

            report.append("")
            report.append("=" * 70)
            report.append("END OF REPORT")
            report.append("=" * 70)

            # Write report to file
            with open(self.report_file, 'w') as f:
                f.write('\n'.join(report))

            # Also log to console
            self.get_logger().info("Mission report generated:")
            for line in report:
                self.get_logger().info(line)

        except Exception as e:
            self.get_logger().error(f"Failed to generate mission report: {e}\n{traceback.format_exc()}")

    def export_waypoints(self):
        """Export waypoints for unhealthy trees for follow-up missions."""
        if not self.enable_logging or not self.enable_waypoints:
            return

        try:
            unhealthy_trees = [d for d in self.detections
                             if d['health_status'] == 'unhealthy'
                             and d.get('world_x') is not None]

            if not unhealthy_trees:
                self.get_logger().info("No unhealthy trees with world coordinates to export as waypoints")
                return

            waypoints = []
            waypoints.append("# Waypoints for Unhealthy Tree Inspection")
            waypoints.append(f"# Mission: {self.mission_name}")
            waypoints.append(f"# Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            waypoints.append(f"# Frame: {self.target_frame}")
            waypoints.append("#")
            waypoints.append("# Format: tree_id, x, y, z")
            waypoints.append("")

            for tree in unhealthy_trees:
                waypoints.append(
                    f"{tree['tree_id']}, {tree['world_x']:.2f}, "
                    f"{tree['world_y']:.2f}, {tree['world_z']:.2f}"
                )

            with open(self.waypoint_file, 'w') as f:
                f.write('\n'.join(waypoints))

            self.get_logger().info(f"Exported {len(unhealthy_trees)} waypoints to {self.waypoint_file}")

        except Exception as e:
            self.get_logger().error(f"Failed to export waypoints: {e}\n{traceback.format_exc()}")


def main(args=None):
    rclpy.init(args=args)
    node = DroneTreeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down - generating final reports...")
    finally:
        # Generate mission report and export waypoints on shutdown
        try:
            node.export_waypoints()
            node.generate_mission_report()
        except Exception as e:
            node.get_logger().error(f"Error generating final reports: {e}")

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
