#!/usr/bin/env python3
"""
HSV Calibrator for Drone Tree Detection

Interactive tool to tune HSV color thresholds for tree detection.
Displays live camera feed with adjustable HSV range sliders.

Usage:
    ros2 run drone_colour_detector hsv_calibrator

Controls:
    - Adjust H/S/V sliders to tune color range
    - ESC: Quit and print final HSV values
    - 's': Save current values to terminal
    - 'r': Reset to default green values

The tool shows three windows side-by-side:
    1. Original camera feed
    2. Binary mask (white = detected pixels)
    3. Masked overlay (original colors where mask is white)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class HSVCalibrator(Node):
    def __init__(self):
        super().__init__('hsv_calibrator')
        self.bridge = CvBridge()
        self.frame = None

        # Subscribe to drone camera
        self.declare_parameter('image_topic', '/camera/image')
        img_topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, img_topic, self.image_cb, 10)

        self.get_logger().info(f"HSV Calibrator started. Subscribed to: {img_topic}")
        self.get_logger().info("Use sliders to adjust HSV range. Press ESC to quit.")
        self.get_logger().info("Press 's' to save current values, 'r' to reset.")

        # Create OpenCV window with trackbars
        cv2.namedWindow("HSV Calibrator", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("HSV Calibrator", 1920, 480)
        self._make_trackbars()

        # Preset configurations
        self.presets = {
            'green': {'low': (40, 80, 40), 'high': (90, 255, 255)},
            'red1': {'low': (0, 100, 40), 'high': (15, 255, 255)},
            'red2': {'low': (165, 100, 40), 'high': (179, 255, 255)},
            'brown': {'low': (10, 40, 40), 'high': (30, 200, 200)},
        }

    def _make_trackbars(self):
        """Create HSV range trackbars."""
        # Lower bounds
        cv2.createTrackbar('H_low', "HSV Calibrator", 40, 179, lambda x: None)
        cv2.createTrackbar('S_low', "HSV Calibrator", 80, 255, lambda x: None)
        cv2.createTrackbar('V_low', "HSV Calibrator", 40, 255, lambda x: None)

        # Upper bounds
        cv2.createTrackbar('H_high', "HSV Calibrator", 90, 179, lambda x: None)
        cv2.createTrackbar('S_high', "HSV Calibrator", 255, 255, lambda x: None)
        cv2.createTrackbar('V_high', "HSV Calibrator", 255, 255, lambda x: None)

    def image_cb(self, msg):
        """Receive camera images."""
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def get_trackbar_values(self):
        """Read current trackbar positions."""
        hL = cv2.getTrackbarPos('H_low', "HSV Calibrator")
        sL = cv2.getTrackbarPos('S_low', "HSV Calibrator")
        vL = cv2.getTrackbarPos('V_low', "HSV Calibrator")
        hH = cv2.getTrackbarPos('H_high', "HSV Calibrator")
        sH = cv2.getTrackbarPos('S_high', "HSV Calibrator")
        vH = cv2.getTrackbarPos('V_high', "HSV Calibrator")

        return (hL, sL, vL), (hH, sH, vH)

    def set_trackbar_values(self, lower, upper):
        """Set trackbar positions."""
        cv2.setTrackbarPos('H_low', "HSV Calibrator", lower[0])
        cv2.setTrackbarPos('S_low', "HSV Calibrator", lower[1])
        cv2.setTrackbarPos('V_low', "HSV Calibrator", lower[2])
        cv2.setTrackbarPos('H_high', "HSV Calibrator", upper[0])
        cv2.setTrackbarPos('S_high', "HSV Calibrator", upper[1])
        cv2.setTrackbarPos('V_high', "HSV Calibrator", upper[2])

    def print_values(self, lower, upper):
        """Print current HSV values in ROS parameter format."""
        print("\n" + "="*60)
        print("Current HSV Range:")
        print(f"  Lower: [{lower[0]}, {lower[1]}, {lower[2]}]")
        print(f"  Upper: [{upper[0]}, {upper[1]}, {upper[2]}]")
        print("\nROS2 Parameter Format:")
        print(f"  'parameter_name_low':  '{lower[0]},{lower[1]},{lower[2]}'")
        print(f"  'parameter_name_high': '{upper[0]},{upper[1]},{upper[2]}'")
        print("="*60 + "\n")

    def spin_once(self):
        """Process one frame and display results."""
        if self.frame is None:
            return

        # Get current HSV range from trackbars
        lower, upper = self.get_trackbar_values()
        lower_np = np.array(lower, dtype=np.uint8)
        upper_np = np.array(upper, dtype=np.uint8)

        # Convert to HSV
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        # Create mask
        mask = cv2.inRange(hsv, lower_np, upper_np)

        # Apply mask to original image
        overlay = cv2.bitwise_and(self.frame, self.frame, mask=mask)

        # Convert mask to 3-channel for display
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Add text overlay with current values
        info_text = f"HSV Low: [{lower[0]},{lower[1]},{lower[2]}]  High: [{upper[0]},{upper[1]},{upper[2]}]"
        cv2.putText(self.frame, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Calculate detection percentage
        total_pixels = mask.size
        detected_pixels = np.count_nonzero(mask)
        detection_pct = (detected_pixels / total_pixels) * 100
        pct_text = f"Detection: {detection_pct:.2f}%"
        cv2.putText(mask_colored, pct_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Stack images horizontally for comparison
        stacked = np.hstack([self.frame, mask_colored, overlay])

        # Display
        cv2.imshow("HSV Calibrator", stacked)

        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            self.print_values(lower, upper)
            self.get_logger().info("Calibration complete. Shutting down...")
            cv2.destroyAllWindows()
            raise SystemExit

        elif key == ord('s'):  # Save
            self.print_values(lower, upper)

        elif key == ord('r'):  # Reset to default green
            self.get_logger().info("Resetting to default green preset...")
            self.set_trackbar_values(
                self.presets['green']['low'],
                self.presets['green']['high']
            )

        elif key == ord('1'):  # Load green preset
            self.get_logger().info("Loading green (healthy) preset...")
            self.set_trackbar_values(
                self.presets['green']['low'],
                self.presets['green']['high']
            )

        elif key == ord('2'):  # Load red1 preset
            self.get_logger().info("Loading red1 (unhealthy) preset...")
            self.set_trackbar_values(
                self.presets['red1']['low'],
                self.presets['red1']['high']
            )

        elif key == ord('3'):  # Load red2 preset
            self.get_logger().info("Loading red2 (unhealthy) preset...")
            self.set_trackbar_values(
                self.presets['red2']['low'],
                self.presets['red2']['high']
            )

        elif key == ord('4'):  # Load brown preset
            self.get_logger().info("Loading brown (dead) preset...")
            self.set_trackbar_values(
                self.presets['brown']['low'],
                self.presets['brown']['high']
            )


def main(args=None):
    rclpy.init(args=args)
    node = HSVCalibrator()

    print("\n" + "="*60)
    print("HSV Calibrator - Interactive Color Tuning Tool")
    print("="*60)
    print("Keyboard shortcuts:")
    print("  ESC - Quit and print final values")
    print("  's' - Save/print current values")
    print("  'r' - Reset to default green values")
    print("  '1' - Load green (healthy) preset")
    print("  '2' - Load red1 (unhealthy) preset")
    print("  '3' - Load red2 (unhealthy) preset")
    print("  '4' - Load brown (dead) preset")
    print("="*60 + "\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.spin_once()
    except SystemExit:
        pass
    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
