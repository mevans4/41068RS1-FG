# Drone Colour Detector

Camera-based tree detection system for aerial drone surveillance. Detects and classifies trees by health status using HSV color analysis.

**Adapted from:** ForestGuard project
**Platform:** ROS2 Humble, Python 3
**Dependencies:** OpenCV, NumPy, cv_bridge

---

## Features

- **HSV Color-Based Detection**: Identifies trees by foliage color
- **Health Classification**: Distinguishes healthy (green) from unhealthy (red/brown) trees
- **Real-Time Processing**: Processes drone camera feed in real-time
- **Visualization**:
  - RViz markers for detected trees
  - Debug overlay images showing detections
- **Deduplication**: Prevents counting same tree multiple times
- **Interactive Calibration Tool**: Tune HSV thresholds for different environments

---

## Installation

### 1. Build the Package

```bash
cd /home/user/41068RS1-FG/group18ws
colcon build --packages-select drone_colour_detector
source install/setup.bash
```

### 2. Verify Installation

```bash
ros2 pkg list | grep drone_colour_detector
```

---

## Quick Start

### Step 1: Launch Drone Simulation

First, start the drone simulation with camera feed:

```bash
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py
```

### Step 2: Calibrate HSV Thresholds (First Time Only)

Use the interactive calibration tool to tune color detection for your environment:

```bash
ros2 run drone_colour_detector hsv_calibrator
```

**Calibrator Controls:**
- Adjust sliders to tune HSV ranges
- Press `1`: Load green (healthy) preset
- Press `2`: Load red1 (unhealthy) preset
- Press `3`: Load red2 (unhealthy) preset
- Press `4`: Load brown (dead trees) preset
- Press `s`: Save/print current values
- Press `r`: Reset to defaults
- Press `ESC`: Quit and print final values

Copy the printed values into `launch/tree_detection.launch.py`.

### Step 3: Start Tree Detection

```bash
ros2 launch drone_colour_detector tree_detection.launch.py
```

Or run the node directly:

```bash
ros2 run drone_colour_detector tree_detector
```

---

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image` | `sensor_msgs/Image` | RGB camera feed from drone |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/drone/tree_counts` | `std_msgs/Int32MultiArray` | `[healthy_count, unhealthy_count]` |
| `/drone/tree_detections` | `visualization_msgs/MarkerArray` | RViz markers for detected trees |
| `/drone/detected_tree_poses` | `geometry_msgs/PoseArray` | Tree positions (for waypoint planning) |
| `/drone/tree_detection_debug` | `sensor_msgs/Image` | Annotated image with detections overlaid |

---

## Viewing Results

### Option 1: View Debug Image

```bash
ros2 run rqt_image_view rqt_image_view /drone/tree_detection_debug
```

### Option 2: Monitor Counts

```bash
ros2 topic echo /drone/tree_counts
```

Output format: `[healthy_count, unhealthy_count]`

### Option 3: RViz Visualization

Add these displays in RViz:
1. **MarkerArray**: Topic `/drone/tree_detections`
   - Green spheres = healthy trees
   - Red spheres = unhealthy trees
2. **Image**: Topic `/drone/tree_detection_debug`

---

## Configuration

### HSV Color Ranges

Default values are calibrated for standard simulation lighting. Adjust based on your environment:

**Healthy Trees (Green Foliage):**
```python
'green_low': '40,80,40'      # [H, S, V]
'green_high': '90,255,255'
```

**Unhealthy Trees (Red/Brown):**
```python
'red1_low': '0,100,40'       # Red hue (low range)
'red1_high': '15,255,255'
'red2_low': '165,100,40'     # Red hue (high range, wraps around)
'red2_high': '179,255,255'
'brown_low': '10,40,40'      # Brown/dead trees
'brown_high': '30,200,200'
```

### Detection Parameters

**Morphological Operations:**
- `kernel`: Size of morphological kernel (default: 7)
- `open_iters`: Noise removal iterations (default: 2)
- `close_iters`: Gap filling iterations (default: 3)

**Filtering:**
- `roi_ymin`: Top portion of image to ignore (default: 0.1 = ignore top 10%)
- `min_area_px`: Minimum detection area in pixels (default: 800)
- `aspect_min`: Minimum aspect ratio (default: 0.8, trees are ~circular from above)
- `aspect_max`: Maximum aspect ratio (default: 1.5, reject elongated objects)

**Tracking:**
- `track_timeout`: Time to remember previous detections (default: 2.0s)
- `dist_thresh_px`: Distance threshold for duplicate detection (default: 80px)

---

## Integration with Existing System

### Method 1: Standalone Launch

Run detection system separately from main drone launch:

```bash
# Terminal 1: Launch drone
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py

# Terminal 2: Launch tree detection
ros2 launch drone_colour_detector tree_detection.launch.py
```

### Method 2: Integrated Launch (Recommended)

Add to main drone launch file (`41068_ignition_drone.launch.py`):

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Add this to the launch description
tree_detection = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('drone_colour_detector'),
            'launch',
            'tree_detection.launch.py'
        ])
    ]),
    launch_arguments={'use_sim_time': use_sim_time}.items()
)
ld.add_action(tree_detection)
```

### Method 3: Combine with Existing LiDAR Detector

Run both LiDAR and camera detection for multi-sensor fusion:

```bash
# Camera-based detection (discovers new trees)
ros2 run drone_colour_detector tree_detector &

# LiDAR-based detection (measures tree width)
ros2 run lidar_tree_detector lidar_tree_detector_node &
```

---

## How It Works

### Detection Pipeline

1. **Image Acquisition**: Subscribe to `/camera/image`
2. **ROI Selection**: Crop top 10% of image (sky/horizon)
3. **Color Conversion**: Convert BGR → HSV color space
4. **Color Segmentation**: Apply HSV thresholds to create binary masks
5. **Morphological Filtering**: Remove noise and fill gaps
6. **Contour Detection**: Find connected components in masks
7. **Filtering**:
   - Reject small detections (< min_area_px)
   - Reject non-circular shapes (aspect ratio filter)
8. **Deduplication**: Check against recent detections
9. **Output**: Publish counts, markers, and poses

### Why HSV Color Space?

- **H (Hue)**: Color tone (green vs red) - insensitive to lighting
- **S (Saturation)**: Color purity - distinguishes foliage from ground
- **V (Value)**: Brightness - adapts to shadows

HSV is more robust to lighting changes than RGB.

### Deduplication Logic

Prevents counting the same tree multiple times as drone moves:
- Maintains short-term memory of detected tree positions
- Compares new detections to recent ones (within `dist_thresh_px`)
- Expires old detections after `track_timeout` seconds

---

## Advantages Over LiDAR-Only Detection

| Aspect | LiDAR-Only (Current) | Camera + LiDAR (New) |
|--------|---------------------|----------------------|
| **Tree Discovery** | Requires pre-registered coordinates | Discovers trees dynamically |
| **Health Status** | Cannot detect | Classifies healthy vs unhealthy |
| **Scalability** | Limited to 18 hardcoded trees | Works in any forest |
| **Robustness** | Single sensor | Multi-sensor fusion |
| **Width Measurement** | ✓ Accurate | ✗ Needs depth data |
| **Real-World Use** | Limited | Production-ready |

**Recommended:** Use both systems together:
- **Camera**: Discover and classify trees by health
- **LiDAR**: Measure tree trunk width accurately

---

## Troubleshooting

### Executable Not Found Error

**Problem:** `ros2 run drone_colour_detector hsv_calibrator` returns "No executable found"

**Root Cause:** The package was built but executables weren't installed to the correct location.

**Solutions:**

1. **Clean rebuild with proper setup.cfg:**
   ```bash
   cd ~/41068_ws
   rm -rf build/drone_colour_detector install/drone_colour_detector
   colcon build --packages-select drone_colour_detector
   source install/setup.bash
   ```

2. **Verify installation:**
   ```bash
   # Check if executables exist
   ls -la ~/41068_ws/install/drone_colour_detector/lib/drone_colour_detector/

   # Should show: hsv_calibrator and tree_detector
   ```

3. **Check package is found:**
   ```bash
   ros2 pkg list | grep drone_colour_detector
   ```

4. **Verify setup.cfg exists:**
   The package now includes a `setup.cfg` file that tells setuptools where to install scripts. If it's missing, create one with:
   ```ini
   [develop]
   script_dir=$base/lib/drone_colour_detector

   [install]
   install_scripts=$base/lib/drone_colour_detector
   ```

### No Detections

**Problem:** `tree_counts` shows `[0, 0]`

**Solutions:**
1. Check camera feed is active: `ros2 topic hz /camera/image`
2. View debug image: `ros2 run rqt_image_view rqt_image_view /drone/tree_detection_debug`
3. Recalibrate HSV thresholds with `hsv_calibrator`
4. Reduce `min_area_px` parameter
5. Check drone altitude (trees might appear too small)

### Too Many False Positives

**Problem:** Detects ground, grass, or other green objects

**Solutions:**
1. Increase `min_area_px` to reject small patches
2. Tighten aspect ratio range (`aspect_min`, `aspect_max`)
3. Adjust HSV ranges to exclude non-tree greens
4. Increase `open_iters` for more aggressive noise filtering

### Duplicate Detections

**Problem:** Same tree counted multiple times

**Solutions:**
1. Increase `dist_thresh_px` (default: 80px)
2. Increase `track_timeout` (default: 2.0s)
3. Slow down drone movement

### Camera Not Pointing Down

**Problem:** Camera view shows too much horizon

**Note:** Current drone model has camera at 45° pitch. To modify:

Edit `/41068_ignition_bringup/urdf_drone/parrot.urdf.xacro` line 92:

```xml
<!-- Change from 45° to 90° (straight down) -->
<origin xyz="0.25 0.0 0.25" rpy="0 1.5708 0"/>  <!-- 1.5708 rad = 90° -->
```

Rebuild: `colcon build --packages-select 41068_ignition_bringup`

---

## Performance Optimization

### For Slower Computers

Reduce processing load:
```python
'open_iters': 1,      # Less morphological operations
'close_iters': 1,
'kernel': 5,          # Smaller kernel
```

Lower camera update rate in `parrot.gazebo.xacro`:
```xml
<update_rate>1</update_rate>  <!-- From 3 Hz to 1 Hz -->
```

### For Real Drone (Non-Simulation)

Set `use_sim_time` to `False`:
```bash
ros2 launch drone_colour_detector tree_detection.launch.py use_sim_time:=False
```

---

## Examples

### Example 1: Basic Tree Counting

```bash
# Launch everything
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py &
ros2 launch drone_colour_detector tree_detection.launch.py &

# Monitor detections
ros2 topic echo /drone/tree_counts
```

### Example 2: Visual Debugging

```bash
# Start detection
ros2 run drone_colour_detector tree_detector

# View annotated camera feed
ros2 run rqt_image_view rqt_image_view /drone/tree_detection_debug
```

### Example 3: Custom HSV Ranges

Create custom launch file with your calibrated values:

```python
parameters=[{
    'green_low': '35,90,50',    # Your calibrated values
    'green_high': '85,255,255',
    # ... etc
}]
```

---

## Future Enhancements

Potential improvements:
1. **Depth Integration**: Use depth camera for accurate 3D positioning
2. **Machine Learning**: CNN-based tree detection for better accuracy
3. **Database Integration**: Store detected trees in persistent database
4. **Waypoint Generation**: Auto-generate waypoints to inspect unhealthy trees
5. **Tree Counting Statistics**: Track detection history over time

---

## Citation

Adapted from the ForestGuard project's color detection system:
- Original: `/home/user/41068RS1-FG/ForestGuard/src/forestguard_colour/`
- Modifications: Optimized for aerial drone perspective and Group18 integration

---

## Support

For issues or questions:
1. Check simulation is running: `gz topic -l`
2. Verify camera topic: `ros2 topic list | grep camera`
3. Review logs: `ros2 node info /drone_tree_detector`
4. Recalibrate HSV ranges with interactive tool

---

## License

MIT License (same as ForestGuard project)
