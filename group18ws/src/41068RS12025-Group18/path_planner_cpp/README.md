# Path Planner with Dynamic Goal Support

## Overview
This path planner integrates with Nav2 and now supports dynamic goal coordinates through a custom service interface.

## Features

### 1. Dynamic Goal Planning Service
- **Service Name:** `/plan_to_goal`
- **Service Type:** `path_planner_cpp/srv/PlanToGoal`
- **Request Parameters:**
  - `float64 x` - Goal X coordinate
  - `float64 y` - Goal Y coordinate
  - `float64 z` - Goal Z coordinate
- **Response:**
  - `bool success` - Planning success status
  - `string message` - Status message

### 2. Legacy Trigger Service (Backward Compatible)
- **Service Name:** `/trigger_path_plan`
- **Service Type:** `std_srvs/srv/Trigger`
- Plans to origin (0, 0, 0) for backward compatibility

### 3. Path Publishing
- Published path is available on `/path` topic
- Compatible with drone controller for automatic waypoint following

## Usage

### Via Service Call
```bash
# Plan to a specific goal
ros2 service call /plan_to_goal path_planner_cpp/srv/PlanToGoal "{x: 4.0, y: 26.0, z: 0.5}"

# Or use the legacy trigger (plans to 0,0,0)
ros2 service call /trigger_path_plan std_srvs/srv/Trigger
```

### Via Drone UI
1. Open the Drone UI
2. Enter goal coordinates in the "Path Planning" section:
   - X: Target X coordinate
   - Y: Target Y coordinate
   - Z: Target altitude
3. Click "Plan Path to Goal"
4. The drone controller will automatically follow the planned path

## Integration with Drone Controller

The drone controller now:
- Subscribes to `/path` topic
- Automatically follows waypoints from the planned path
- Falls back to hardcoded waypoints if no path is received
- Provides progress updates as it navigates through waypoints

## Building

```bash
cd group18ws
colcon build --packages-select path_planner_cpp drone_controller drone_ui
source install/setup.bash
```

## Testing

1. Launch the Nav2 planner (if using Nav2)
2. Start the path planner node:
   ```bash
   ros2 run path_planner_cpp planner_node
   ```
3. Start the drone controller:
   ```bash
   ros2 run drone_controller drone_controller
   ```
4. Launch the drone UI:
   ```bash
   ros2 run drone_ui drone_ui_node
   ```
5. Enter goal coordinates and click "Plan Path to Goal"
6. Watch the drone follow the planned path!
