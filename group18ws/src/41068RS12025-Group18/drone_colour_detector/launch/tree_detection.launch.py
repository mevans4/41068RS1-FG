#!/usr/bin/env python3
"""
Launch file for drone camera-based tree detection system.

Starts the tree detector node with configurable parameters.
Can be included in main drone launch file or run standalone.

Features:
- Real-time tree detection with HSV color segmentation
- Data logging to CSV/JSON with world coordinates
- Cumulative statistics tracking
- Mission report generation on shutdown
- Waypoint export for unhealthy trees
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime


def generate_launch_description():
    """Generate launch description for tree detection system."""

    # Declare launch arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image',
        description='Camera image topic to subscribe to'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )

    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='True',
        description='Enable data logging to files'
    )

    log_directory_arg = DeclareLaunchArgument(
        'log_directory',
        default_value='~/drone_tree_logs',
        description='Directory for log files'
    )

    mission_name_arg = DeclareLaunchArgument(
        'mission_name',
        default_value=f'mission_{datetime.now().strftime("%Y%m%d_%H%M%S")}',
        description='Name for this mission (used in filenames)'
    )

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='map',
        description='Target coordinate frame for world positions (map, odom, or world)'
    )

    # Tree detector node
    tree_detector_node = Node(
        package='drone_colour_detector',
        executable='tree_detector',
        name='drone_tree_detector',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'image_topic': LaunchConfiguration('image_topic'),

            # Data logging parameters
            'enable_logging': LaunchConfiguration('enable_logging'),
            'log_directory': LaunchConfiguration('log_directory'),
            'mission_name': LaunchConfiguration('mission_name'),
            'target_frame': LaunchConfiguration('target_frame'),
            'save_format': 'csv',  # csv or json
            'enable_waypoints': True,

            # HSV thresholds (adjust these based on calibration)
            'green_low': '40,80,40',
            'green_high': '90,255,255',
            'red1_low': '0,100,40',
            'red1_high': '15,255,255',
            'red2_low': '165,100,40',
            'red2_high': '179,255,255',
            'brown_low': '10,40,40',
            'brown_high': '30,200,200',

            # Morphological operations
            'kernel': 7,
            'open_iters': 2,
            'close_iters': 3,

            # Detection filters
            'roi_ymin': 0.1,
            'min_area_px': 800,
            'aspect_min': 0.8,
            'aspect_max': 1.5,

            # Tracking
            'track_timeout': 2.0,
            'dist_thresh_px': 80.0,
        }],
        remappings=[
            # Remap topics if needed
        ]
    )

    return LaunchDescription([
        image_topic_arg,
        use_sim_time_arg,
        enable_logging_arg,
        log_directory_arg,
        mission_name_arg,
        target_frame_arg,
        tree_detector_node,
    ])
