#!/usr/bin/env python3
"""
Launch file for drone camera-based tree detection system.

Starts the tree detector node with configurable parameters.
Can be included in main drone launch file or run standalone.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    # Tree detector node
    tree_detector_node = Node(
        package='drone_colour_detector',
        executable='tree_detector',
        name='drone_tree_detector',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'image_topic': LaunchConfiguration('image_topic'),

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
        tree_detector_node,
    ])
