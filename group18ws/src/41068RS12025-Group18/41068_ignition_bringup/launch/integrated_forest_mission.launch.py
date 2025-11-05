#!/usr/bin/env python3
"""
Integrated Forest Mission Launch File

Brings together all system components for autonomous forest inventory:
- Ignition Gazebo simulation with plantation world
- Drone model with LIDAR and camera sensors
- LIDAR-based tree detection and measurement
- Camera-based tree health detection (optional)
- Mission orchestrator for autonomous navigation
- User interface for control and monitoring
- RViz for visualization

This launch file provides a complete, integrated system that meets all MVP requirements.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    bringup_dir = get_package_share_directory('41068_ignition_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    rviz = LaunchConfiguration('rviz', default='True')
    world = LaunchConfiguration('world', default='simple_trees')
    enable_camera_detection = LaunchConfiguration('camera_detection', default='False')
    enable_ui = LaunchConfiguration('ui', default='True')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz for visualization'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        choices=['simple_trees', 'large_demo', 'PlantationTest', 'Plantation2'],
        description='Gazebo world to load'
    )

    declare_camera_detection = DeclareLaunchArgument(
        'camera_detection',
        default_value='False',
        description='Enable camera-based tree detection (requires camera sensor)'
    )

    declare_ui = DeclareLaunchArgument(
        'ui',
        default_value='True',
        description='Launch the drone control UI'
    )

    # Include the main drone simulation launch file
    # This handles: Gazebo, robot spawning, sensors, robot_state_publisher, etc.
    drone_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('41068_ignition_bringup'),
                'launch',
                '41068_ignition_drone.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'nav2': 'False',  # We'll use our custom controller instead of Nav2
            'color_detection': enable_camera_detection,
            'world': world,
        }.items()
    )

    # LIDAR Tree Detector Node (critical for tree measurement)
    lidar_tree_detector = Node(
        package='lidar_tree_detector',
        executable='lidar_tree_detector_node',
        name='lidar_tree_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/scan', '/scan'),
            ('/odometry', '/odometry'),
        ]
    )

    # Forest Mission Orchestrator (autonomous navigation & mission management)
    mission_orchestrator = Node(
        package='drone_controller',
        executable='forest_mission_orchestrator',
        name='forest_mission_orchestrator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odometry', '/odometry'),
        ]
    )

    # Altitude Controller (maintains stable flight altitude)
    altitude_controller = Node(
        package='drone_controller',
        executable='altitude_controller',
        name='altitude_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition('False'),  # Disabled - mission orchestrator handles altitude
        remappings=[
            ('/cmd_vel_nav', '/cmd_vel_nav'),
            ('/cmd_vel', '/cmd_vel'),
            ('/odometry', '/odometry'),
        ]
    )

    # Drone UI (control panel for operators)
    drone_ui = Node(
        package='drone_ui',
        executable='drone_ui_node',
        name='drone_ui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_ui)
    )

    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_rviz,
        declare_world,
        declare_camera_detection,
        declare_ui,

        # Core simulation
        drone_simulation,

        # Perception
        lidar_tree_detector,

        # Control & Mission
        mission_orchestrator,
        altitude_controller,

        # User Interface
        drone_ui,
    ])
