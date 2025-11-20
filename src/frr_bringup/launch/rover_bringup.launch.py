#!/usr/bin/env python3
"""
Rover Bringup Launch File
=========================

This launch file starts all core rover systems:
- MPU6050 IMU (orientation and acceleration)
- LiDAR navigation (optional, enabled by default)
  * LiDAR scanning
  * Position tracking with triangulation (starts from 0,0)
  * Obstacle avoidance (4-corner detection)
- Camera and video streaming (optional)
- Motor control

Topic Flow:
-----------
When LiDAR is ENABLED:
  Teleop → /cmd_vel_teleop → Obstacle Avoidance → /cmd_vel → Motors
  (Teleop must be run separately with cmd_vel remapping)

When LiDAR is DISABLED:
  Teleop → /cmd_vel → Motors (direct control, no obstacle avoidance)

Usage:
------
# Full system with LiDAR and video:
ros2 launch frr_bringup rover_bringup.launch.py

# Without video (save CPU):
ros2 launch frr_bringup rover_bringup.launch.py enable_video_stream:=false

# Without LiDAR (if not connected):
ros2 launch frr_bringup rover_bringup.launch.py enable_lidar:=false

# Minimal (no video, no LiDAR):
ros2 launch frr_bringup rover_bringup.launch.py enable_video_stream:=false enable_lidar:=false

Teleop (run in separate terminal):
-----------------------------------
# With obstacle avoidance (LiDAR enabled):
ros2 run frr_control teleop_node --ros-args -r /cmd_vel:=/cmd_vel_teleop

# Without obstacle avoidance (direct control):
ros2 run frr_control teleop_node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'enable_video_stream',
            default_value='true',
            description='Enable video streaming (set to false to reduce CPU usage)'
        ),
        
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='true',
            description='Enable LiDAR and navigation features'
        ),
        
        # MPU6050 IMU Node (Center of Rover) - With vibration filtering
        Node(
            package='frr_sensors',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='log',  # Don't spam terminal
            parameters=[{
                'use_sim_time': False,
            }]
        ),
        
        # LiDAR Odometry Node (Position tracking with triangulation)
        # Uses external LiDAR (e.g. YDLidar) publishing on /scan and the
        # MPU6050 IMU for sensor fusion / triangulation corrections.
        Node(
            package='frr_sensors',
            executable='lidar_odometry_node',
            name='lidar_odometry_node',
            output='screen',
            parameters=[{
                'lidar_offset_x': 0.10,    # 10cm forward from center
                'lidar_offset_y': -0.0125,  # 1.25cm to right (avg offset)
                'use_sim_time': False,
            }],
            condition=IfCondition(LaunchConfiguration('enable_lidar'))
        ),
        
        # LiDAR is provided by an external driver (e.g. YDLidar). Do not start
        # the internal serial-based lidar node here. The external driver should
        # publish LaserScan on /scan and the obstacle avoidance node will
        # subscribe to that topic. We do, however, start the LiDAR odometry
        # node which fuses the MPU6050 IMU with incoming LaserScan data to
        # provide /lidar_odom when LiDAR is enabled.

        Node(
            package='frr_sensors',
            executable='lidar_odometry_node',
            name='lidar_odometry_node',
            output='screen',
            parameters=[{
                'lidar_offset_x': 0.10,    # 10cm forward from center
                'lidar_offset_y': -0.0125,  # 1.25cm to right (avg offset)
                'use_sim_time': False,
            }],
            condition=IfCondition(LaunchConfiguration('enable_lidar'))
        ),
        
        # Obstacle Avoidance Node (4-corner detection)
        # Monitors front, back, left, right zones
        # Topic flow: /cmd_vel_teleop (from teleop) → obstacle_avoidance → /cmd_vel (to motors)
        # Obstacle Avoidance Node
        Node(
            package='frr_sensors',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance_node',
            output='log',  # Less spam
            parameters=[{
                'rover_width': 0.20,
                'rover_length': 0.30,
                'safety_margin': 0.25,  # Increased from 0.15
                'stop_distance': 0.35,  # Increased from 0.30
            }],
            condition=IfCondition(LaunchConfiguration('enable_lidar'))
        ),
        
        # Camera Node (Optimized for Performance)
        Node(
            package='frr_sensors',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_id': 0,
                'frame_width': 320,      # Reduced for less lag
                'frame_height': 240,     # Reduced for less lag
                'fps': 15,               # Reduced FPS for stability
                'enable_streaming': LaunchConfiguration('enable_video_stream'),
                'use_sim_time': False,
            }]
        ),
        
        # Motor Driver Node (L298N H-Bridge with Encoders)
        Node(
            package='frr_control',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen',  # Keep this one visible
            parameters=[{
                'max_speed': 2.5,  # Increased to 2.5 m/s as requested!
                'max_angular_speed': 3.0,
                'wheel_base': 0.2,
                'enable_feedback_control': True,
                'feedback_gain': 0.3,
                'servo_min_angle': -90.0,
                'servo_max_angle': 90.0,
                'servo_center_angle': 0.0,
            }]
        ),
        
        # Video Streamer Node (Optimized for Performance)
        Node(
            package='frr_video',
            executable='streamer_node',
            name='streamer_node',
            output='screen',
            parameters=[{
                'port': 8080,
                'host': '0.0.0.0',
                'jpeg_quality': 70,      # Reduced quality for better performance
                'buffer_size': 2,        # Smaller buffer
                'use_sim_time': False,
            }],
            condition=IfCondition(LaunchConfiguration('enable_lidar'))
        ),
    ])
