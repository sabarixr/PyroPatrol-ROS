#!/usr/bin/env python3
"""
ESP32-S3 Based Rover Launch File
Launches rover with ESP32-S3 motor controller, sensors, and WebSocket bridge
Updated for ESP32-S3 N16R8 with full telemetry support and optional LIDAR
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    enable_video_stream_arg = DeclareLaunchArgument(
        'enable_video_stream',
        default_value='true',
        description='Enable video streaming'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='ESP32-S3 serial port'
    )

    ws_port_arg = DeclareLaunchArgument(
        'ws_port',
        default_value='8765',
        description='WebSocket server port for Flutter app'
    )

    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='false',
        description='Enable YDLidar X2'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='LIDAR serial port'
    )

    # Get ydlidar launch file path
    ydlidar_pkg = get_package_share_directory('ydlidar_ros2_driver')
    ydlidar_launch = os.path.join(ydlidar_pkg, 'launch', 'x2.launch.py')

    return LaunchDescription([
        enable_video_stream_arg,
        serial_port_arg,
        ws_port_arg,
        enable_lidar_arg,
        lidar_port_arg,

        # ESP32-S3 Bridge Node (handles motor control + command forwarding)
        Node(
            package='frr_control',
            executable='esp32_bridge_node',
            name='esp32_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
                'wheel_base': 0.15,  # Updated for actual wheelbase
                'max_linear_speed': 100,  # PWM 0-100
                'max_angular_speed': 100,
                'use_sim_time': False,
            }]
        ),

        # ESP32-S3 Sensor Node (processes all ESP32-S3 telemetry)
        Node(
            package='frr_sensors',
            executable='esp32_sensors_node',
            name='esp32_sensors_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),

        # Ultrasonic Sensor Node (HC-SR04 on Raspberry Pi GPIO - backup when LIDAR unavailable)
        Node(
            package='frr_sensors',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen',
            parameters=[{
                'trigger_pin': 23,  # GPIO 23 (Pin 16)
                'echo_pin': 24,     # GPIO 24 (Pin 18)
                'publish_rate': 10.0,
                'use_sim_time': False,
            }]
        ),

        # Mission Controller (handles autonomous modes)
        Node(
            package='frr_control',
            executable='mission_controller',
            name='mission_controller',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),

        # WebSocket Bridge (Flutter app communication)
        Node(
            package='frr_control',
            executable='ws_bridge_node',
            name='ws_bridge_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('ws_port'),
                'use_sim_time': False,
            }]
        ),

        # Camera Node (runs on Pi)
        Node(
            package='frr_sensors',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_id': 0,
                'frame_width': 640,
                'frame_height': 480,
                'fps': 30,
                'enable_streaming': True,
                'jpeg_quality': 80,
                'use_sim_time': False,
            }]
        ),

        # Video Streamer Node (runs on Pi)
        Node(
            package='frr_video',
            executable='streamer_node',
            name='streamer_node',
            output='screen',
            parameters=[{
                'port': 8080,
                'quality': 85,
                'use_sim_time': False,
            }],
            condition=IfCondition(LaunchConfiguration('enable_video_stream'))
        ),

        # YDLidar X2 Node (optional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ydlidar_launch),
            launch_arguments={
                'port': LaunchConfiguration('lidar_port')
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_lidar'))
        ),
    ])
