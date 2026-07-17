#!/usr/bin/env python3
"""
Launch file for ESP32-S3 Fire Robot System

Starts:
- ESP32 Bridge (serial communication)
- ESP32 Sensors (telemetry parsing)
- WebSocket Bridge (Flutter app connection)
- Mission Controller (autonomous modes)
- Camera Node (video streaming)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='ESP32-S3 serial port'
        ),
        DeclareLaunchArgument(
            'ws_port',
            default_value='8765',
            description='WebSocket port for Flutter app'
        ),
        
        # ESP32 Bridge Node - Serial communication with ESP32-S3
        Node(
            package='frr_control',
            executable='esp32_bridge_node',
            name='esp32_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
                'fallback_ports': ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM1'],
            }],
            respawn=True,
            respawn_delay=2.0,
        ),
        
        # ESP32 Sensors Node - Parse telemetry and publish to topics
        Node(
            package='frr_sensors',
            executable='esp32_sensors_node',
            name='esp32_sensors_node',
            output='screen',
            respawn=True,
        ),
        
        # WebSocket Bridge - Connect Flutter app
        Node(
            package='frr_control',
            executable='ws_bridge_node',
            name='ws_bridge_node',
            output='screen',
            parameters=[{
                'host': '0.0.0.0',
                'port': LaunchConfiguration('ws_port'),
            }],
            respawn=True,
        ),
        
        # Mission Controller - Autonomous modes (manual, fire seeking, aruco)
        Node(
            package='frr_control',
            executable='mission_controller',
            name='mission_controller',
            output='screen',
            respawn=True,
        ),
        
        # Camera Node - Video streaming
        Node(
            package='frr_sensors',
            executable='simple_camera_node',
            name='simple_camera_node',
            output='screen',
            parameters=[{
                'camera_id': 0,
                'fps': 15,
                'width': 640,
                'height': 480,
                'port': 8080,
            }],
            respawn=True,
        ),
    ])
