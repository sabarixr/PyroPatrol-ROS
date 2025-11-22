#!/usr/bin/env python3
"""
ESP32-Based Rover Launch File
Launches rover with ESP32 motor controller and sensor bridge
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
        description='ESP32 serial port'
    )
    
    return LaunchDescription([
        enable_video_stream_arg,
        serial_port_arg,
        
        # ESP32 Bridge Node (replaces motor_driver_node)
        Node(
            package='frr_control',
            executable='esp32_bridge_node',
            name='esp32_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
                'wheel_base': 0.2,
                'max_linear_speed': 100,  # PWM 0-100
                'max_angular_speed': 100,
                'use_sim_time': False,
            }]
        ),
        
        # ESP32 Sensor Node (reads from ESP32 telemetry)
        Node(
            package='frr_sensors',
            executable='esp32_sensor_node',
            name='esp32_sensor_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
        
        # Camera Node (still runs on Pi)
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
                'enable_streaming': LaunchConfiguration('enable_video_stream'),
                'use_sim_time': False,
            }]
        ),
        
        # Video Streamer Node (still runs on Pi)
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
            condition=lambda context: LaunchConfiguration('enable_video_stream').perform(context) == 'true'
        ),
    ])
