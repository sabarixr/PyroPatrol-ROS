#!/usr/bin/env python3
"""
Simple Teleop Launch - Works every time!
==========================================

This is the CORRECT way to start teleop.

Usage:
------
# With obstacle avoidance (SAFE mode - recommended):
ros2 launch frr_bringup teleop_simple.launch.py

# Without obstacle avoidance (DIRECT mode):
ros2 launch frr_bringup teleop_simple.launch.py avoidance:=false

Features:
---------
✓ Clean, organized terminal
✓ Works in launch files (proper TTY handling)
✓ Obstacle avoidance integration
✓ Real-time status display
✓ Simple controls

The teleop node will take over your terminal with interactive controls.
Press Ctrl+C to exit cleanly.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Launch argument
    avoidance_arg = DeclareLaunchArgument(
        'avoidance',
        default_value='true',
        description='Enable obstacle avoidance (true/false)'
    )
    
    # Obstacle avoidance node (runs in background)
    obstacle_node = Node(
        package='frr_sensors',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='log',  # Don't clutter the terminal
        parameters=[{
            'rover_width': 0.20,
            'rover_length': 0.30,
            'safety_margin': 0.15,
            'stop_distance': 0.30,
        }],
        condition=IfCondition(LaunchConfiguration('avoidance'))
    )
    
    # Teleop node WITH avoidance (publishes to /cmd_vel_teleop)
    teleop_with_avoidance = Node(
        package='frr_control',
        executable='teleop_node_clean',
        name='teleop_node',
        output='screen',
        prefix='xterm -e',  # Run in dedicated terminal window
        parameters=[{
            'with_avoidance': True,
        }],
        condition=IfCondition(LaunchConfiguration('avoidance'))
    )
    
    # Teleop node WITHOUT avoidance (publishes directly to /cmd_vel)
    teleop_direct = Node(
        package='frr_control',
        executable='teleop_node_clean',
        name='teleop_node',
        output='screen',
        prefix='xterm -e',  # Run in dedicated terminal window
        parameters=[{
            'with_avoidance': False,
        }],
        condition=UnlessCondition(LaunchConfiguration('avoidance'))
    )
    
    return LaunchDescription([
        avoidance_arg,
        obstacle_node,
        teleop_with_avoidance,
        teleop_direct,
    ])
