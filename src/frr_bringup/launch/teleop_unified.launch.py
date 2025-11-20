#!/usr/bin/env python3
"""
Unified Teleop Launch File
==========================

Simple, clean way to start teleop with or without obstacle avoidance.

Usage:
------
# With obstacle avoidance (RECOMMENDED):
ros2 launch frr_bringup teleop_unified.launch.py

# Without obstacle avoidance (direct control):
ros2 launch frr_bringup teleop_unified.launch.py avoidance:=false

Features:
---------
- Clean, organized terminal output
- Automatic topic remapping based on mode
- Real-time obstacle detection status
- Single command to start everything needed
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch arguments
    avoidance_arg = DeclareLaunchArgument(
        'avoidance',
        default_value='true',
        description='Enable obstacle avoidance (true/false)'
    )
    
    # Obstacle avoidance node (only if enabled)
    obstacle_avoidance_node = Node(
        package='frr_sensors',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='log',  # Don't clutter terminal
        parameters=[{
            'rover_width': 0.20,
            'rover_length': 0.30,
            'safety_margin': 0.15,
            'stop_distance': 0.30,
        }],
        condition=IfCondition(LaunchConfiguration('avoidance'))
    )
    
    # Teleop node (clean version)
    teleop_node = Node(
        package='frr_control',
        executable='teleop_node_clean',
        name='teleop_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'with_avoidance': LaunchConfiguration('avoidance'),
        }]
    )
    
    return LaunchDescription([
        avoidance_arg,
        obstacle_avoidance_node,
        teleop_node,
    ])
