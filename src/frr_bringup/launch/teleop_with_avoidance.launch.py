#!/usr/bin/env python3
"""
Teleop Launch File with Obstacle Avoidance Support
===================================================

This launch file starts the teleop node with proper topic remapping
for obstacle avoidance integration.

Topic Flow:
-----------
When used with obstacle avoidance (default):
  Teleop → /cmd_vel_teleop → Obstacle Avoidance → /cmd_vel → Motors

When used standalone (no_remap:=true):
  Teleop → /cmd_vel → Motors (direct control)

Usage:
------
# With obstacle avoidance (recommended):
ros2 launch frr_bringup teleop_with_avoidance.launch.py

# Direct control (no obstacle avoidance):
ros2 launch frr_bringup teleop_with_avoidance.launch.py no_remap:=true

# Original teleop (no remapping):
ros2 launch frr_bringup teleop.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import conditions

def generate_launch_description():
    return LaunchDescription([
        # Launch argument to control remapping
        DeclareLaunchArgument(
            'no_remap',
            default_value='false',
            description='Set to true to disable cmd_vel remapping (direct motor control)'
        ),
        
        # Teleop Node with remapping for obstacle avoidance
        Node(
            package='frr_control',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'linear_speed': 0.5,
                'angular_speed': 1.0,
                'speed_increment': 0.1,
                'servo_angle_increment': 5.0,
                'use_sim_time': False,
            }],
            remappings=[
                ('/cmd_vel', '/cmd_vel_teleop'),
            ],
            condition=conditions.UnlessCondition(LaunchConfiguration('no_remap'))
        ),
        
        # Teleop Node without remapping (direct control)
        Node(
            package='frr_control',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'linear_speed': 0.5,
                'angular_speed': 1.0,
                'speed_increment': 0.1,
                'servo_angle_increment': 5.0,
                'use_sim_time': False,
            }],
            condition=conditions.IfCondition(LaunchConfiguration('no_remap'))
        ),
    ])
