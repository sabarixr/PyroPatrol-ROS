#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Enhanced Teleop Node with Camera Control and Dynamic Speed
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
            }]
        ),
    ])
