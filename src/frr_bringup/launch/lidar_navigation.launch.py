#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MPU6050 IMU Node (only working IMU)
        Node(
            package='frr_sensors',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
        
        # LiDAR is expected to be provided by an external driver (e.g. YDLidar)
        # We run LiDAR odometry here (it subscribes to /scan and uses the
        # MPU6050 IMU for fusion/triangulation) and the obstacle avoidance node.
        Node(
            package='frr_sensors',
            executable='lidar_odometry_node',
            name='lidar_odometry_node',
            output='screen',
            parameters=[{
                'lidar_offset_x': 0.10,
                'lidar_offset_y': -0.0125,
            }]
        ),

        # Obstacle Avoidance Node
        Node(
            package='frr_sensors',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance_node',
            output='screen',
            parameters=[{
                'rover_width': 0.20,       # 20cm width
                'rover_length': 0.30,      # 30cm length
                'safety_margin': 0.15,     # 15cm safety margin
                'stop_distance': 0.30,     # 30cm emergency stop
            }]
        ),
    ])
