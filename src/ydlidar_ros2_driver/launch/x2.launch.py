from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_x2',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'frame_id': 'laser_frame',
                'lidar_type': 1,
                'device_type': 0,
                'isSingleChannel': True,
                'sample_rate': 4,
                'frequency': 6.0,
                'angle_min': -180.0,
                'angle_max': 180.0,
                'range_min': 0.12,
                'range_max': 8.0,
                'fixed_resolution': True,
                'reversion': False,
                'inverted': False,
                'auto_reconnect': True,
                'support_motor_dtr': False
            }]
        )
    ])
