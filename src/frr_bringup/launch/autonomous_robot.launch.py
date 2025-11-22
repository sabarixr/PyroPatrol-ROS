from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'enable_video_stream',
            default_value='true',
            description='Enable video streaming'
        ),
        
        # YDLidar X2 - LIDAR for obstacle avoidance
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ydlidar_ros2_driver'),
                    'launch',
                    'x2.launch.py'
                ])
            ])
        ),
        
        # ESP32 Bridge (Muscle - executes commands)
        Node(
            package='frr_control',
            executable='esp32_bridge_node',
            name='esp32_bridge',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baud_rate': 115200,
                'wheel_base': 0.2,
                'max_linear_speed': 100,
                'max_angular_speed': 100,
            }]
        ),
        
        # Autonomous Firebot Brain (makes intelligent decisions)
        Node(
            package='frr_navigation',
            executable='autonomous_firebot_node',
            name='autonomous_firebot',
            output='screen',
            parameters=[{
                'max_speed': 0.3,
                'min_obstacle_distance': 0.5,
                'fire_threshold_mq2': 600,
                'fire_threshold_mq5': 550,
                'temp_rise_threshold': 2.0,
            }]
        ),
        
        # Camera Node (Raspberry Pi handles vision)
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
        
        # Video Streamer (FPV camera feed)
        Node(
            package='frr_video',
            executable='streamer_node',
            name='streamer_node',
            output='screen',
            parameters=[{
                'port': 8080,
                'quality': 80,
                'use_sim_time': False,
            }],
            condition=lambda context: context.launch_configurations['enable_video_stream'] == 'true'
        ),
    ])