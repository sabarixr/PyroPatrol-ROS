from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Simple ArUco follower launch - no SLAM, just obstacle avoidance + ArUco detection
    
    What it does:
    1. Starts camera with ArUco detection (fixed orientation)
    2. Starts LIDAR for obstacle avoidance
    3. Runs ArUco follower node (moves forward 2s when marker detected)
    4. Connects to ESP32 for motor control
    5. (Optional) Video streaming
    """
    
    return LaunchDescription([
        
        # ========== CAMERA ==========
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
            }]
        ),
        
        # ========== LIDAR (Obstacle Detection) ==========
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ydlidar_ros2_driver'),
                    'launch',
                    'x2.launch.py'
                ])
            ])
        ),
        
        # ========== ESP32 BRIDGE ==========
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
        
        # ========== ARUCO FOLLOWER ==========
        Node(
            package='frr_navigation',
            executable='aruco_follower_node',
            name='aruco_follower',
            output='screen',
            parameters=[{
                'forward_speed': 0.2,      # Speed in m/s
                'move_duration': 2.0,      # Move for 2 seconds
                'min_obstacle_distance': 0.5,  # Stop if obstacle within 0.5m
            }]
        ),
        
        # ========== VIDEO STREAMER (Optional) ==========
        Node(
            package='frr_video',
            executable='streamer_node',
            name='streamer_node',
            output='screen',
            parameters=[{
                'port': 8080,
                'host': '0.0.0.0',
                'quality': 80,
            }]
        ),
    ])
