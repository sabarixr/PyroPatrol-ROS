from launch import LaunchDescription
from launch import conditions
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Main rover bringup - launches everything you need
    - Camera with ArUco detection (FIXED INVERSION)
    - LIDAR for obstacle avoidance
    - ESP32 motor control
    - ArUco follower (moves 2s when marker detected)
    - Video streaming
    """
    
    # Declare arguments
    enable_video_arg = DeclareLaunchArgument(
        'enable_video_stream',
        default_value='true',
        description='Enable video streaming server'
    )
    
    enable_aruco_arg = DeclareLaunchArgument(
        'enable_aruco_follower',
        default_value='false',
        description='Enable ArUco following behavior (disable for manual control)'
    )
    
    enable_fire_arg = DeclareLaunchArgument(
        'enable_fire_seeking',
        default_value='true',
        description='Enable fire detection and seeking'
    )
    
    return LaunchDescription([
        enable_video_arg,
        enable_aruco_arg,
        enable_fire_arg,
        
        # ========== CAMERA (WITH FIXED INVERSION) ==========
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
        
        # ========== ESP32 BRIDGE (Motor Control + Fire Sensors) ==========
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
        
        # ========== TELEOP (Manual Control) ==========
        Node(
            package='frr_navigation',
            executable='esp32_teleop_node',
            name='teleop',
            output='screen',
            prefix='xterm -e',  # Opens in new terminal window
        ),
        
        # ========== FIRE SEEKING (Autonomous) ==========
        Node(
            package='frr_navigation',
            executable='autonomous_firebot_node',
            name='fire_seeker',
            output='screen',
            parameters=[{
                'max_speed': 0.3,
                'min_obstacle_distance': 0.5,
                'fire_threshold_mq2': 600,
                'fire_threshold_mq5': 550,
                'temp_rise_threshold': 2.0,
            }],
            condition=conditions.IfCondition(
                LaunchConfiguration('enable_fire_seeking')
            )
        ),
        
        # ========== ARUCO FOLLOWER ==========
        Node(
            package='frr_navigation',
            executable='aruco_follower_node',
            name='aruco_follower',
            output='screen',
            parameters=[{
                'forward_speed': 0.2,           # Speed when moving
                'move_duration': 2.0,           # Move for 2 seconds
                'min_obstacle_distance': 0.5,   # Stop if obstacle within 0.5m
            }],
            condition=conditions.IfCondition(
                LaunchConfiguration('enable_aruco_follower')
            )
        ),
        
        # ========== VIDEO STREAMER ==========
        Node(
            package='frr_video',
            executable='streamer_node',
            name='streamer_node',
            output='screen',
            parameters=[{
                'port': 8080,
                'host': '0.0.0.0',
                'quality': 80,
            }],
            condition=conditions.IfCondition(
                LaunchConfiguration('enable_video_stream')
            )
        ),
    ])
