from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Declare map file argument
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/alibaba/frr_ws/maps/firebot_map.posegraph',
        description='Path to the saved map file'
    )
    
    return LaunchDescription([
        map_file_arg,
        
        # ========== ROBOT TRANSFORMS ==========
        
        # Static TF Publisher - Robot frame transforms
        Node(
            package='frr_sensors',
            executable='robot_tf_publisher',
            name='robot_tf_publisher',
            output='screen'
        ),
        
        # ========== SENSORS ==========
        
        # YDLidar X2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ydlidar_ros2_driver'),
                    'launch',
                    'x2.launch.py'
                ])
            ])
        ),
        
        # MPU6050 IMU
        Node(
            package='frr_sensors',
            executable='mpu6050_node',
            name='mpu6050',
            output='screen',
            parameters=[{
                'use_vibration_filter': True,
                'filter_window_size': 5,
            }]
        ),
        
        # Sensor Fusion
        Node(
            package='frr_sensors',
            executable='sensor_fusion_node',
            name='sensor_fusion',
            output='screen',
            parameters=[{
                'wheel_diameter': 0.065,
                'wheel_base': 0.2,
                'encoder_ticks_per_rev': 20,
            }]
        ),
        
        # Camera with ArUco Detection
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
            }]
        ),
        
        # ========== ESP32 COMMUNICATION ==========
        
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
        
        # ========== SLAM TOOLBOX - LOCALIZATION MODE ==========
        
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'localization',
                'map_file_name': LaunchConfiguration('map_file'),
                
                # Localization params
                'resolution': 0.05,
                'max_laser_range': 12.0,
                'transform_publish_period': 0.02,
                
                # Scan matcher
                'use_scan_matching': True,
                'use_scan_barycenter': True,
            }]
        ),
        
        # ========== VISUALIZATION ==========
        
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
        
        # ========== AUTONOMOUS NAVIGATION ==========
        
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
    ])
