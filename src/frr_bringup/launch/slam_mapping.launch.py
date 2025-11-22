from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        
        # ========== ROBOT TRANSFORMS ==========
        
        # Static TF Publisher - Robot frame transforms
        Node(
            package='frr_sensors',
            executable='robot_tf_publisher',
            name='robot_tf_publisher',
            output='screen'
        ),
        
        # ========== SENSORS ==========
        
        # YDLidar X2 - LIDAR mapping and obstacle avoidance
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ydlidar_ros2_driver'),
                    'launch',
                    'x2.launch.py'
                ])
            ])
        ),
        
        # MPU6050 IMU - Orientation and angular velocity
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
        
        # Sensor Fusion - Combines LIDAR + IMU + Wheel Odometry
        Node(
            package='frr_sensors',
            executable='sensor_fusion_node',
            name='sensor_fusion',
            output='screen',
            parameters=[{
                'wheel_diameter': 0.065,  # meters
                'wheel_base': 0.2,         # meters
                'encoder_ticks_per_rev': 20,
            }]
        ),
        
        # Camera with ArUco Detection - For landmark localization
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
        
        # ESP32 Bridge - Motor control and sensor data
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
        
        # ========== SLAM TOOLBOX ==========
        
        # SLAM Toolbox - Online mapping
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',  # or 'localization'
                'map_file_name': '/home/alibaba/frr_ws/maps/firebot_map',
                'map_start_at_dock': True,
                
                # SLAM params
                'resolution': 0.05,
                'max_laser_range': 12.0,
                'minimum_time_interval': 0.5,
                'transform_publish_period': 0.02,
                'map_update_interval': 1.0,
                
                # Scan matcher params
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'minimum_travel_distance': 0.2,
                'minimum_travel_heading': 0.2,
                
                # Loop closure
                'loop_search_maximum_distance': 3.0,
                'do_loop_closing': True,
                'loop_match_minimum_chain_size': 10,
            }]
        ),
        
        # ========== VISUALIZATION ==========
        
        # Video Streamer
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
        
        # Autonomous Firebot Brain
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
