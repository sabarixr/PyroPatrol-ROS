#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
import math
import numpy as np
import time

class LidarOdometryNode(Node):
    def __init__(self):
        super().__init__('lidar_odometry_node')
        
        # LiDAR offset from rover center (in meters)
        # Back: 16cm, Left: 8cm, Right: 10.5cm, Front: 10cm
        self.declare_parameter('lidar_offset_x', 0.10)  # 10cm forward from center
        self.declare_parameter('lidar_offset_y', -0.0125)  # 1.25cm to the right (avg of left/right)
        
        self.lidar_offset_x = self.get_parameter('lidar_offset_x').value
        self.lidar_offset_y = self.get_parameter('lidar_offset_y').value
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/mpu6050', self.imu_callback, 10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/lidar_odom', 10)
        
        # State variables - starting from (0, 0)
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_yaw = 0.0
        
        # IMU data
        self.gyro_z = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        
        # Previous scan for motion estimation
        self.prev_scan = None
        self.prev_time = time.time()
        
        # Reference points for triangulation (fixed landmarks)
        self.reference_points = []  # Will be populated from first scan
        self.reference_initialized = False
        
        # Velocity estimation
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        
        self.get_logger().info('LiDAR Odometry Node started with triangulation')
        self.get_logger().info(f'LiDAR offset: x={self.lidar_offset_x:.3f}m, y={self.lidar_offset_y:.3f}m')

    def imu_callback(self, msg):
        """Receive IMU data for orientation."""
        self.gyro_z = msg.angular_velocity.z
        self.accel_x = msg.linear_acceleration.x / 9.81  # Convert to g
        self.accel_y = msg.linear_acceleration.y / 9.81

    def scan_callback(self, msg):
        """Process LiDAR scan for odometry estimation."""
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt < 0.001:  # Avoid division by zero
            return
        
        # Update orientation from gyroscope integration
        self.orientation_yaw += self.gyro_z * dt
        
        # Normalize angle to [-pi, pi]
        while self.orientation_yaw > math.pi:
            self.orientation_yaw -= 2 * math.pi
        while self.orientation_yaw < -math.pi:
            self.orientation_yaw += 2 * math.pi
        
        # Extract key features from scan
        features = self.extract_features(msg)
        
        # Initialize reference points on first scan
        if not self.reference_initialized and len(features) > 3:
            self.reference_points = self.transform_features_to_global(features)
            self.reference_initialized = True
            self.get_logger().info(f'Initialized {len(self.reference_points)} reference points')
        
        # Estimate motion using scan matching and triangulation
        if self.prev_scan is not None:
            dx, dy = self.estimate_motion(self.prev_scan, msg, dt)
            
            # Transform motion to global frame
            cos_yaw = math.cos(self.orientation_yaw)
            sin_yaw = math.sin(self.orientation_yaw)
            
            dx_global = dx * cos_yaw - dy * sin_yaw
            dy_global = dx * sin_yaw + dy * cos_yaw
            
            # Update position
            self.position_x += dx_global
            self.position_y += dy_global
            
            # Update velocity
            self.velocity_x = dx_global / dt
            self.velocity_y = dy_global / dt
        
        # Use triangulation to correct position drift
        if self.reference_initialized and len(features) >= 3:
            corrected_pos = self.triangulate_position(features)
            if corrected_pos is not None:
                # Apply correction with low-pass filter
                alpha = 0.3  # Weight for triangulation correction
                self.position_x = (1 - alpha) * self.position_x + alpha * corrected_pos[0]
                self.position_y = (1 - alpha) * self.position_y + alpha * corrected_pos[1]
        
        # Publish odometry
        self.publish_odometry()
        
        # Store current scan for next iteration
        self.prev_scan = msg
        self.prev_time = current_time

    def extract_features(self, scan):
        """Extract point features from laser scan."""
        features = []
        
        for i, r in enumerate(scan.ranges):
            # Filter invalid ranges
            if r < scan.range_min or r > scan.range_max or math.isnan(r) or math.isinf(r):
                continue
            
            # Calculate angle for this measurement
            angle = scan.angle_min + i * scan.angle_increment
            
            # Convert to Cartesian coordinates (in LiDAR frame)
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            # Only keep significant features (corners, edges)
            # Simple edge detection: look for large range differences
            if i > 0 and i < len(scan.ranges) - 1:
                prev_r = scan.ranges[i - 1]
                next_r = scan.ranges[i + 1]
                
                if not (math.isnan(prev_r) or math.isnan(next_r)):
                    # Detect edges (large discontinuities)
                    if abs(r - prev_r) > 0.3 or abs(r - next_r) > 0.3:
                        features.append((x, y, r, angle))
        
        return features

    def transform_features_to_global(self, features):
        """Transform features from LiDAR frame to global frame."""
        global_features = []
        
        cos_yaw = math.cos(self.orientation_yaw)
        sin_yaw = math.sin(self.orientation_yaw)
        
        for x, y, r, angle in features:
            # Compensate for LiDAR offset
            x_rover = x + self.lidar_offset_x
            y_rover = y + self.lidar_offset_y
            
            # Transform to global frame
            x_global = self.position_x + x_rover * cos_yaw - y_rover * sin_yaw
            y_global = self.position_y + x_rover * sin_yaw + y_rover * cos_yaw
            
            global_features.append((x_global, y_global))
        
        return global_features

    def estimate_motion(self, prev_scan, curr_scan, dt):
        """Estimate motion between consecutive scans using ICP-like approach."""
        # Simple centroid-based motion estimation
        prev_points = self.scan_to_points(prev_scan)
        curr_points = self.scan_to_points(curr_scan)
        
        if len(prev_points) == 0 or len(curr_points) == 0:
            return 0.0, 0.0
        
        # Calculate centroids
        prev_centroid = np.mean(prev_points, axis=0)
        curr_centroid = np.mean(curr_points, axis=0)
        
        # Motion is the difference in centroids
        dx = curr_centroid[0] - prev_centroid[0]
        dy = curr_centroid[1] - prev_centroid[1]
        
        # Compensate for LiDAR offset
        dx -= self.lidar_offset_x * 0.01  # Small correction
        dy -= self.lidar_offset_y * 0.01
        
        return dx, dy

    def scan_to_points(self, scan):
        """Convert LaserScan to point cloud."""
        points = []
        
        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max or math.isnan(r) or math.isinf(r):
                continue
            
            angle = scan.angle_min + i * scan.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append([x, y])
        
        return np.array(points) if len(points) > 0 else np.array([])

    def triangulate_position(self, features):
        """Use triangulation with reference points to estimate position."""
        if len(self.reference_points) < 3 or len(features) < 3:
            return None
        
        # Match features to reference points (simple nearest neighbor)
        matches = []
        
        for fx, fy, fr, fangle in features:
            # Transform feature to global estimate
            cos_yaw = math.cos(self.orientation_yaw)
            sin_yaw = math.sin(self.orientation_yaw)
            
            fx_rover = fx + self.lidar_offset_x
            fy_rover = fy + self.lidar_offset_y
            
            fx_global = self.position_x + fx_rover * cos_yaw - fy_rover * sin_yaw
            fy_global = self.position_y + fx_rover * sin_yaw + fy_rover * cos_yaw
            
            # Find nearest reference point
            min_dist = float('inf')
            best_ref = None
            
            for ref_x, ref_y in self.reference_points:
                dist = math.sqrt((fx_global - ref_x)**2 + (fy_global - ref_y)**2)
                if dist < min_dist and dist < 1.0:  # Within 1 meter
                    min_dist = dist
                    best_ref = (ref_x, ref_y, fr, fangle)
            
            if best_ref is not None:
                matches.append(best_ref)
        
        if len(matches) < 3:
            return None
        
        # Triangulate position using matched landmarks
        # Use first 3 matches for triangulation
        positions = []
        
        for i in range(min(3, len(matches))):
            ref_x, ref_y, distance, angle = matches[i]
            
            # Calculate position based on this landmark
            # The rover is at distance 'distance' from landmark at 'angle'
            angle_global = self.orientation_yaw + angle
            
            pos_x = ref_x - distance * math.cos(angle_global)
            pos_y = ref_y - distance * math.sin(angle_global)
            
            positions.append((pos_x, pos_y))
        
        # Average the positions
        avg_x = sum(p[0] for p in positions) / len(positions)
        avg_y = sum(p[1] for p in positions) / len(positions)
        
        return (avg_x, avg_y)

    def publish_odometry(self):
        """Publish odometry message."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        qz = math.sin(self.orientation_yaw / 2.0)
        qw = math.cos(self.orientation_yaw / 2.0)
        
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # Velocity
        odom_msg.twist.twist.linear.x = self.velocity_x
        odom_msg.twist.twist.linear.y = self.velocity_y
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = self.gyro_z
        
        # Covariance (rough estimates)
        odom_msg.pose.covariance[0] = 0.05   # x
        odom_msg.pose.covariance[7] = 0.05   # y
        odom_msg.pose.covariance[35] = 0.03  # yaw
        
        odom_msg.twist.covariance[0] = 0.02   # vx
        odom_msg.twist.covariance[7] = 0.02   # vy
        odom_msg.twist.covariance[35] = 0.01  # vyaw
        
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        lidar_odom_node = LidarOdometryNode()
        rclpy.spin(lidar_odom_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
