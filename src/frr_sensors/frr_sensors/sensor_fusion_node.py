#!/usr/bin/env python3
"""
SLAM Mapping Node - Sensor Fusion
Combines: LIDAR + MPU6050 IMU + Wheel Odometry + ArUco markers
Uses robot_localization for EKF fusion and SLAM Toolbox for mapping
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import json
import math
import tf2_ros

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Parameters
        self.declare_parameter('wheel_diameter', 0.065)  # meters
        self.declare_parameter('wheel_base', 0.2)  # meters between wheels
        self.declare_parameter('encoder_ticks_per_rev', 20)  # IR encoder resolution
        
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.encoder_tpr = self.get_parameter('encoder_ticks_per_rev').value
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_time = self.get_clock().now()
        
        # Subscribers
        self.telemetry_sub = self.create_subscription(
            String, '/esp32_telemetry', self.telemetry_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/mpu6050', self.imu_callback, 10
        )
        
    # Publishers
    self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
    # Publish a lightweight fused pose for consumption by higher-level
    # autonomy nodes (e.g. SimpleAutonomousNode)
    self.fusion_pose_pub = self.create_publisher(PoseStamped, '/sensor_fusion/pose', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # IMU data storage
        self.imu_orientation = None
        self.imu_angular_velocity_z = 0.0
        
        self.get_logger().info('🗺️  SLAM Sensor Fusion Node started!')
        self.get_logger().info('📡 Fusing: LIDAR + MPU6050 + Wheel Encoders')

    def telemetry_callback(self, msg):
        """Receive encoder data from ESP32 and compute odometry"""
        try:
            data = json.loads(msg.data)
            
            # Get encoder counts (assuming ESP32 sends cumulative ticks)
            encoders = data.get('encoders', {})
            left_ticks = encoders.get('left', 0)
            right_ticks = encoders.get('right', 0)
            
            # Calculate odometry
            self.update_odometry(left_ticks, right_ticks)
            
        except (json.JSONDecodeError, KeyError):
            pass

    def imu_callback(self, msg):
        """Store IMU data for sensor fusion"""
        self.imu_orientation = msg.orientation
        self.imu_angular_velocity_z = msg.angular_velocity.z

    def update_odometry(self, left_ticks, right_ticks):
        """Calculate odometry from wheel encoders"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0.0 or dt > 1.0:  # Sanity check
            self.last_time = current_time
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            return
        
        # Calculate distance traveled by each wheel
        delta_left_ticks = left_ticks - self.last_left_ticks
        delta_right_ticks = right_ticks - self.last_right_ticks
        
        # Convert ticks to meters
        wheel_circumference = math.pi * self.wheel_diameter
        meters_per_tick = wheel_circumference / self.encoder_tpr
        
        left_distance = delta_left_ticks * meters_per_tick
        right_distance = delta_right_ticks * meters_per_tick
        
        # Differential drive kinematics
        center_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base
        
        # Use IMU for more accurate heading if available
        if self.imu_orientation is not None and self.imu_angular_velocity_z != 0.0:
            # Fuse encoder-based rotation with IMU gyro
            delta_theta = 0.7 * delta_theta + 0.3 * (self.imu_angular_velocity_z * dt)
        
        # Update pose
        if abs(delta_theta) < 0.001:  # Straight line
            delta_x = center_distance * math.cos(self.theta)
            delta_y = center_distance * math.sin(self.theta)
        else:  # Arc movement
            radius = center_distance / delta_theta
            delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        vx = center_distance / dt
        vth = delta_theta / dt
        
        # Publish odometry
        self.publish_odometry(current_time, vx, vth)
        
        # Update state
        self.last_time = current_time
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

    def publish_odometry(self, current_time, vx, vth):
        """Publish odometry message and TF transform"""
        # Create quaternion from yaw
        quat_z = math.sin(self.theta / 2.0)
        quat_w = math.cos(self.theta / 2.0)
        
        # Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (use IMU if available, otherwise use encoder-based theta)
        if self.imu_orientation is not None:
            odom.pose.pose.orientation = self.imu_orientation
        else:
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = quat_z
            odom.pose.pose.orientation.w = quat_w
        
        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        
        # Covariance (tune these based on your robot's characteristics)
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[35] = 0.05  # theta
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[35] = 0.05  # vth
        
        # Publish
        self.odom_pub.publish(odom)
        
        # Publish TF transform odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        if self.imu_orientation is not None:
            t.transform.rotation = self.imu_orientation
        else:
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = quat_z
            t.transform.rotation.w = quat_w
        
        self.tf_broadcaster.sendTransform(t)

        # Also publish a simple PoseStamped containing the fused pose.
        try:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time.to_msg()
            pose_msg.header.frame_id = 'odom'
            pose_msg.pose = odom.pose.pose
            self.fusion_pose_pub.publish(pose_msg)
        except Exception:
            # Keep fusion lightweight; failures here shouldn't crash the node
            pass


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SensorFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
