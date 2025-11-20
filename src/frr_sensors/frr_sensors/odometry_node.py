#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32MultiArray
import math
import time

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Note: MMA8452 accelerometer removed (sensor burned/not working)
        # Only using MPU6050 for odometry now
        
        # Subscribers
        self.mpu6050_sub = self.create_subscription(
            Imu, '/imu/mpu6050', self.mpu6050_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.velocity_error_pub = self.create_publisher(Vector3, '/velocity_error', 10)
        self.speed_correction_pub = self.create_publisher(Float32MultiArray, '/speed_correction', 10)
        
        # State variables
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        
        self.orientation_z = 0.0  # Yaw angle from gyro integration
        
        # Commanded velocity (from teleop)
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        
        # Latest sensor readings (MPU6050 only)
        self.mpu6050_accel_x = 0.0
        self.mpu6050_accel_y = 0.0
        self.mpu6050_gyro_z = 0.0
        
        # Timing
        self.last_time = time.time()
        
        # Timer for odometry computation (50 Hz)
        self.timer = self.create_timer(0.02, self.compute_odometry)
        
        self.get_logger().info('Odometry node started (MPU6050 only)')

    def mpu6050_callback(self, msg):
        """Receive MPU6050 data (center of rover)"""
        self.mpu6050_accel_x = msg.linear_acceleration.x / 9.81  # Convert to g
        self.mpu6050_accel_y = msg.linear_acceleration.y / 9.81
        self.mpu6050_gyro_z = msg.angular_velocity.z

    def cmd_vel_callback(self, msg):
        """Receive commanded velocity from teleop"""
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z

    def compute_odometry(self):
        """Compute odometry from fused IMU data"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0.1:  # Prevent large jumps
            dt = 0.02
            
        # Use only MPU6050 accelerometer (MMA8452 is not working)
        fused_accel_x = self.mpu6050_accel_x
        fused_accel_y = self.mpu6050_accel_y
        
        # Integrate gyroscope for orientation
        self.orientation_z += self.mpu6050_gyro_z * dt
        
        # Normalize angle to [-pi, pi]
        while self.orientation_z > math.pi:
            self.orientation_z -= 2 * math.pi
        while self.orientation_z < -math.pi:
            self.orientation_z += 2 * math.pi
        
        # Integrate acceleration to velocity (with drift compensation)
        decay = 0.99  # Prevents unbounded drift
        self.velocity_x = (self.velocity_x + fused_accel_x * 9.81 * dt) * decay
        self.velocity_y = (self.velocity_y + fused_accel_y * 9.81 * dt) * decay
        
        # Integrate velocity to position
        self.position_x += self.velocity_x * dt
        self.position_y += self.velocity_y * dt
        
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (from gyro integration)
        # Convert yaw to quaternion
        qz = math.sin(self.orientation_z / 2.0)
        qw = math.cos(self.orientation_z / 2.0)
        
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # Velocity in body frame
        odom_msg.twist.twist.linear.x = self.velocity_x
        odom_msg.twist.twist.linear.y = self.velocity_y
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = self.mpu6050_gyro_z
        
        # Set covariance (rough estimates)
        odom_msg.pose.covariance[0] = 0.1   # x position variance
        odom_msg.pose.covariance[7] = 0.1   # y position variance
        odom_msg.pose.covariance[35] = 0.05 # yaw variance
        
        odom_msg.twist.covariance[0] = 0.05  # x velocity variance
        odom_msg.twist.covariance[7] = 0.05  # y velocity variance
        odom_msg.twist.covariance[35] = 0.03 # angular velocity variance
        
        self.odom_pub.publish(odom_msg)
        
        # Compute velocity error (commanded vs actual)
        velocity_error_x = self.cmd_linear_x - self.velocity_x
        velocity_error_angular = self.cmd_angular_z - self.mpu6050_gyro_z
        
        error_msg = Vector3()
        error_msg.x = velocity_error_x
        error_msg.y = 0.0
        error_msg.z = velocity_error_angular
        self.velocity_error_pub.publish(error_msg)
        
        # Compute speed correction factors (for motor controller)
        # If actual speed is lower than commanded, need to increase motor power
        if abs(self.cmd_linear_x) > 0.05:
            linear_correction = velocity_error_x / self.cmd_linear_x
        else:
            linear_correction = 0.0
            
        if abs(self.cmd_angular_z) > 0.05:
            angular_correction = velocity_error_angular / self.cmd_angular_z
        else:
            angular_correction = 0.0
        
        # Limit correction factors
        linear_correction = max(-0.5, min(0.5, linear_correction))
        angular_correction = max(-0.5, min(0.5, angular_correction))
        
        correction_msg = Float32MultiArray()
        correction_msg.data = [linear_correction, angular_correction]
        self.speed_correction_pub.publish(correction_msg)
        
        self.last_time = current_time
        
        # Log velocity error periodically
        if abs(velocity_error_x) > 0.1 or abs(velocity_error_angular) > 0.1:
            self.get_logger().debug(
                f'Velocity error: linear={velocity_error_x:.2f} m/s, angular={velocity_error_angular:.2f} rad/s'
            )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        odometry_node = OdometryNode()
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
