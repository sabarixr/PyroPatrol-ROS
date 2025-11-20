#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import smbus
import struct
import time
import math
from geometry_msgs.msg import Vector3, Quaternion

class MMA8452Node(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # I2C setup
        self.bus = smbus.SMBus(1)  # I2C bus 1 on Raspberry Pi
        self.addr = 0x1D  # MMA8452 address when ADDR pin is GND
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.fall_pub = self.create_publisher(Bool, '/rover/fall_detected', 10)
        
        # Timer for periodic readings (50 Hz = 20ms)
        self.timer = self.create_timer(0.02, self.read_imu)
        
        # Initialize sensor
        self.init_sensor()
        
        # Velocity estimation variables
        self.last_time = time.time()
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        
        # Fall detection threshold (g)
        self.fall_threshold = 0.3
        
        self.get_logger().info('MMA8452 IMU Node started')

    def init_sensor(self):
        """Initialize the MMA8452 accelerometer"""
        try:
            # Put device in standby mode
            self.bus.write_byte_data(self.addr, 0x2A, 0x00)
            
            # Set full scale range to ±2g
            self.bus.write_byte_data(self.addr, 0x0E, 0x00)
            
            # Set data rate to 50 Hz and activate
            self.bus.write_byte_data(self.addr, 0x2A, 0x20)
            
            self.get_logger().info('MMA8452 initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MMA8452: {e}')

    def read_raw_data(self):
        """Read raw acceleration data from MMA8452"""
        try:
            # Read 6 bytes starting from X MSB register (0x01)
            data = self.bus.read_i2c_block_data(self.addr, 0x01, 6)
            
            # Convert to signed 12-bit values
            x = (data[0] << 4) | (data[1] >> 4)
            y = (data[2] << 4) | (data[3] >> 4)
            z = (data[4] << 4) | (data[5] >> 4)
            
            # Convert to signed integers
            if x > 2047:
                x -= 4096
            if y > 2047:
                y -= 4096
            if z > 2047:
                z -= 4096
            
            # Convert to g (±2g range, 12-bit resolution)
            x_g = x / 1024.0
            y_g = y / 1024.0
            z_g = z / 1024.0
            
            return x_g, y_g, z_g
            
        except Exception as e:
            self.get_logger().warn(f'Failed to read IMU data: {e}')
            return 0.0, 0.0, 0.0

    def integrate_velocity(self, ax, ay, az):
        """Simple velocity integration with drift compensation"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0.1:  # Reset if too much time has passed
            dt = 0.02
            
        # Simple integration: v = v + a*dt
        # Apply small decay factor to reduce drift
        decay = 0.999
        self.velocity_x = (self.velocity_x + ax * dt * 9.81) * decay
        self.velocity_y = (self.velocity_y + ay * dt * 9.81) * decay
        self.velocity_z = (self.velocity_z + az * dt * 9.81) * decay
        
        self.last_time = current_time
        
        return self.velocity_x, self.velocity_y, self.velocity_z

    def detect_fall(self, z_accel):
        """Detect if rover has fallen based on Z-axis acceleration"""
        # If Z acceleration is less than threshold, rover might have fallen
        return abs(z_accel) < self.fall_threshold

    def read_imu(self):
        """Main timer callback to read and publish IMU data"""
        # Read raw accelerometer data
        ax, ay, az = self.read_raw_data()
        
        # Integrate velocity (lightweight estimation)
        vx, vy, vz = self.integrate_velocity(ax, ay, az)
        
        # Create and populate IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = ax * 9.81
        imu_msg.linear_acceleration.y = ay * 9.81
        imu_msg.linear_acceleration.z = az * 9.81
        
        # We don't have gyroscope data, so angular velocity is zero
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        
        # We don't have magnetometer, so orientation is not available
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        
        # Set covariance matrices (-1 means unknown)
        imu_msg.linear_acceleration_covariance = [-1.0] * 9
        imu_msg.angular_velocity_covariance = [-1.0] * 9
        imu_msg.orientation_covariance = [-1.0] * 9
        
        # Publish IMU data
        self.imu_pub.publish(imu_msg)
        
        # Check for fall detection
        fall_detected = self.detect_fall(az)
        fall_msg = Bool()
        fall_msg.data = fall_detected
        self.fall_pub.publish(fall_msg)
        
        if fall_detected:
            self.get_logger().warn(f'Fall detected! Z-accel: {az:.3f}g')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        imu_node = MMA8452Node()
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()