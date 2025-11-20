#!/usr/bin/env python3
"""
Robust MPU6050 Node with improved I2C error handling and retry logic
This version addresses common I2C bus contention and communication issues
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import smbus
import time
import math

class MPU6050NodeRobust(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        
        # I2C setup with error handling
        self.bus = None
        self.addr = 0x68  # MPU6050 default address
        self.max_retries = 3
        self.read_failure_count = 0
        self.max_consecutive_failures = 10
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/mpu6050', 10)
        self.raw_pub = self.create_publisher(Float32MultiArray, '/imu/mpu6050_raw', 10)
        
        # Initialize I2C bus with retry
        self.init_i2c_bus()
        
        # Initialize sensor
        self.init_sensor()
        
        # Calibration offsets
        self.accel_offset_x = 0.0
        self.accel_offset_y = 0.0
        self.accel_offset_z = 0.0
        self.gyro_offset_x = 0.0
        self.gyro_offset_y = 0.0
        self.gyro_offset_z = 0.0
        
        # Odometry variables
        self.last_time = time.time()
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        
        # Calibrate sensor
        self.calibrate()
        
        # Timer for periodic readings (50 Hz = 20ms) - reduced from 100Hz to reduce bus load
        self.timer = self.create_timer(0.02, self.read_imu)
        
        self.get_logger().info('MPU6050 IMU Node started (robust version with retry logic)')

    def init_i2c_bus(self):
        """Initialize I2C bus with retry logic"""
        for attempt in range(self.max_retries):
            try:
                self.bus = smbus.SMBus(1)
                self.get_logger().info('I2C bus opened successfully')
                return
            except Exception as e:
                self.get_logger().warn(f'Failed to open I2C bus (attempt {attempt+1}/{self.max_retries}): {e}')
                time.sleep(0.5)
        
        raise RuntimeError('Failed to initialize I2C bus after multiple attempts')

    def init_sensor(self):
        """Initialize the MPU6050 with retry logic and proper delays"""
        for attempt in range(self.max_retries):
            try:
                # Reset the device first
                self.write_byte_with_retry(0x6B, 0x80)
                time.sleep(0.1)  # Wait for reset to complete
                
                # Wake up the MPU6050
                self.write_byte_with_retry(0x6B, 0x00)
                time.sleep(0.05)
                
                # Set accelerometer range to ±2g
                self.write_byte_with_retry(0x1C, 0x00)
                time.sleep(0.01)
                
                # Set gyroscope range to ±250°/s
                self.write_byte_with_retry(0x1B, 0x00)
                time.sleep(0.01)
                
                # Set sample rate divider (1kHz / (1 + 4) = 200Hz)
                self.write_byte_with_retry(0x19, 0x04)
                time.sleep(0.01)
                
                # Configure digital low pass filter to 94 Hz
                self.write_byte_with_retry(0x1A, 0x02)
                time.sleep(0.1)
                
                # Verify sensor is responding
                who_am_i = self.read_byte_with_retry(0x75)
                if who_am_i == 0x68:
                    self.get_logger().info(f'MPU6050 initialized successfully (WHO_AM_I: 0x{who_am_i:02X})')
                    return
                else:
                    self.get_logger().warn(f'Unexpected WHO_AM_I value: 0x{who_am_i:02X}')
                    
            except Exception as e:
                self.get_logger().warn(f'Init attempt {attempt+1}/{self.max_retries} failed: {e}')
                time.sleep(0.5)
        
        raise RuntimeError('Failed to initialize MPU6050 after multiple attempts')

    def write_byte_with_retry(self, reg, value):
        """Write a byte to I2C with retry logic"""
        for attempt in range(self.max_retries):
            try:
                self.bus.write_byte_data(self.addr, reg, value)
                return
            except Exception as e:
                if attempt == self.max_retries - 1:
                    raise e
                time.sleep(0.005)  # 5ms delay before retry

    def read_byte_with_retry(self, reg):
        """Read a byte from I2C with retry logic"""
        for attempt in range(self.max_retries):
            try:
                return self.bus.read_byte_data(self.addr, reg)
            except Exception as e:
                if attempt == self.max_retries - 1:
                    raise e
                time.sleep(0.005)  # 5ms delay before retry

    def read_raw_data(self, addr):
        """Read raw 16-bit data from MPU6050 with retry logic"""
        try:
            # Read high and low bytes with retry
            high = self.read_byte_with_retry(addr)
            low = self.read_byte_with_retry(addr + 1)
            
            # Combine bytes
            value = (high << 8) | low
            
            # Convert to signed value
            if value > 32767:
                value -= 65536
            
            # Reset failure counter on success
            self.read_failure_count = 0
            return value
            
        except Exception as e:
            self.read_failure_count += 1
            
            if self.read_failure_count <= 3:
                self.get_logger().warn(f'Failed to read MPU6050 data at 0x{addr:02X}: {e}')
            
            # If too many consecutive failures, try to reinitialize
            if self.read_failure_count >= self.max_consecutive_failures:
                self.get_logger().error(f'Too many consecutive read failures ({self.read_failure_count}), reinitializing sensor...')
                try:
                    self.init_sensor()
                    self.read_failure_count = 0
                except Exception as reinit_error:
                    self.get_logger().error(f'Reinitialization failed: {reinit_error}')
            
            return 0

    def read_accel_gyro(self):
        """Read accelerometer and gyroscope data"""
        # Read accelerometer data
        accel_x = self.read_raw_data(0x3B)
        accel_y = self.read_raw_data(0x3D)
        accel_z = self.read_raw_data(0x3F)
        
        # Read gyroscope data
        gyro_x = self.read_raw_data(0x43)
        gyro_y = self.read_raw_data(0x45)
        gyro_z = self.read_raw_data(0x47)
        
        # Convert to real units
        ax = (accel_x / 16384.0) - self.accel_offset_x
        ay = (accel_y / 16384.0) - self.accel_offset_y
        az = (accel_z / 16384.0) - self.accel_offset_z
        
        # Gyroscope: ±250°/s range, convert to rad/s
        gx = math.radians((gyro_x / 131.0) - self.gyro_offset_x)
        gy = math.radians((gyro_y / 131.0) - self.gyro_offset_y)
        gz = math.radians((gyro_z / 131.0) - self.gyro_offset_z)
        
        return ax, ay, az, gx, gy, gz

    def calibrate(self):
        """Calibrate sensor by averaging readings while stationary"""
        self.get_logger().info('Calibrating MPU6050... Keep rover stationary!')
        
        samples = 50  # Reduced from 100 to speed up startup
        sum_ax, sum_ay, sum_az = 0.0, 0.0, 0.0
        sum_gx, sum_gy, sum_gz = 0.0, 0.0, 0.0
        
        successful_samples = 0
        
        for i in range(samples * 2):  # Try more times to get enough good samples
            try:
                accel_x = self.read_raw_data(0x3B) / 16384.0
                accel_y = self.read_raw_data(0x3D) / 16384.0
                accel_z = self.read_raw_data(0x3F) / 16384.0
                
                gyro_x = self.read_raw_data(0x43) / 131.0
                gyro_y = self.read_raw_data(0x45) / 131.0
                gyro_z = self.read_raw_data(0x47) / 131.0
                
                # Only count if values seem reasonable
                if abs(accel_z - 1.0) < 0.5:  # Z-axis should be near 1g
                    sum_ax += accel_x
                    sum_ay += accel_y
                    sum_az += accel_z
                    sum_gx += gyro_x
                    sum_gy += gyro_y
                    sum_gz += gyro_z
                    successful_samples += 1
                
                if successful_samples >= samples:
                    break
                    
                time.sleep(0.02)
                
            except Exception as e:
                self.get_logger().warn(f'Calibration sample failed: {e}')
                time.sleep(0.02)
        
        if successful_samples < samples // 2:
            self.get_logger().error(f'Calibration incomplete: only {successful_samples}/{samples} good samples')
            return
        
        # Calculate average offsets
        self.accel_offset_x = sum_ax / successful_samples
        self.accel_offset_y = sum_ay / successful_samples
        self.accel_offset_z = (sum_az / successful_samples) - 1.0  # Subtract 1g
        
        self.gyro_offset_x = sum_gx / successful_samples
        self.gyro_offset_y = sum_gy / successful_samples
        self.gyro_offset_z = sum_gz / successful_samples
        
        self.get_logger().info(f'Calibration complete with {successful_samples} samples:')
        self.get_logger().info(f'  Accel offsets: X={self.accel_offset_x:.3f}, Y={self.accel_offset_y:.3f}, Z={self.accel_offset_z:.3f}')
        self.get_logger().info(f'  Gyro offsets: X={self.gyro_offset_x:.3f}, Y={self.gyro_offset_y:.3f}, Z={self.gyro_offset_z:.3f}')

    def integrate_velocity(self, ax, ay, az):
        """Simple velocity integration with drift compensation"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0.1:  # Reset if too much time has passed
            dt = 0.02
            
        # Simple integration with decay
        decay = 0.998
        self.velocity_x = (self.velocity_x + ax * dt * 9.81) * decay
        self.velocity_y = (self.velocity_y + ay * dt * 9.81) * decay
        self.velocity_z = (self.velocity_z + az * dt * 9.81) * decay
        
        self.last_time = current_time
        
        return self.velocity_x, self.velocity_y, self.velocity_z

    def read_imu(self):
        """Main timer callback to read and publish IMU data"""
        try:
            # Read accelerometer and gyroscope data
            ax, ay, az, gx, gy, gz = self.read_accel_gyro()
            
            # Integrate velocity for odometry
            vx, vy, vz = self.integrate_velocity(ax, ay, az)
            
            # Create and populate IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'mpu6050_link'
            
            # Linear acceleration (m/s²)
            imu_msg.linear_acceleration.x = ax * 9.81
            imu_msg.linear_acceleration.y = ay * 9.81
            imu_msg.linear_acceleration.z = az * 9.81
            
            # Angular velocity (rad/s)
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            
            # Orientation is not available
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0
            
            # Set covariance matrices
            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.01
            
            imu_msg.angular_velocity_covariance[0] = 0.001
            imu_msg.angular_velocity_covariance[4] = 0.001
            imu_msg.angular_velocity_covariance[8] = 0.001
            
            imu_msg.orientation_covariance[0] = -1.0
            
            # Publish messages
            self.imu_pub.publish(imu_msg)
            
            raw_msg = Float32MultiArray()
            raw_msg.data = [ax, ay, az, gx, gy, gz, vx, vy, vz]
            self.raw_pub.publish(raw_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in read_imu callback: {e}')

    def __del__(self):
        """Cleanup on node shutdown"""
        if self.bus:
            try:
                self.bus.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        mpu6050_node = MPU6050NodeRobust()
        rclpy.spin(mpu6050_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
