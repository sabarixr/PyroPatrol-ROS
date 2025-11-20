#!/usr/bin/env python3
"""
Robust MPU6050 Node with vibration filtering for LiDAR interference
Addresses: I2C errors, retry logic, and high-frequency vibration from spinning LiDAR
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import smbus
import time
import math
from collections import deque
import numpy as np

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
        
        # Vibration filtering parameters
        self.use_vibration_filter = True
        self.vibration_threshold = 0.5  # g - ignore high-freq changes above this
        self.filter_window_size = 5  # Moving average window
        
        # Circular buffers for filtering
        self.accel_x_buffer = deque(maxlen=self.filter_window_size)
        self.accel_y_buffer = deque(maxlen=self.filter_window_size)
        self.accel_z_buffer = deque(maxlen=self.filter_window_size)
        self.gyro_x_buffer = deque(maxlen=self.filter_window_size)
        self.gyro_y_buffer = deque(maxlen=self.filter_window_size)
        self.gyro_z_buffer = deque(maxlen=self.filter_window_size)
        
        # Low-pass filter (complementary to moving average)
        self.alpha = 0.7  # 0 = all old, 1 = all new
        self.filtered_ax = 0.0
        self.filtered_ay = 0.0
        self.filtered_az = 0.0
        self.filtered_gx = 0.0
        self.filtered_gy = 0.0
        self.filtered_gz = 0.0
        
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
        
        # Timer for periodic readings (50 Hz = 20ms) - reduced from 100Hz
        self.timer = self.create_timer(0.02, self.read_imu)
        
        self.get_logger().info('MPU6050 IMU Node started with LiDAR vibration filtering')
        self.get_logger().info(f'Vibration filter: {"ENABLED" if self.use_vibration_filter else "DISABLED"}')

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
        """Initialize the MPU6050 with vibration-resistant settings"""
        for attempt in range(self.max_retries):
            try:
                # Reset the device first
                self.write_byte_with_retry(0x6B, 0x80)
                time.sleep(0.1)
                
                # Wake up the MPU6050
                self.write_byte_with_retry(0x6B, 0x00)
                time.sleep(0.05)
                
                # Set accelerometer range to ±2g (most sensitive, best for vibration detection)
                self.write_byte_with_retry(0x1C, 0x00)
                time.sleep(0.01)
                
                # Set gyroscope range to ±250°/s
                self.write_byte_with_retry(0x1B, 0x00)
                time.sleep(0.01)
                
                # Set sample rate divider (1kHz / (1 + 4) = 200Hz internal sampling)
                self.write_byte_with_retry(0x19, 0x04)
                time.sleep(0.01)
                
                # Configure DLPF for vibration rejection
                # Set DLPF to 5Hz cutoff (register 0x1A, value 0x06)
                # This filters out high-frequency vibrations from LiDAR motor
                self.write_byte_with_retry(0x1A, 0x06)
                time.sleep(0.1)
                
                # Verify sensor is responding
                who_am_i = self.read_byte_with_retry(0x75)
                if who_am_i == 0x68:
                    self.get_logger().info(f'MPU6050 initialized with DLPF at 5Hz (vibration filtering)')
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
                time.sleep(0.005)

    def read_byte_with_retry(self, reg):
        """Read a byte from I2C with retry logic"""
        for attempt in range(self.max_retries):
            try:
                return self.bus.read_byte_data(self.addr, reg)
            except Exception as e:
                if attempt == self.max_retries - 1:
                    raise e
                time.sleep(0.005)

    def read_raw_data(self, addr):
        """Read raw 16-bit data from MPU6050 with retry logic"""
        try:
            high = self.read_byte_with_retry(addr)
            low = self.read_byte_with_retry(addr + 1)
            
            value = (high << 8) | low
            if value > 32767:
                value -= 65536
            
            self.read_failure_count = 0
            return value
            
        except Exception as e:
            self.read_failure_count += 1
            
            if self.read_failure_count <= 3:
                self.get_logger().warn(f'Failed to read MPU6050 data at 0x{addr:02X}: {e}')
            
            if self.read_failure_count >= self.max_consecutive_failures:
                self.get_logger().error(f'Too many consecutive failures, reinitializing...')
                try:
                    self.init_sensor()
                    self.read_failure_count = 0
                except Exception as reinit_error:
                    self.get_logger().error(f'Reinitialization failed: {reinit_error}')
            
            return 0

    def apply_moving_average_filter(self, buffer, new_value):
        """Apply moving average filter to reduce vibration noise"""
        buffer.append(new_value)
        if len(buffer) > 0:
            return sum(buffer) / len(buffer)
        return new_value

    def apply_low_pass_filter(self, old_value, new_value, alpha):
        """Apply exponential low-pass filter"""
        return alpha * new_value + (1 - alpha) * old_value

    def detect_vibration_spike(self, current, previous, threshold):
        """Detect if change is likely vibration vs real movement"""
        delta = abs(current - previous)
        return delta > threshold

    def read_accel_gyro(self):
        """Read and filter accelerometer and gyroscope data"""
        # Read raw data
        accel_x = self.read_raw_data(0x3B)
        accel_y = self.read_raw_data(0x3D)
        accel_z = self.read_raw_data(0x3F)
        
        gyro_x = self.read_raw_data(0x43)
        gyro_y = self.read_raw_data(0x45)
        gyro_z = self.read_raw_data(0x47)
        
        # Convert to real units
        ax_raw = (accel_x / 16384.0) - self.accel_offset_x
        ay_raw = (accel_y / 16384.0) - self.accel_offset_y
        az_raw = (accel_z / 16384.0) - self.accel_offset_z
        
        gx_raw = math.radians((gyro_x / 131.0) - self.gyro_offset_x)
        gy_raw = math.radians((gyro_y / 131.0) - self.gyro_offset_y)
        gz_raw = math.radians((gyro_z / 131.0) - self.gyro_offset_z)
        
        if self.use_vibration_filter:
            # Apply moving average filter
            ax_ma = self.apply_moving_average_filter(self.accel_x_buffer, ax_raw)
            ay_ma = self.apply_moving_average_filter(self.accel_y_buffer, ay_raw)
            az_ma = self.apply_moving_average_filter(self.accel_z_buffer, az_raw)
            
            gx_ma = self.apply_moving_average_filter(self.gyro_x_buffer, gx_raw)
            gy_ma = self.apply_moving_average_filter(self.gyro_y_buffer, gy_raw)
            gz_ma = self.apply_moving_average_filter(self.gyro_z_buffer, gz_raw)
            
            # Apply low-pass filter on top of moving average
            self.filtered_ax = self.apply_low_pass_filter(self.filtered_ax, ax_ma, self.alpha)
            self.filtered_ay = self.apply_low_pass_filter(self.filtered_ay, ay_ma, self.alpha)
            self.filtered_az = self.apply_low_pass_filter(self.filtered_az, az_ma, self.alpha)
            
            self.filtered_gx = self.apply_low_pass_filter(self.filtered_gx, gx_ma, self.alpha)
            self.filtered_gy = self.apply_low_pass_filter(self.filtered_gy, gy_ma, self.alpha)
            self.filtered_gz = self.apply_low_pass_filter(self.filtered_gz, gz_ma, self.alpha)
            
            return (self.filtered_ax, self.filtered_ay, self.filtered_az,
                    self.filtered_gx, self.filtered_gy, self.filtered_gz)
        else:
            return ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw

    def calibrate(self):
        """Calibrate sensor - now with vibration filtering during calibration"""
        self.get_logger().info('Calibrating MPU6050... Keep rover stationary!')
        self.get_logger().warn('Note: LiDAR vibration will be filtered out during calibration')
        
        samples = 100  # More samples for better calibration with vibration
        accel_x_samples = []
        accel_y_samples = []
        accel_z_samples = []
        gyro_x_samples = []
        gyro_y_samples = []
        gyro_z_samples = []
        
        for i in range(samples * 2):  # Collect more samples
            try:
                accel_x = self.read_raw_data(0x3B) / 16384.0
                accel_y = self.read_raw_data(0x3D) / 16384.0
                accel_z = self.read_raw_data(0x3F) / 16384.0
                
                gyro_x = self.read_raw_data(0x43) / 131.0
                gyro_y = self.read_raw_data(0x45) / 131.0
                gyro_z = self.read_raw_data(0x47) / 131.0
                
                # Only use samples that aren't obvious vibration spikes
                if abs(accel_z - 1.0) < 0.5:  # Z should be near 1g
                    accel_x_samples.append(accel_x)
                    accel_y_samples.append(accel_y)
                    accel_z_samples.append(accel_z)
                    gyro_x_samples.append(gyro_x)
                    gyro_y_samples.append(gyro_y)
                    gyro_z_samples.append(gyro_z)
                
                if len(accel_x_samples) >= samples:
                    break
                    
                time.sleep(0.02)
                
            except Exception as e:
                self.get_logger().warn(f'Calibration sample failed: {e}')
                time.sleep(0.02)
        
        if len(accel_x_samples) < samples // 2:
            self.get_logger().error(f'Calibration incomplete: only {len(accel_x_samples)}/{samples} good samples')
            return
        
        # Use median instead of mean to reject outliers (vibration spikes)
        self.accel_offset_x = float(np.median(accel_x_samples))
        self.accel_offset_y = float(np.median(accel_y_samples))
        self.accel_offset_z = float(np.median(accel_z_samples)) - 1.0
        
        self.gyro_offset_x = float(np.median(gyro_x_samples))
        self.gyro_offset_y = float(np.median(gyro_y_samples))
        self.gyro_offset_z = float(np.median(gyro_z_samples))
        
        # Calculate vibration noise level
        accel_std = np.std(accel_x_samples)
        
        self.get_logger().info(f'Calibration complete with {len(accel_x_samples)} samples:')
        self.get_logger().info(f'  Accel offsets: X={self.accel_offset_x:.3f}, Y={self.accel_offset_y:.3f}, Z={self.accel_offset_z:.3f}')
        self.get_logger().info(f'  Gyro offsets: X={self.gyro_offset_x:.3f}, Y={self.gyro_offset_y:.3f}, Z={self.gyro_offset_z:.3f}')
        self.get_logger().info(f'  Vibration noise level: {accel_std:.4f}g (lower is better)')

    def integrate_velocity(self, ax, ay, az):
        """Velocity integration with stronger drift compensation (filtered data)"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0.1:
            dt = 0.02
            
        # Stronger decay for filtered data (less drift accumulation)
        decay = 0.995
        self.velocity_x = (self.velocity_x + ax * dt * 9.81) * decay
        self.velocity_y = (self.velocity_y + ay * dt * 9.81) * decay
        self.velocity_z = (self.velocity_z + az * dt * 9.81) * decay
        
        self.last_time = current_time
        
        return self.velocity_x, self.velocity_y, self.velocity_z

    def read_imu(self):
        """Main timer callback with vibration filtering"""
        try:
            # Read and filter data
            ax, ay, az, gx, gy, gz = self.read_accel_gyro()
            
            # Integrate velocity
            vx, vy, vz = self.integrate_velocity(ax, ay, az)
            
            # Create IMU message
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
            
            # Orientation not available
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0
            
            # Covariance - increase for filtered data
            imu_msg.linear_acceleration_covariance[0] = 0.02  # Increased due to filtering
            imu_msg.linear_acceleration_covariance[4] = 0.02
            imu_msg.linear_acceleration_covariance[8] = 0.02
            
            imu_msg.angular_velocity_covariance[0] = 0.002
            imu_msg.angular_velocity_covariance[4] = 0.002
            imu_msg.angular_velocity_covariance[8] = 0.002
            
            imu_msg.orientation_covariance[0] = -1.0
            
            # Publish
            self.imu_pub.publish(imu_msg)
            
            raw_msg = Float32MultiArray()
            raw_msg.data = [ax, ay, az, gx, gy, gz, vx, vy, vz]
            self.raw_pub.publish(raw_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in read_imu: {e}')

    def __del__(self):
        """Cleanup"""
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
