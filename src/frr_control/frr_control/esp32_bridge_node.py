#!/usr/bin/env python3
"""
ESP32 Bridge Node - Communicates with ESP32 for motor control and sensor data
Replaces direct GPIO motor control with serial commands to ESP32
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import Imu, Range, Temperature
import serial
import json
import threading
import time

class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge_node')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.2)
        self.declare_parameter('max_linear_speed', 100)  # PWM 0-100
        self.declare_parameter('max_angular_speed', 100)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().integer_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().integer_value
        
        # Serial connection
        self.serial_connected = False
        self.ser = None
        self.connect_serial()
        
        # ROS Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.servo_sub = self.create_subscription(
            Float64, '/camera_tilt', self.servo_callback, 10
        )
        self.pump_sub = self.create_subscription(
            Bool, '/water_pump', self.pump_callback, 10
        )
        self.scan_sub = self.create_subscription(
            Bool, '/fire_scan', self.scan_callback, 10
        )
        
        # ROS Publishers
        self.status_pub = self.create_publisher(String, '/motor_status', 10)
        self.telemetry_pub = self.create_publisher(String, '/esp32_telemetry', 10)
        self.smoke_pub = self.create_publisher(Range, '/sensors/smoke', 10)
        self.gas_pub = self.create_publisher(Range, '/sensors/gas', 10)
        self.flame_pub = self.create_publisher(Bool, '/sensors/flame', 10)
        self.temp_pub = self.create_publisher(Temperature, '/sensors/temperature', 10)
        
        # Status timer (10 Hz)
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        # Start serial reading thread
        self.reading_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.reading_thread.start()
        
        # Current state
        self.current_left_speed = 0
        self.current_right_speed = 0
        self.scanning_enabled = False
        
        self.get_logger().info(f'ESP32 Bridge started on {self.serial_port}')

    def connect_serial(self):
        """Connect to ESP32 via serial"""
        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=1
            )
            time.sleep(2)  # Wait for ESP32 to initialize
            self.serial_connected = True
            self.get_logger().info(f'Connected to ESP32 on {self.serial_port}')
            
            # Send initial status request
            self.send_command("STATUS")
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            self.serial_connected = False

    def send_command(self, cmd):
        """Send command to ESP32"""
        if not self.serial_connected or not self.ser:
            self.get_logger().warn(f'ESP32 not connected, cannot send: {cmd}')
            return False
        
        try:
            self.ser.write(f"{cmd}\n".encode())
            self.get_logger().debug(f'Sent to ESP32: {cmd}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
            return False

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands - convert to ESP32 motor commands"""
        linear_vel = msg.linear.x  # -1.0 to 1.0
        angular_vel = msg.angular.z  # -1.0 to 1.0
        
        # Convert to differential drive (tank steering)
        left_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)
        
        # Normalize to -1.0 to 1.0 range
        max_vel = max(abs(left_vel), abs(right_vel))
        if max_vel > 1.0:
            left_vel /= max_vel
            right_vel /= max_vel
        
        # Convert to PWM (0-100 for your ESP32)
        left_pwm = int(left_vel * 100)  # Can be negative
        right_pwm = int(right_vel * 100)  # Can be negative
        
        # Send command using DRIVE command (sets both motors at once)
        if left_pwm == 0 and right_pwm == 0:
            self.send_command("STOP")
        else:
            # Use DRIVE <left> <right> command
            self.send_command(f"DRIVE {left_pwm} {right_pwm}")
        
        self.current_left_speed = left_vel
        self.current_right_speed = right_vel
        
        self.get_logger().debug(f'Motors: L={left_pwm}, R={right_pwm}')

    def servo_callback(self, msg):
        """Handle camera tilt servo commands"""
        angle = int(msg.data)
        # Map -90 to 90 degrees to servo range 30-150
        servo_angle = int((angle + 90) * (150 - 30) / 180 + 30)
        servo_angle = max(30, min(150, servo_angle))
        
        # Send servo position to turret servo (Pin 39)
        # Note: You may need to add a SERVO command to ESP32 code
        self.send_command(f"SERVO {servo_angle}")
        self.get_logger().info(f'Servo tilt: {angle}° -> {servo_angle}°')

    def pump_callback(self, msg):
        """Handle water pump control"""
        if msg.data:
            self.send_command("PUMP_ON")
        else:
            self.send_command("PUMP_OFF")

    def scan_callback(self, msg):
        """Handle fire scanning mode"""
        if msg.data:
            self.send_command("SCAN")
            self.scanning_enabled = True
        else:
            self.send_command("DISABLE")
            self.scanning_enabled = False

    def read_serial_loop(self):
        """Continuously read from ESP32 serial port"""
        while rclpy.ok():
            if not self.serial_connected or not self.ser:
                time.sleep(1)
                continue
            
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if not line:
                        continue
                    
                    # Check if it's JSON telemetry
                    if line.startswith('{'):
                        self.parse_telemetry(line)
                    else:
                        # Regular text message
                        self.get_logger().info(f'ESP32: {line}')
                        
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.1)

    def parse_telemetry(self, json_str):
        """Parse JSON telemetry from ESP32"""
        try:
            data = json.loads(json_str)
            
            # Publish telemetry as string
            telemetry_msg = String()
            telemetry_msg.data = json_str
            self.telemetry_pub.publish(telemetry_msg)
            
            # Publish individual sensor readings
            # Smoke sensor (MQ2)
            if 'mq2' in data:
                smoke_msg = Range()
                smoke_msg.header.stamp = self.get_clock().now().to_msg()
                smoke_msg.header.frame_id = 'smoke_sensor'
                smoke_msg.range = float(data['mq2'])
                self.smoke_pub.publish(smoke_msg)
            
            # Gas sensor (MQ5)
            if 'mq5' in data:
                gas_msg = Range()
                gas_msg.header.stamp = self.get_clock().now().to_msg()
                gas_msg.header.frame_id = 'gas_sensor'
                gas_msg.range = float(data['mq5'])
                self.gas_pub.publish(gas_msg)
            
            # Flame sensor
            if 'flame' in data:
                flame_msg = Bool()
                flame_msg.data = (data['flame'] == 0)  # LOW = flame detected
                self.flame_pub.publish(flame_msg)
            
            # Temperature
            if 'temp' in data:
                temp_msg = Temperature()
                temp_msg.header.stamp = self.get_clock().now().to_msg()
                temp_msg.header.frame_id = 'temperature_sensor'
                temp_msg.temperature = float(data['temp'])
                self.temp_pub.publish(temp_msg)
            
            self.get_logger().debug(f'Telemetry: {data}')
            
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid JSON: {json_str}')

    def publish_status(self):
        """Publish motor status"""
        status_msg = String()
        status_msg.data = f'Left: {self.current_left_speed:.2f}, Right: {self.current_right_speed:.2f}, Scanning: {self.scanning_enabled}'
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean up resources"""
        self.get_logger().info('Stopping ESP32 bridge')
        if self.serial_connected and self.ser:
            self.send_command("STOP")
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ESP32BridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()