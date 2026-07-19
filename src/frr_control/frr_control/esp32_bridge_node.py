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
        self.declare_parameter('fallback_ports', [ '/dev/ttyACM0'])
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base',1.0)
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
        self.esp32_cmd_sub = self.create_subscription(
            String, '/esp32_command', self.esp32_command_callback, 10
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

        # ESP32 handshake tracking
        self.esp_ready = False
        self.handshake_sent = False
        self.last_status_request = 0

        # Send READY handshake after brief delay
        self.create_timer(0.5, self.check_handshake)
        self.create_timer(5.0, self.request_status)  # Request status every 5 seconds

        self.get_logger().info(f'ESP32 Bridge started on {self.serial_port}')

    def connect_serial(self):
        """Connect to ESP32 via serial with fallback ports"""
        ports_to_try = [self.serial_port] + self.get_parameter('fallback_ports').get_parameter_value().string_array_value

        for port in ports_to_try:
            try:
                self.get_logger().info(f'Trying to connect to ESP32 on {port}...')
                self.ser = serial.Serial(
                    port,
                    self.baud_rate,
                    timeout=1
                )
                time.sleep(2)  # Wait for ESP32 to initialize
                self.serial_connected = True
                self.serial_port = port  # Update to actual connected port
                self.get_logger().info(f'[OK] Connected to ESP32 on {port}')
                return

            except Exception as e:
                self.get_logger().warn(f'✗ Failed to connect to {port}: {e}')
                continue

        # If we get here, all ports failed
        self.get_logger().error('Failed to connect to ESP32 on any port!')
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

    def check_handshake(self):
        """Check if ESP32 needs READY handshake and send it"""
        if not self.serial_connected:
            return

        if not self.handshake_sent:
            self.get_logger().info('Sending READY handshake to ESP32...')
            if self.send_command('READY'):
                self.handshake_sent = True
                self.get_logger().info('✅ READY handshake sent to ESP32')

                # Send STATUS command to verify ESP32 is responding
                self.get_logger().info('📊 Requesting ESP32 status...')
                if self.send_command('STATUS'):
                    self.get_logger().info('✅ STATUS command sent - ESP32 should respond with system state')
                else:
                    self.get_logger().warn('[!] Failed to send STATUS command')

    def request_status(self):
        """Periodically request status from ESP32"""
        if self.serial_connected and self.handshake_sent:
            import time
            now = time.time()
            if now - self.last_status_request >= 5.0:
                self.get_logger().info('📊 Requesting ESP32 status update...')
                if self.send_command('STATUS'):
                    self.last_status_request = now
                    self.get_logger().debug('STATUS request sent')
                else:
                    self.get_logger().warn('Failed to send STATUS request')
            else:
                self.get_logger().warn('[!] Failed to send READY, will retry...')


    def esp32_command_callback(self, msg):
        """Handle incoming ESP32 commands - forward directly to serial"""
        command = msg.data.strip()

        # Parse command to update internal state
        if command == "STOP":
            self.current_left_speed = 0
            self.current_right_speed = 0
            self.scanning_enabled = False
        elif command.startswith("DRIVE"):
            # Parse DRIVE L R command
            parts = command.split()
            if len(parts) == 3:
                try:
                    left_pwm = int(parts[1])
                    right_pwm = int(parts[2])
                    self.current_left_speed = left_pwm / 100.0
                    self.current_right_speed = right_pwm / 100.0
                except ValueError:
                    self.get_logger().warn(f'Invalid DRIVE command: {command}')
        elif command in ["LEFT", "RIGHT", "TURN_LEFT", "TURN_RIGHT"]:
            # Turn commands handled by ESP32 (uses jerky motion)
            pass
        elif command == "SCAN":
            # Directional scan (left/center/right)
            self.scanning_enabled = True
        elif command == "SCAN_FIRE":
            # Continuous fire detection scan
            self.scanning_enabled = True
        elif command == "DISABLE":
            # Disable scanning
            self.scanning_enabled = False
        elif command in ["PUMP_ON", "PUMP_OFF"]:
            # Pump control
            pass
        elif command.startswith("SERVO"):
            # Servo control
            pass
        elif command in ["FORWARD", "FWD", "BACKWARD", "BACK", "BWD"]:
            # Basic movement commands
            pass

        # Forward command to ESP32
        self.send_command(command)
        self.get_logger().debug(f'ESP32 command forwarded: {command}')

    def servo_callback(self, msg):
        """Handle camera tilt servo commands"""
        angle = int(msg.data)
        # Clamp angle to 0-180 degrees (servo range)
        angle = max(0, min(180, angle))

        # Send TILT_CAMERA command to ESP32
        self.send_command(f"TILT_CAMERA {angle}")
        self.get_logger().info(f'Camera tilt: {angle}°')

    def pump_callback(self, msg):
        """Handle water pump control"""
        if msg.data:
            self.send_command("PUMP_ON")
        else:
            self.send_command("PUMP_OFF")

    def scan_callback(self, msg):
        """Handle fire scanning mode"""
        if msg.data:
            # Default to SCAN_FIRE for continuous fire detection
            self.send_command("SCAN_FIRE")
            self.scanning_enabled = True
            self.get_logger().info('Fire scanning enabled (SCAN_FIRE)')
        else:
            self.send_command("DISABLE")
            self.scanning_enabled = False
            self.get_logger().info('Fire scanning disabled')

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
