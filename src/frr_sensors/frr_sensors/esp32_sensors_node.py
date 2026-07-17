#!/usr/bin/env python3
"""
ESP32 Sensors Node - Handles all ESP32 telemetry parsing
Publishes sensor data to ROS2 topics for other nodes to use
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Temperature, Range
import json


class ESP32SensorsNode(Node):
    def __init__(self):
        super().__init__('esp32_sensors_node')
        
        # Publishers for individual sensor topics
        self.mq2_pub = self.create_publisher(Int32, '/sensors/mq2', 10)
        self.mq5_pub = self.create_publisher(Int32, '/sensors/mq5', 10)
        self.temp_pub = self.create_publisher(Temperature, '/sensors/temperature', 10)
        self.flame_pub = self.create_publisher(Bool, '/sensors/flame', 10)
        
        # Scan results
        self.scan_complete_pub = self.create_publisher(String, '/sensors/scan_complete', 10)
        self.scan_sample_pub = self.create_publisher(String, '/sensors/scan_sample', 10)
        self.scan_result_pub = self.create_publisher(String, '/sensors/scan_result', 10)
        
        # System status
        self.esp_state_pub = self.create_publisher(String, '/esp32/state', 10)
        self.esp_ready_pub = self.create_publisher(Bool, '/esp32/ready', 10)
        self.pump_status_pub = self.create_publisher(Bool, '/esp32/pump_active', 10)
        
        # Odometry
        self.odom_pub = self.create_publisher(String, '/esp32/odometry', 10)
        self.rpm_pub = self.create_publisher(String, '/esp32/rpm', 10)
        
        # Alert/warnings
        self.alert_pub = self.create_publisher(String, '/sensors/alert', 10)
        
        # Subscribe to raw ESP32 telemetry
        self.create_subscription(String, '/esp32_telemetry', self.telemetry_callback, 10)
        
        self.get_logger().info('ESP32 Sensors Node started - parsing telemetry from /esp32_telemetry')
        
        # State tracking
        self.esp_ready = False
        self.current_state = 'boot_init'
        
    def telemetry_callback(self, msg: String):
        """Parse ESP32 JSON telemetry and publish to specific topics"""
        try:
            data = json.loads(msg.data)
            msg_type = data.get('type', '')
            
            # Handle different message types
            if msg_type == 'boot':
                self.get_logger().info(f"ESP32 Boot: {data.get('msg', '')}")
                state_msg = String()
                state_msg.data = json.dumps({'state': data.get('state', 'init'), 'msg': data.get('msg', '')})
                self.esp_state_pub.publish(state_msg)
                
            elif msg_type == 'ping':
                # ESP32 is alive, update state
                self.current_state = data.get('state', 'unknown')
                
            elif msg_type == 'handshake':
                status = data.get('status', '')
                if status == 'pi_ready' or status == 'manual_override':
                    self.get_logger().info(f"ESP32 Handshake: {status}")
                    
            elif msg_type == 'state':
                new_state = data.get('new', '')
                self.current_state = new_state
                self.get_logger().info(f"ESP32 State: {new_state}")
                
                # Publish ready status
                ready_msg = Bool()
                ready_msg.data = (new_state == 'ready')
                self.esp_ready_pub.publish(ready_msg)
                
            elif msg_type == 'system':
                state = data.get('state', '')
                if state == 'ready':
                    self.esp_ready = True
                    ready_msg = Bool()
                    ready_msg.data = True
                    self.esp_ready_pub.publish(ready_msg)
                    self.get_logger().info("ESP32 READY - All systems operational")
                    
            elif msg_type == 'scan_sample':
                # Real-time sensor readings during scan
                mq2 = data.get('mq2', 0)
                mq5 = data.get('mq5', 0)
                temp = data.get('temp', 25.0)
                flame = data.get('flame', 0)
                
                # Publish individual sensors
                mq2_msg = Int32()
                mq2_msg.data = mq2
                self.mq2_pub.publish(mq2_msg)
                
                mq5_msg = Int32()
                mq5_msg.data = mq5
                self.mq5_pub.publish(mq5_msg)
                
                temp_msg = Temperature()
                temp_msg.temperature = temp
                self.temp_pub.publish(temp_msg)
                
                flame_msg = Bool()
                flame_msg.data = (flame == 1)
                self.flame_pub.publish(flame_msg)
                
                # Publish full scan sample
                self.scan_sample_pub.publish(msg)
                
            elif msg_type == 'scan_result':
                # Endpoint scan result (averaged)
                self.scan_result_pub.publish(msg)
                self.get_logger().info(f"Scan result: {data.get('side', '')} angle={data.get('angle', 0)} score={data.get('score', 0):.3f}")
                
            elif msg_type == 'scan_complete':
                # Final scan direction decision
                direction = data.get('direction', 'none')
                reason = data.get('reason', '')
                self.scan_complete_pub.publish(msg)
                self.get_logger().info(f"Scan complete: direction={direction} ({reason})")
                
            elif msg_type == 'alert':
                # Fire/flame alert
                self.alert_pub.publish(msg)
                self.get_logger().warn(f"ALERT: {data.get('msg', 'unknown')}")
                
            elif msg_type == 'pump':
                # Pump status
                status = data.get('status', 'off')
                pump_msg = Bool()
                pump_msg.data = (status == 'on')
                self.pump_status_pub.publish(pump_msg)
                self.get_logger().info(f"Pump: {status}")
                
            elif msg_type == 'odom':
                # Odometry update
                self.odom_pub.publish(msg)
                
            elif msg_type == 'rpm':
                # RPM update
                self.rpm_pub.publish(msg)
                
            elif msg_type == 'status_report':
                # Full system status
                self.get_logger().info(f"ESP32 Status: state={data.get('state', '')}, "
                                      f"mpu_cal={data.get('mpu_cal', False)}, "
                                      f"mq_cal={data.get('mq_cal', False)}")
                
            elif msg_type == 'calibration':
                # Calibration progress
                sensor = data.get('sensor', '')
                status = data.get('status', '')
                self.get_logger().info(f"Calibration {sensor}: {status}")
                
            elif msg_type == 'self_test':
                # Self-test results
                component = data.get('component', '')
                status = data.get('status', '')
                self.get_logger().info(f"Self-test {component}: {status}")
                
            elif msg_type == 'watchdog':
                # Watchdog status
                status = data.get('status', '')
                self.get_logger().info(f"Watchdog: {status}")
                
            elif msg_type == 'error':
                # Error messages
                self.get_logger().error(f"ESP32 Error: {data.get('msg', '')}")
                
            elif msg_type == 'cmd_rx':
                # Command received acknowledgment
                pass  # Don't log every command
                
            elif msg_type == 'ack':
                # Command acknowledgment
                pass
                
        except json.JSONDecodeError:
            # Not JSON, ignore
            pass
        except Exception as e:
            self.get_logger().error(f"Error parsing telemetry: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ESP32SensorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
