#!/usr/bin/env python3
"""
ESP32 Sensor Node - Reads sensor data from ESP32 telemetry
Replaces direct sensor reading with data from ESP32 bridge
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Range, Temperature
import json

class ESP32SensorNode(Node):
    def __init__(self):
        super().__init__('esp32_sensor_node')
        
        # Subscribe to ESP32 telemetry
        self.telemetry_sub = self.create_subscription(
            String, '/esp32_telemetry', self.telemetry_callback, 10
        )
        
        # Publishers for processed sensor data
        self.fire_detected_pub = self.create_publisher(Bool, '/fire_detected', 10)
        self.hazard_level_pub = self.create_publisher(String, '/hazard_level', 10)
        
        # Thresholds (from ESP32 code)
        self.MQ2_THRESHOLD = 600  # Smoke
        self.MQ5_THRESHOLD = 550  # Gas
        self.TEMP_RISE_MIN = 2.0  # Temperature rise
        
        # Baseline temperature for rise detection
        self.baseline_temp = None
        
        # Latest sensor readings
        self.latest_mq2 = 0
        self.latest_mq5 = 0
        self.latest_flame = 1  # 1 = no flame
        self.latest_temp = 0.0
        
        self.get_logger().info('ESP32 Sensor Node started')

    def telemetry_callback(self, msg):
        """Process telemetry data from ESP32"""
        try:
            data = json.loads(msg.data)
            
            # Update latest readings
            self.latest_mq2 = data.get('mq2', 0)
            self.latest_mq5 = data.get('mq5', 0)
            self.latest_flame = data.get('flame', 1)
            self.latest_temp = data.get('temp', 0.0)
            
            # Set baseline temperature if not set
            if self.baseline_temp is None:
                self.baseline_temp = self.latest_temp
            
            # Check for fire conditions
            fire_detected = self.check_fire_conditions()
            
            # Publish fire detection status
            fire_msg = Bool()
            fire_msg.data = fire_detected
            self.fire_detected_pub.publish(fire_msg)
            
            # Publish hazard level
            hazard_level = self.calculate_hazard_level()
            hazard_msg = String()
            hazard_msg.data = hazard_level
            self.hazard_level_pub.publish(hazard_msg)
            
            if fire_detected:
                self.get_logger().warn(f'🔥 FIRE DETECTED! Hazard: {hazard_level}')
            
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid telemetry JSON: {msg.data}')

    def check_fire_conditions(self):
        """Check if fire conditions are met"""
        # Flame sensor (LOW = flame detected)
        flame_detected = (self.latest_flame == 0)
        
        # Smoke detection
        smoke_detected = (self.latest_mq2 > self.MQ2_THRESHOLD)
        
        # Gas detection
        gas_detected = (self.latest_mq5 > self.MQ5_THRESHOLD)
        
        # Temperature rise
        temp_rise = (self.latest_temp - self.baseline_temp) if self.baseline_temp else 0
        temp_spike = (temp_rise > self.TEMP_RISE_MIN)
        
        # Fire detected if any critical condition is met
        return flame_detected or (smoke_detected and temp_spike) or (gas_detected and temp_spike)

    def calculate_hazard_level(self):
        """Calculate overall hazard level"""
        hazard_score = 0
        
        # Flame sensor
        if self.latest_flame == 0:
            hazard_score += 3
        
        # Smoke level
        if self.latest_mq2 > self.MQ2_THRESHOLD:
            hazard_score += 2
        elif self.latest_mq2 > (self.MQ2_THRESHOLD * 0.7):
            hazard_score += 1
        
        # Gas level
        if self.latest_mq5 > self.MQ5_THRESHOLD:
            hazard_score += 2
        elif self.latest_mq5 > (self.MQ5_THRESHOLD * 0.7):
            hazard_score += 1
        
        # Temperature
        if self.baseline_temp:
            temp_rise = self.latest_temp - self.baseline_temp
            if temp_rise > self.TEMP_RISE_MIN * 2:
                hazard_score += 2
            elif temp_rise > self.TEMP_RISE_MIN:
                hazard_score += 1
        
        # Classify hazard level
        if hazard_score >= 5:
            return "CRITICAL"
        elif hazard_score >= 3:
            return "HIGH"
        elif hazard_score >= 1:
            return "MODERATE"
        else:
            return "LOW"


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ESP32SensorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()