#!/usr/bin/env python3
"""
Fire Fighter Rover - Autonomous Navigation Brain
Raspberry Pi analyzes sensor data and decides where to go
ESP32 is just the muscle that executes commands
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import Twist
import json
import time

class AutonomousNavigationNode(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/nav_status', 10)
        
        # Subscribers - ESP32 sends sensor data
        self.sensor_sub = self.create_subscription(
            String, '/esp32/sensors', self.sensor_callback, 10
        )
        
        # Sensor readings from ESP32
        self.mq5_value = 0.0      # LPG/Natural gas sensor
        self.mq6_value = 0.0      # LPG/Propane sensor  
        self.temperature = 0.0     # Temperature sensor
        self.obstacle_distance = 100.0  # Ultrasonic/IR distance
        self.imu_data = {'ax': 0, 'ay': 0, 'az': 0}
        
        # Navigation parameters
        self.declare_parameter('fire_threshold_temp', 40.0)  # °C
        self.declare_parameter('gas_threshold_mq5', 500.0)   # PPM
        self.declare_parameter('gas_threshold_mq6', 500.0)   # PPM
        self.declare_parameter('obstacle_threshold', 30.0)   # cm
        self.declare_parameter('search_speed', 0.3)          # m/s
        self.declare_parameter('approach_speed', 0.2)        # m/s
        self.declare_parameter('turn_speed', 0.5)            # rad/s
        
        # Get parameters
        self.fire_threshold = self.get_parameter('fire_threshold_temp').value
        self.gas_threshold_mq5 = self.get_parameter('gas_threshold_mq5').value
        self.gas_threshold_mq6 = self.get_parameter('gas_threshold_mq6').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.search_speed = self.get_parameter('search_speed').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        
        # State machine
        self.state = 'SEARCHING'  # SEARCHING, APPROACHING, AVOIDING, ARRIVED
        self.previous_readings = []
        self.max_reading_history = 5
        
        # Decision making timer (10 Hz)
        self.decision_timer = self.create_timer(0.1, self.make_decision)
        
        self.get_logger().info('🧠 Autonomous Navigation Brain started')
        self.get_logger().info(f'Fire threshold: {self.fire_threshold}°C')
        self.get_logger().info(f'Gas thresholds: MQ5={self.gas_threshold_mq5}, MQ6={self.gas_threshold_mq6}')

    def sensor_callback(self, msg):
        """Receive sensor data from ESP32 muscle"""
        try:
            data = json.loads(msg.data)
            
            # Update sensor readings
            if 'mq5' in data:
                self.mq5_value = data['mq5']
            if 'mq6' in data:
                self.mq6_value = data['mq6']
            if 'temperature' in data:
                self.temperature = data['temperature']
            if 'obstacle' in data:
                self.obstacle_distance = data['obstacle']
            if 'imu' in data:
                self.imu_data = data['imu']
                
            # Store reading history for trend analysis
            reading = {
                'mq5': self.mq5_value,
                'mq6': self.mq6_value,
                'temp': self.temperature,
                'time': time.time()
            }
            self.previous_readings.append(reading)
            
            # Keep only recent history
            if len(self.previous_readings) > self.max_reading_history:
                self.previous_readings.pop(0)
                
        except Exception as e:
            self.get_logger().error(f'Failed to parse sensor data: {e}')

    def make_decision(self):
        """Brain's decision-making loop - analyzes sensors and decides movement"""
        
        # Calculate danger intensity (higher = more dangerous/closer to fire)
        danger_score = self.calculate_danger_score()
        
        # Check for obstacles first (safety priority)
        if self.obstacle_distance < self.obstacle_threshold:
            self.state = 'AVOIDING'
            self.avoid_obstacle()
            return
        
        # Check if we've arrived at fire/gas source
        if self.is_at_target():
            self.state = 'ARRIVED'
            self.stop_and_report()
            return
        
        # Analyze if we're getting closer to danger
        is_getting_closer = self.is_trend_increasing()
        
        if danger_score > 0:
            # Detected fire or gas
            self.state = 'APPROACHING'
            if is_getting_closer:
                # Keep going forward, we're on the right track
                self.move_forward()
            else:
                # We're moving away, need to search
                self.search_pattern()
        else:
            # No fire/gas detected yet, search
            self.state = 'SEARCHING'
            self.search_pattern()
        
        # Publish status
        self.publish_status(danger_score)

    def calculate_danger_score(self):
        """Calculate how dangerous/close to fire we are"""
        score = 0.0
        
        # Temperature contribution
        if self.temperature > self.fire_threshold:
            temp_score = (self.temperature - self.fire_threshold) / 10.0
            score += temp_score
            
        # MQ5 gas sensor contribution
        if self.mq5_value > self.gas_threshold_mq5:
            mq5_score = (self.mq5_value - self.gas_threshold_mq5) / 1000.0
            score += mq5_score
            
        # MQ6 gas sensor contribution  
        if self.mq6_value > self.gas_threshold_mq6:
            mq6_score = (self.mq6_value - self.gas_threshold_mq6) / 1000.0
            score += mq6_score
            
        return score

    def is_trend_increasing(self):
        """Check if danger readings are increasing (getting closer to source)"""
        if len(self.previous_readings) < 3:
            return False
            
        # Compare recent readings to older ones
        recent = self.previous_readings[-2:]
        older = self.previous_readings[:2]
        
        recent_avg = sum(r['mq5'] + r['mq6'] + r['temp'] for r in recent) / len(recent)
        older_avg = sum(r['mq5'] + r['mq6'] + r['temp'] for r in older) / len(older)
        
        return recent_avg > older_avg

    def is_at_target(self):
        """Check if we're at the fire/gas source"""
        # Very high readings mean we're at the source
        high_temp = self.temperature > self.fire_threshold * 1.5
        high_mq5 = self.mq5_value > self.gas_threshold_mq5 * 2
        high_mq6 = self.mq6_value > self.gas_threshold_mq6 * 2
        
        return high_temp or high_mq5 or high_mq6

    def move_forward(self):
        """Move forward towards increasing danger"""
        twist = Twist()
        twist.linear.x = self.approach_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info('🎯 Moving forward towards target')

    def search_pattern(self):
        """Search pattern - slow turn to scan for fire/gas"""
        twist = Twist()
        twist.linear.x = self.search_speed * 0.5  # Slow forward
        twist.angular.z = self.turn_speed * 0.5   # Gentle turn
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info('🔍 Searching for fire/gas source')

    def avoid_obstacle(self):
        """Obstacle detected - turn away"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.turn_speed  # Turn left
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().warn(f'🚧 Obstacle at {self.obstacle_distance:.1f}cm - avoiding!')

    def stop_and_report(self):
        """Stop at target and report"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info('🔥 ARRIVED AT FIRE/GAS SOURCE!')
        self.get_logger().info(f'   Temperature: {self.temperature:.1f}°C')
        self.get_logger().info(f'   MQ5: {self.mq5_value:.1f} PPM')
        self.get_logger().info(f'   MQ6: {self.mq6_value:.1f} PPM')

    def publish_status(self, danger_score):
        """Publish navigation status"""
        status = {
            'state': self.state,
            'danger_score': danger_score,
            'temperature': self.temperature,
            'mq5': self.mq5_value,
            'mq6': self.mq6_value,
            'obstacle_distance': self.obstacle_distance
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AutonomousNavigationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()