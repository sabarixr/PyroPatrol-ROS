#!/usr/bin/env python3
"""
Autonomous Fire Fighter Robot - Brain Node
Raspberry Pi makes intelligent navigation decisions based on:
- Fire/Gas sensor data from ESP32
- LIDAR obstacle avoidance
- Temperature gradients
Then sends movement commands to ESP32
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
import json
import math

class AutonomousFirebotNode(Node):
    def __init__(self):
        super().__init__('autonomous_firebot_node')
        
        # Parameters
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('min_obstacle_distance', 0.5)  # meters
        self.declare_parameter('fire_threshold_mq2', 600)
        self.declare_parameter('fire_threshold_mq5', 550)
        self.declare_parameter('temp_rise_threshold', 2.0)  # degrees
        
        self.max_speed = self.get_parameter('max_speed').value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.fire_threshold_mq2 = self.get_parameter('fire_threshold_mq2').value
        self.fire_threshold_mq5 = self.get_parameter('fire_threshold_mq5').value
        self.temp_rise_threshold = self.get_parameter('temp_rise_threshold').value
        
        # State variables
        self.sensor_data = {
            'mq2': 0,
            'mq5': 0,
            'flame': False,
            'temp': 25.0,
            'l_rpm': 0,
            'r_rpm': 0
        }
        self.baseline_temp = 25.0
        self.obstacle_detected = False
        self.obstacle_direction = None  # 'front', 'left', 'right'
        self.fire_detected = False
        self.autonomous_enabled = False
        
        # LIDAR data
        self.lidar_ranges = []
        self.min_front_distance = float('inf')
        self.min_left_distance = float('inf')
        self.min_right_distance = float('inf')
        
        # Subscribers
        self.telemetry_sub = self.create_subscription(
            String, '/esp32_telemetry', self.telemetry_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        self.aruco_sub = self.create_subscription(
            Pose, '/aruco/pose', self.aruco_callback, 10
        )
        self.mode_sub = self.create_subscription(
            Bool, '/autonomous_mode', self.mode_callback, 10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pump_pub = self.create_publisher(Bool, '/water_pump', 10)
        self.status_pub = self.create_publisher(String, '/autonomous_status', 10)
        
        # Control timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🧠 Autonomous Firebot Brain started!')
        self.get_logger().info('📡 Waiting for LIDAR and sensor data...')

    def telemetry_callback(self, msg):
        """Receive sensor data from ESP32"""
        try:
            data = json.loads(msg.data)
            self.sensor_data = data
            
            # Check for fire indicators
            mq2_high = data.get('mq2', 0) > self.fire_threshold_mq2
            mq5_high = data.get('mq5', 0) > self.fire_threshold_mq5
            temp_rise = (data.get('temp', 25) - self.baseline_temp) > self.temp_rise_threshold
            flame = data.get('flame', 0) == 0  # Flame sensor is active LOW
            
            self.fire_detected = mq2_high or mq5_high or temp_rise or flame
            
        except json.JSONDecodeError:
            pass

    def lidar_callback(self, msg):
        """Process LIDAR data for obstacle avoidance"""
        self.lidar_ranges = msg.ranges
        
        # Divide LIDAR into sectors (assumes 360° LIDAR)
        num_readings = len(msg.ranges)
        if num_readings == 0:
            return
        
        # Front sector: -30° to +30° (center)
        front_start = int(num_readings * (330/360))
        front_end = int(num_readings * (30/360))
        front_ranges = list(msg.ranges[front_start:]) + list(msg.ranges[:front_end])
        front_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]
        self.min_front_distance = min(front_ranges) if front_ranges else float('inf')
        
        # Left sector: 30° to 90°
        left_start = int(num_readings * (30/360))
        left_end = int(num_readings * (90/360))
        left_ranges = [r for r in msg.ranges[left_start:left_end] if msg.range_min < r < msg.range_max]
        self.min_left_distance = min(left_ranges) if left_ranges else float('inf')
        
        # Right sector: 270° to 330°
        right_start = int(num_readings * (270/360))
        right_end = int(num_readings * (330/360))
        right_ranges = [r for r in msg.ranges[right_start:right_end] if msg.range_min < r < msg.range_max]
        self.min_right_distance = min(right_ranges) if right_ranges else float('inf')
        
        # Determine obstacle direction
        if self.min_front_distance < self.min_obstacle_distance:
            self.obstacle_detected = True
            # Choose direction with more space
            if self.min_left_distance > self.min_right_distance:
                self.obstacle_direction = 'turn_left'
            else:
                self.obstacle_direction = 'turn_right'
        else:
            self.obstacle_detected = False
            self.obstacle_direction = None

    def mode_callback(self, msg):
        """Enable/disable autonomous mode"""
        self.autonomous_enabled = msg.data
        if self.autonomous_enabled:
            self.get_logger().info('🤖 Autonomous mode ENABLED')
        else:
            self.get_logger().info('🛑 Autonomous mode DISABLED')
            self.stop_robot()

    def control_loop(self):
        """Main brain - decision making happens here!"""
        if not self.autonomous_enabled:
            return
        
        twist = Twist()
        status = ""
        
        # Priority 1: OBSTACLE AVOIDANCE (Safety first!)
        if self.obstacle_detected:
            status = f"🚧 Obstacle {self.min_front_distance:.2f}m - {self.obstacle_direction}"
            
            if self.obstacle_direction == 'turn_left':
                twist.linear.x = 0.0
                twist.angular.z = 0.5  # Turn left
            elif self.obstacle_direction == 'turn_right':
                twist.linear.x = 0.0
                twist.angular.z = -0.5  # Turn right
            else:
                twist.linear.x = -0.2  # Back up
                twist.angular.z = 0.0
        
        # Priority 2: FIRE DETECTED - Navigate towards it!
        elif self.fire_detected:
            status = f"🔥 FIRE DETECTED! MQ2:{self.sensor_data.get('mq2',0)} MQ5:{self.sensor_data.get('mq5',0)}"
            
            # Move towards fire (forward)
            twist.linear.x = self.max_speed * 0.5  # Slower approach
            twist.angular.z = 0.0
            
            # If very close or flame detected, stop and activate pump
            if self.sensor_data.get('flame', 0) == 0:
                twist.linear.x = 0.0
                pump_msg = Bool()
                pump_msg.data = True
                self.pump_pub.publish(pump_msg)
                status += " - 💦 SPRAYING WATER!"

            # If we have an ArUco pose we can optionally steer towards it
            # (simple heuristic): if marker at non-zero x, adjust angular.z
            try:
                if hasattr(self, 'aruco_pose') and self.aruco_pose is not None:
                    # ArUco pose position.x is lateral offset in camera frame; use as angle proxy
                    x = self.aruco_pose.position.x
                    # small proportional steering
                    twist.angular.z = max(min(-0.5 * x, 0.6), -0.6)
            except Exception:
                pass
        
        # Priority 3: SEARCH PATTERN - Look for fire
        else:
            status = "🔍 Searching for fire..."
            
            # Simple search: move forward slowly, rotate occasionally
            twist.linear.x = self.max_speed * 0.3
            twist.angular.z = 0.1  # Slight rotation while moving
        
        # Send command
        self.cmd_vel_pub.publish(twist)
        
        # Publish status
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

    def stop_robot(self):
        """Emergency stop"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Turn off pump
        pump_msg = Bool()
        pump_msg.data = False
        self.pump_pub.publish(pump_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AutonomousFirebotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
