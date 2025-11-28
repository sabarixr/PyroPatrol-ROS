#!/usr/bin/env python3
"""
Simple Autonomous Navigation Node
- Obstacle avoidance using LIDAR
- ArUco marker detection and approach
- No SLAM mapping required
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json
import time


class SimpleAutonomousNode(Node):
    def __init__(self):
        super().__init__('simple_autonomous_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.aruco_sub = self.create_subscription(
            Pose, '/aruco/pose', self.aruco_callback, 10)
        self.telemetry_sub = self.create_subscription(
            String, '/esp32_telemetry', self.telemetry_callback, 10)
        # Sensor fusion pose subscription (fused odometry/pose)
        self.fusion_sub = self.create_subscription(
            PoseStamped, '/sensor_fusion/pose', self.fusion_callback, 10
        )
        
        # Parameters
        self.declare_parameter('obstacle_distance', 0.5)  # meters
        self.declare_parameter('max_speed', 0.3)  # m/s
        self.declare_parameter('approach_duration', 2.0)  # seconds
        
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.approach_duration = self.get_parameter('approach_duration').value
        
        # State variables
        self.last_scan = None
        self.aruco_detected = False
        self.aruco_pose = None
    self.fusion_pose = None
        self.approaching_marker = False
        self.approach_start_time = None
        
        # Fire detection
        self.fire_detected = False
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Simple autonomous navigation started')
        self.get_logger().info(f'Obstacle distance: {self.obstacle_distance}m')
        self.get_logger().info(f'Max speed: {self.max_speed}m/s')
        self.get_logger().info(f'ArUco approach duration: {self.approach_duration}s')
    
    def scan_callback(self, msg):
        """Process LIDAR scan data"""
        self.last_scan = msg
    
    def aruco_callback(self, msg):
        """Process ArUco marker detection"""
        self.aruco_detected = True
        self.aruco_pose = msg
        
        # Log detection
        self.get_logger().info(
            f'ArUco marker detected! Position: '
            f'x={msg.position.x:.2f}m, '
            f'y={msg.position.y:.2f}m, '
            f'z={msg.position.z:.2f}m'
        )
        
        # Start approach if not already approaching
        if not self.approaching_marker:
            self.approaching_marker = True
            self.approach_start_time = time.time()
            self.get_logger().info('Starting ArUco marker approach!')
    
    def telemetry_callback(self, msg):
        """Process ESP32 telemetry for fire detection"""
        try:
            data = json.loads(msg.data)
            mq2 = data.get('mq2', 0)
            mq5 = data.get('mq5', 0)
            flame = data.get('flame', 0)
            
            # Simple fire detection
            if mq2 > 600 or mq5 > 550 or flame < 100:
                if not self.fire_detected:
                    self.get_logger().warn('FIRE DETECTED!')
                self.fire_detected = True
            else:
                self.fire_detected = False
        except:
            pass

    def fusion_callback(self, msg):
        """Receive fused pose from sensor fusion node"""
        self.fusion_pose = msg.pose
        # Log intentionally at debug level to avoid spamming
        self.get_logger().debug(
            f'Received fused pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
        )
    
    def check_obstacles(self):
        """Check for obstacles in front using LIDAR"""
        if self.last_scan is None:
            return False, False, False
        
        ranges = self.last_scan.ranges
        num_ranges = len(ranges)
        
        if num_ranges == 0:
            return False, False, False
        
        # Divide into 3 sectors: front, left, right
        sector_size = num_ranges // 3
        
        front_ranges = ranges[:sector_size] + ranges[-sector_size:]
        left_ranges = ranges[sector_size:2*sector_size]
        right_ranges = ranges[2*sector_size:3*sector_size]
        
        # Check minimum distance in each sector
        front_min = min([r for r in front_ranges if r > 0.0], default=float('inf'))
        left_min = min([r for r in left_ranges if r > 0.0], default=float('inf'))
        right_min = min([r for r in right_ranges if r > 0.0], default=float('inf'))
        
        front_obstacle = front_min < self.obstacle_distance
        left_obstacle = left_min < self.obstacle_distance
        right_obstacle = right_min < self.obstacle_distance
        
        return front_obstacle, left_obstacle, right_obstacle
    
    def control_loop(self):
        """Main control loop with priority system"""
        twist = Twist()
        
        # Check if approaching ArUco marker
        if self.approaching_marker:
            elapsed = time.time() - self.approach_start_time
            
            if elapsed < self.approach_duration:
                # Move forward for specified duration
                twist.linear.x = self.max_speed
                twist.angular.z = 0.0
                self.get_logger().info(
                    f'Approaching ArUco marker... {self.approach_duration - elapsed:.1f}s remaining'
                )
            else:
                # Stop after approach duration
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.approaching_marker = False
                self.aruco_detected = False
                self.get_logger().info('ArUco approach complete! Stopping.')
        else:
            # Normal obstacle avoidance mode
            front_obstacle, left_obstacle, right_obstacle = self.check_obstacles()
            
            if front_obstacle:
                # Obstacle in front - stop and turn
                twist.linear.x = 0.0
                
                # Turn away from obstacles
                if left_obstacle and not right_obstacle:
                    # Turn right
                    twist.angular.z = -0.5
                    self.get_logger().info('Obstacle in front and left - turning right')
                elif right_obstacle and not left_obstacle:
                    # Turn left
                    twist.angular.z = 0.5
                    self.get_logger().info('Obstacle in front and right - turning left')
                else:
                    # Turn left by default
                    twist.angular.z = 0.5
                    self.get_logger().info('Obstacle in front - turning left')
            else:
                # No obstacles - move forward
                twist.linear.x = self.max_speed * 0.5  # Half speed for safety
                twist.angular.z = 0.0
                
                # Gentle avoidance if obstacles on sides
                if left_obstacle:
                    twist.angular.z = -0.2  # Turn slightly right
                elif right_obstacle:
                    twist.angular.z = 0.2   # Turn slightly left
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAutonomousNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
