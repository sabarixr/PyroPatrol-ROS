#!/usr/bin/env python3
"""
Simple ArUco Follower Node
- Subscribes to /aruco/pose and /scan (obstacle detection)
- Moves forward for 2 seconds when ArUco marker detected
- Stops if obstacle detected
- Logs all ArUco detections clearly
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import time


class ArucoFollowerNode(Node):
    def __init__(self):
        super().__init__('aruco_follower_node')
        
        # Parameters
        self.declare_parameter('forward_speed', 0.2)  # m/s
        self.declare_parameter('move_duration', 2.0)  # seconds
        self.declare_parameter('min_obstacle_distance', 0.5)  # meters
        
        self.forward_speed = self.get_parameter('forward_speed').value
        self.move_duration = self.get_parameter('move_duration').value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        
        # State variables
        self.moving = False
        self.move_start_time = None
        self.obstacle_detected = False
        self.last_aruco_pose = None
        self.aruco_detection_count = 0
        
        # Subscriptions
        self.aruco_sub = self.create_subscription(
            Pose,
            '/aruco/pose',
            self.aruco_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('═══════════════════════════════════════════════')
        self.get_logger().info('   ArUco Follower Node Started')
        self.get_logger().info('═══════════════════════════════════════════════')
        self.get_logger().info(f'  Forward speed: {self.forward_speed} m/s')
        self.get_logger().info(f'  Move duration: {self.move_duration} seconds')
        self.get_logger().info(f'  Min obstacle distance: {self.min_obstacle_distance} m')
        self.get_logger().info('═══════════════════════════════════════════════')
        self.get_logger().info('Waiting for ArUco markers...')
        self.get_logger().info('')

    def aruco_callback(self, msg):
        """Handle ArUco marker detection"""
        self.last_aruco_pose = msg
        self.aruco_detection_count += 1
        
        # Log detection with clear formatting
        distance = msg.position.z
        x_offset = msg.position.x
        y_offset = msg.position.y
        
        self.get_logger().info('┌─────────────────────────────────────────────┐')
        self.get_logger().info('│         🎯 ARUCO MARKER DETECTED            │')
        self.get_logger().info('├─────────────────────────────────────────────┤')
        self.get_logger().info(f'│  Detection #{self.aruco_detection_count:04d}                           │')
        self.get_logger().info(f'│  Distance:  {distance:.3f} m                     │')
        self.get_logger().info(f'│  X offset:  {x_offset:+.3f} m (left/right)       │')
        self.get_logger().info(f'│  Y offset:  {y_offset:+.3f} m (up/down)          │')
        self.get_logger().info('└─────────────────────────────────────────────┘')
        
        # Start moving if not already moving and no obstacle
        if not self.moving and not self.obstacle_detected:
            self.get_logger().info('🚀 Starting forward movement...')
            self.moving = True
            self.move_start_time = self.get_clock().now()
        elif self.obstacle_detected:
            self.get_logger().warn('⚠️  Obstacle detected! Cannot move forward.')

    def scan_callback(self, msg):
        """Check for obstacles in front"""
        if len(msg.ranges) == 0:
            return

        num_readings = len(msg.ranges)
        
    
        front_indices = []
        front_indices.extend(range(0, int(num_readings * 0.083)))  
        front_indices.extend(range(int(num_readings * 0.917), num_readings))  
        front_distances = [msg.ranges[i] for i in front_indices if 0.0 < msg.ranges[i] < 12.0]
        
        if front_distances:
            min_distance = min(front_distances)
            
            previous_state = self.obstacle_detected
            self.obstacle_detected = min_distance < self.min_obstacle_distance
        
            if self.obstacle_detected and not previous_state:
                self.get_logger().warn('╔═══════════════════════════════════════════╗')
                self.get_logger().warn('║   ⚠️  OBSTACLE DETECTED IN FRONT ⚠️       ║')
                self.get_logger().warn('╚═══════════════════════════════════════════╝')
                self.get_logger().warn(f'   Minimum distance: {min_distance:.2f} m')
                self.get_logger().warn('')
                
                # Stop moving if obstacle detected
                if self.moving:
                    self.moving = False
                    self.get_logger().warn('🛑 Stopping due to obstacle!')
            
            elif not self.obstacle_detected and previous_state:
                self.get_logger().info('✅ Obstacle cleared - path is clear')
                self.get_logger().info('')
            
            # Publish obstacle status
            obstacle_msg = Bool()
            obstacle_msg.data = self.obstacle_detected
            self.obstacle_pub.publish(obstacle_msg)

    def control_loop(self):
        """Main control loop"""
        cmd = Twist()
        
        if self.moving:
            # Check if move duration has elapsed
            elapsed = (self.get_clock().now() - self.move_start_time).nanoseconds / 1e9
            
            if elapsed < self.move_duration:
                # Still moving forward
                if not self.obstacle_detected:
                    cmd.linear.x = self.forward_speed
                    
                    # Log progress every 0.5 seconds
                    if int(elapsed * 2) % 1 == 0:  # Every 0.5 seconds
                        remaining = self.move_duration - elapsed
                        self.get_logger().info(f'⏱️  Moving forward... {remaining:.1f}s remaining')
                else:
                    # Obstacle detected during movement
                    self.moving = False
                    self.get_logger().warn('🛑 Movement stopped - obstacle detected!')
            else:
                # Duration complete
                self.moving = False
                self.get_logger().info('✅ Forward movement complete!')
                self.get_logger().info('')
                self.get_logger().info('Waiting for next ArUco detection...')
                self.get_logger().info('')
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)

    def destroy_node(self):
        """Cleanup on shutdown"""
        # Stop the robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('ArUco Follower Node shutting down')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
