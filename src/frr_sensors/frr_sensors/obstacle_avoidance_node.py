#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Rover dimensions (from center to edge in meters)
        # LiDAR is offset: 16cm back, 8cm left, 10.5cm right, 10cm front
        self.declare_parameter('rover_width', 0.20)  # 20cm total width
        self.declare_parameter('rover_length', 0.30)  # 30cm total length
        self.declare_parameter('safety_margin', 0.25)  # 25cm safety margin (increased!)
        self.declare_parameter('stop_distance', 0.35)  # 35cm emergency stop (increased!)
        # Topic to subscribe for LaserScan (YDLidar publishes to /scan by default)
        self.declare_parameter('scan_topic', '/scan')
        
        self.rover_width = self.get_parameter('rover_width').value
        self.rover_length = self.get_parameter('rover_length').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.stop_distance = self.get_parameter('stop_distance').value
        
        # LiDAR detection and mode switching
        self.lidar_detected = False
        self.last_scan_time = None
        self.lidar_timeout = 3.0  # If no scan for 3 seconds, consider LiDAR absent
        self.mode_check_timer = self.create_timer(1.0, self.check_lidar_mode)
        
        # LiDAR offset from rover center
        self.lidar_offset_front = 0.10   # 10cm to front
        self.lidar_offset_back = 0.16    # 16cm to back
        self.lidar_offset_left = 0.08    # 8cm to left
        self.lidar_offset_right = 0.105  # 10.5cm to right
        
        # Subscribers
        self.scan_topic = self.get_parameter('scan_topic').value
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_teleop', self.cmd_vel_callback, 10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)

        # Periodic publisher: ensure teleop commands still reach motors even if
        # LiDAR scans aren't arriving (prevents teleop from being silent when
        # obstacle avoidance is enabled but /scan is missing). Publishes at 10Hz.
        self._periodic_pub = self.create_timer(0.1, self.periodic_publish)
        
        # Current commanded velocity
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        
        # Obstacle detection zones
        self.obstacle_front = False
        self.obstacle_back = False
        self.obstacle_left = False
        self.obstacle_right = False
        
        self.get_logger().info('Obstacle Avoidance Node started')
        self.get_logger().info(f'Subscribed to LiDAR scan topic: {self.scan_topic}')
        self.get_logger().info(f'Rover dimensions: {self.rover_length:.2f}m x {self.rover_width:.2f}m')
        self.get_logger().info(f'Safety margin: {self.safety_margin:.2f}m, Stop distance: {self.stop_distance:.2f}m')
        self.get_logger().info('⚠️  LiDAR auto-detection enabled - will switch to passthrough if no LiDAR detected')

    def check_lidar_mode(self):
        """Check if LiDAR is present and switch modes accordingly"""
        current_time = self.get_clock().now()
        
        if self.last_scan_time is None:
            # Never received a scan yet
            if not self.lidar_detected:
                self.get_logger().warn('⚠️  No LiDAR detected - running in PASSTHROUGH mode (no obstacle avoidance)', throttle_duration_sec=5.0)
            return
        
        # Check if we've received recent scans
        time_since_scan = (current_time - self.last_scan_time).nanoseconds / 1e9
        
        if time_since_scan > self.lidar_timeout:
            if self.lidar_detected:
                self.lidar_detected = False
                self.get_logger().warn(f'⚠️  LiDAR data lost (no scan for {time_since_scan:.1f}s) - switching to PASSTHROUGH mode')
        else:
            if not self.lidar_detected:
                self.lidar_detected = True
                self.get_logger().info('✓ LiDAR detected - obstacle avoidance ACTIVE')

    def cmd_vel_callback(self, msg):
        """Receive commanded velocity from teleop/navigation."""
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z

    def scan_callback(self, msg):
        """Process LiDAR scan and check for obstacles in all 4 corners."""
        # Mark that we received a scan
        self.last_scan_time = self.get_clock().now()
        
        # Reset obstacle flags
        self.obstacle_front = False
        self.obstacle_back = False
        self.obstacle_left = False
        self.obstacle_right = False
        
        # Define detection zones (in degrees, relative to LiDAR)
        # Front: -45° to +45°
        # Right: +45° to +135°
        # Back: +135° to -135° (or 135° to 225°)
        # Left: -135° to -45° (or 225° to 315°)
        
        for i, r in enumerate(msg.ranges):
            # Skip invalid measurements (including spurious 0.0 and very close readings)
            if r < 0.05 or r > msg.range_max or math.isnan(r) or math.isinf(r) or r == 0.0:
                continue
            
            # Calculate angle for this measurement
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)
            
            # Normalize angle to [-180, 180]
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360
            
            # Convert polar to Cartesian (in LiDAR frame)
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            # Check each zone with rover dimensions and LiDAR offset
            self.check_zone_front(x, y, r, angle_deg)
            self.check_zone_back(x, y, r, angle_deg)
            self.check_zone_left(x, y, r, angle_deg)
            self.check_zone_right(x, y, r, angle_deg)
        
        # Apply obstacle avoidance logic
        safe_cmd_vel = self.apply_obstacle_avoidance()
        
        # Publish safe command velocity
        self.cmd_vel_pub.publish(safe_cmd_vel)
        
        # Publish obstacle detection status
        obstacle_msg = Bool()
        obstacle_msg.data = any([self.obstacle_front, self.obstacle_back, 
                                self.obstacle_left, self.obstacle_right])
        self.obstacle_pub.publish(obstacle_msg)

    def check_zone_front(self, x, y, distance, angle_deg):
        """Check for obstacles in front zone."""
        # Front zone: -45° to +45°
        if -45 <= angle_deg <= 45:
            # Account for LiDAR being 10cm in front of center
            # Front edge is at lidar_offset_front + rover_length/2
            front_edge = self.lidar_offset_front + self.rover_length / 2
            
            # Check if obstacle is within safety zone
            if x > 0 and x < (front_edge + self.safety_margin):
                # Also check if it's within the width of the rover
                if abs(y) < (self.rover_width / 2 + self.safety_margin):
                    self.obstacle_front = True
                    if x < self.stop_distance and not hasattr(self, '_front_warned'):
                        self._front_warned = True
                        self.get_logger().warn(f'🛑 FRONT obstacle at {distance:.2f}m')
        else:
            if hasattr(self, '_front_warned'):
                delattr(self, '_front_warned')
    
    def check_zone_back(self, x, y, distance, angle_deg):
        """Check for obstacles in back zone."""
        # Back zone: 135° to 225° (or -180° to -135° and 135° to 180°)
        if angle_deg > 135 or angle_deg < -135:
            # Account for LiDAR being 16cm behind the back edge
            # Back edge is at -lidar_offset_back - rover_length/2
            back_edge = self.lidar_offset_back + self.rover_length / 2
            
            # Check if obstacle is behind
            if x < 0 and abs(x) < (back_edge + self.safety_margin):
                # Check if it's within the width
                if abs(y) < (self.rover_width / 2 + self.safety_margin):
                    self.obstacle_back = True
                    if abs(x) < self.stop_distance:
                        self.get_logger().warn(f'BACK obstacle at {distance:.2f}m, angle {angle_deg:.1f}°')

    def check_zone_left(self, x, y, distance, angle_deg):
        """Check for obstacles in left zone."""
        # Left zone: -135° to -45° (or 225° to 315°)
        if -135 <= angle_deg <= -45:
            # Account for LiDAR being 8cm left of center
            # Left edge is at -lidar_offset_left - rover_width/2
            left_edge = self.lidar_offset_left + self.rover_width / 2
            
            # Check if obstacle is on the left
            if y < 0 and abs(y) < (left_edge + self.safety_margin):
                # Check if it's within the length
                if abs(x) < (self.rover_length / 2 + self.safety_margin):
                    self.obstacle_left = True
                    if abs(y) < self.stop_distance:
                        self.get_logger().warn(f'LEFT obstacle at {distance:.2f}m, angle {angle_deg:.1f}°')

    def check_zone_right(self, x, y, distance, angle_deg):
        """Check for obstacles in right zone."""
        # Right zone: 45° to 135°
        if 45 <= angle_deg <= 135:
            # Account for LiDAR being 10.5cm right of center
            # Right edge is at lidar_offset_right + rover_width/2
            right_edge = self.lidar_offset_right + self.rover_width / 2
            
            # Check if obstacle is on the right
            if y > 0 and y < (right_edge + self.safety_margin):
                # Check if it's within the length
                if abs(x) < (self.rover_length / 2 + self.safety_margin):
                    self.obstacle_right = True
                    if y < self.stop_distance:
                        self.get_logger().warn(f'RIGHT obstacle at {distance:.2f}m, angle {angle_deg:.1f}°')

    def apply_obstacle_avoidance(self):
        """Apply obstacle avoidance logic based on detected obstacles."""
        safe_cmd = Twist()
        safe_cmd.linear.x = self.cmd_linear_x
        safe_cmd.linear.y = 0.0
        safe_cmd.linear.z = 0.0
        safe_cmd.angular.x = 0.0
        safe_cmd.angular.y = 0.0
        safe_cmd.angular.z = self.cmd_angular_z
        
        # PASSTHROUGH MODE: If LiDAR not detected, forward commands unchanged
        if not self.lidar_detected:
            return safe_cmd
        
        # OBSTACLE AVOIDANCE MODE: LiDAR is active
        # Forward motion check
        if self.cmd_linear_x > 0.0:  # Moving forward
            if self.obstacle_front:
                self.get_logger().info('Obstacle ahead - stopping forward motion')
                safe_cmd.linear.x = 0.0
                safe_cmd.angular.z = 0.0  # Also stop rotation
        
        # Backward motion check
        elif self.cmd_linear_x < 0.0:  # Moving backward
            if self.obstacle_back:
                self.get_logger().info('Obstacle behind - stopping reverse motion')
                safe_cmd.linear.x = 0.0
                safe_cmd.angular.z = 0.0  # Also stop rotation
        
        # Rotation check (left turn)
        if self.cmd_angular_z > 0.0:  # Turning left
            if self.obstacle_left:
                self.get_logger().info('Obstacle on left - stopping left turn')
                safe_cmd.angular.z = 0.0
                # Allow slight right turn to avoid
                if not self.obstacle_right:
                    safe_cmd.angular.z = -0.3
        
        # Rotation check (right turn)
        elif self.cmd_angular_z < 0.0:  # Turning right
            if self.obstacle_right:
                self.get_logger().info('Obstacle on right - stopping right turn')
                safe_cmd.angular.z = 0.0
                # Allow slight left turn to avoid
                if not self.obstacle_left:
                    safe_cmd.angular.z = 0.3
        
        # Emergency stop if obstacles on multiple sides
        if (self.obstacle_front and self.obstacle_back) or \
           (self.obstacle_left and self.obstacle_right):
            self.get_logger().warn('Obstacles on multiple sides - EMERGENCY STOP')
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = 0.0
        
        # Combined forward + rotation check
        if self.cmd_linear_x > 0.0 and self.cmd_angular_z != 0.0:
            # Check corners during turning
            if self.cmd_angular_z > 0.0:  # Forward + left
                if self.obstacle_front or self.obstacle_left:
                    safe_cmd.linear.x = 0.0
                    safe_cmd.angular.z = 0.0
            else:  # Forward + right
                if self.obstacle_front or self.obstacle_right:
                    safe_cmd.linear.x = 0.0
                    safe_cmd.angular.z = 0.0
        
        # Combined backward + rotation check
        if self.cmd_linear_x < 0.0 and self.cmd_angular_z != 0.0:
            # Check corners during turning while reversing
            if self.cmd_angular_z > 0.0:  # Backward + left
                if self.obstacle_back or self.obstacle_right:  # Note: reversed in reverse
                    safe_cmd.linear.x = 0.0
                    safe_cmd.angular.z = 0.0
            else:  # Backward + right
                if self.obstacle_back or self.obstacle_left:  # Note: reversed in reverse
                    safe_cmd.linear.x = 0.0
                    safe_cmd.angular.z = 0.0
        
        return safe_cmd

    def periodic_publish(self):
        """Periodically publish the latest (safe) command so motors receive
        teleop commands even when scan callbacks are not occurring.
        """
        safe_cmd = self.apply_obstacle_avoidance()
        # Publish the latest safe command
        try:
            self.cmd_vel_pub.publish(safe_cmd)
        except Exception:
            # Swallow publish errors to avoid crashing the node on transient issues
            pass
        # Also publish obstacle flag (will be False until scans set it)
        try:
            obstacle_msg = Bool()
            obstacle_msg.data = any([self.obstacle_front, self.obstacle_back,
                                     self.obstacle_left, self.obstacle_right])
            self.obstacle_pub.publish(obstacle_msg)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        obstacle_avoidance_node = ObstacleAvoidanceNode()
        rclpy.spin(obstacle_avoidance_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
