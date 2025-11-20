#!/usr/bin/env python3
"""
Debug tool: Check if obstacle avoidance is working
Shows real-time topic data flow
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

# Colors
RED = '\033[91m'
YELLOW = '\033[93m'
GREEN = '\033[92m'
CYAN = '\033[96m'
BLUE = '\033[94m'
WHITE = '\033[97m'
RESET = '\033[0m'
BOLD = '\033[1m'

class ObstacleDebugger(Node):
    def __init__(self):
        super().__init__('obstacle_debugger')
        
        # Subscribe to all relevant topics
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.teleop_sub = self.create_subscription(Twist, '/cmd_vel_teleop', self.teleop_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Data storage
        self.last_scan_time = None
        self.last_teleop_time = None
        self.last_cmd_vel_time = None
        
        self.front_distance = float('inf')
        self.teleop_speed = 0.0
        self.actual_speed = 0.0
        
        self.scan_count = 0
        self.teleop_count = 0
        self.cmd_vel_count = 0
        
        # Timer for display
        self.timer = self.create_timer(0.5, self.display_status)
        
        print(f"\n{CYAN}{'='*70}{RESET}")
        print(f"{BOLD}{CYAN}Obstacle Avoidance Debugger{RESET}")
        print(f"{CYAN}{'='*70}{RESET}\n")
        print(f"{GREEN}Monitoring topics...{RESET}\n")
    
    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()
        self.scan_count += 1
        
        # Get front distance (±30 degrees)
        ranges = list(msg.ranges)
        num_points = len(ranges)
        
        if num_points == 0:
            return
        
        front_distances = []
        for i, distance in enumerate(ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue
            
            angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360
            
            if -30 <= angle_deg <= 30:
                front_distances.append(distance)
        
        if front_distances:
            self.front_distance = min(front_distances)
    
    def teleop_callback(self, msg):
        self.last_teleop_time = self.get_clock().now()
        self.teleop_count += 1
        self.teleop_speed = msg.linear.x
    
    def cmd_vel_callback(self, msg):
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_count += 1
        self.actual_speed = msg.linear.x
    
    def display_status(self):
        # Clear screen
        print('\033[2J\033[H', end='')
        
        print(f"{CYAN}{'='*70}{RESET}")
        print(f"{BOLD}{CYAN}Obstacle Avoidance Debug Status{RESET}")
        print(f"{CYAN}{'='*70}{RESET}\n")
        
        # Topic activity
        print(f"{BOLD}Topic Activity:{RESET}")
        print(f"  /scan:          {self.format_activity(self.scan_count, self.last_scan_time)}")
        print(f"  /cmd_vel_teleop: {self.format_activity(self.teleop_count, self.last_teleop_time)}")
        print(f"  /cmd_vel:        {self.format_activity(self.cmd_vel_count, self.last_cmd_vel_time)}")
        
        print(f"\n{CYAN}{'─'*70}{RESET}\n")
        
        # Current values
        print(f"{BOLD}Current Values:{RESET}")
        print(f"  Front Distance:   {self.format_distance(self.front_distance)}")
        print(f"  Teleop Command:   {self.format_speed(self.teleop_speed)} m/s (what YOU want)")
        print(f"  Actual Command:   {self.format_speed(self.actual_speed)} m/s (what MOTORS get)")
        
        print(f"\n{CYAN}{'─'*70}{RESET}\n")
        
        # Analysis
        print(f"{BOLD}Analysis:{RESET}")
        
        if self.scan_count == 0:
            print(f"  {RED}✗ NO LIDAR DATA - /scan not receiving messages{RESET}")
            print(f"    → Check: ros2 topic hz /scan")
        else:
            print(f"  {GREEN}✓ LiDAR is working{RESET}")
        
        if self.teleop_count == 0:
            print(f"  {YELLOW}⚠ No teleop commands yet{RESET}")
            print(f"    → Start teleop and press keys")
        else:
            print(f"  {GREEN}✓ Teleop is publishing{RESET}")
        
        if self.cmd_vel_count == 0:
            print(f"  {RED}✗ NO MOTOR COMMANDS - /cmd_vel not receiving messages{RESET}")
            print(f"    → Obstacle avoidance node might not be running!")
        else:
            print(f"  {GREEN}✓ Motor commands being sent{RESET}")
        
        # Check if obstacle avoidance is working
        if self.teleop_count > 0 and self.cmd_vel_count > 0:
            if self.front_distance < 0.35 and self.teleop_speed > 0 and self.actual_speed < self.teleop_speed * 0.5:
                print(f"\n  {GREEN}{BOLD}✓ OBSTACLE AVOIDANCE IS WORKING!{RESET}")
                print(f"    Front obstacle at {self.front_distance:.2f}m, speed reduced")
            elif self.front_distance < 0.35 and self.teleop_speed > 0 and self.actual_speed >= self.teleop_speed * 0.5:
                print(f"\n  {RED}{BOLD}✗ OBSTACLE AVOIDANCE NOT WORKING!{RESET}")
                print(f"    Front obstacle at {self.front_distance:.2f}m but speed not reduced!")
                print(f"    Teleop: {self.teleop_speed:.2f} → Motors: {self.actual_speed:.2f}")
            elif abs(self.teleop_speed - self.actual_speed) < 0.01:
                print(f"\n  {GREEN}✓ Commands passing through normally (no obstacles){RESET}")
        
        print(f"\n{CYAN}{'─'*70}{RESET}")
        print(f"\n{YELLOW}To test:{RESET}")
        print(f"  1. Put box 30cm in front of robot")
        print(f"  2. In teleop, press 'w' to drive forward")
        print(f"  3. Watch if 'Actual Command' drops to near 0")
        print(f"\n{CYAN}Press Ctrl+C to exit{RESET}")
    
    def format_activity(self, count, last_time):
        if count == 0:
            return f"{RED}No messages (0){RESET}"
        
        if last_time is None:
            return f"{YELLOW}{count} msgs{RESET}"
        
        age = (self.get_clock().now() - last_time).nanoseconds / 1e9
        if age < 1.0:
            return f"{GREEN}{count} msgs (active){RESET}"
        else:
            return f"{YELLOW}{count} msgs ({age:.1f}s ago){RESET}"
    
    def format_distance(self, dist):
        if math.isinf(dist) or dist > 5.0:
            return f"{GREEN}>5.0m (clear){RESET}"
        elif dist < 0.35:
            return f"{RED}{BOLD}{dist:.2f}m (DANGER!){RESET}"
        elif dist < 0.50:
            return f"{YELLOW}{dist:.2f}m (warning){RESET}"
        else:
            return f"{GREEN}{dist:.2f}m (safe){RESET}"
    
    def format_speed(self, speed):
        if abs(speed) < 0.01:
            return f"{WHITE}{speed:.2f}{RESET}"
        elif speed > 0:
            return f"{GREEN}{speed:.2f}{RESET}"
        else:
            return f"{YELLOW}{speed:.2f}{RESET}"

def main(args=None):
    rclpy.init(args=args)
    debugger = ObstacleDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        print(f"\n\n{CYAN}Debugger stopped.{RESET}\n")
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
