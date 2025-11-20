#!/usr/bin/env python3
"""
LiDAR Motor Diagnostic - Check if LiDAR is spinning properly
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

# Colors
RED = '\033[91m'
YELLOW = '\033[93m'
GREEN = '\033[92m'
CYAN = '\033[96m'
RESET = '\033[0m'
BOLD = '\033[1m'


class LidarMotorCheck(Node):
    def __init__(self):
        super().__init__('lidar_motor_check')
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/lidar/raw',
            self.raw_callback,
            10)
        
        self.angles_seen = []
        self.frame_count = 0
        self.start_time = time.time()
        
        print(f"\n{CYAN}{'='*70}{RESET}")
        print(f"{BOLD}{CYAN}LiDAR Motor Diagnostic Tool{RESET}")
        print(f"{CYAN}{'='*70}{RESET}\n")
        print(f"{GREEN}Listening to /lidar/raw topic...{RESET}")
        print(f"{YELLOW}Checking if LiDAR is spinning 360°...{RESET}\n")
        
        # Timer to print status
        self.timer = self.create_timer(2.0, self.print_status)
    
    def raw_callback(self, msg):
        self.frame_count += 1
        if len(msg.data) > 0:
            angle_start = msg.data[0]
            self.angles_seen.append(angle_start)
    
    def print_status(self):
        print('\033[2J\033[H', end='')  # Clear screen
        
        elapsed = time.time() - self.start_time
        
        print(f"{BOLD}{CYAN}LiDAR Motor Status{RESET}")
        print(f"{CYAN}{'='*70}{RESET}\n")
        
        print(f"Runtime: {elapsed:.1f}s")
        print(f"Frames received: {self.frame_count}")
        print(f"Rate: {self.frame_count/elapsed:.1f} frames/sec\n")
        
        if not self.angles_seen:
            print(f"{RED}{BOLD}❌ NO DATA from LiDAR!{RESET}")
            print(f"{YELLOW}Check:{RESET}")
            print(f"  1. Is LiDAR powered? (should have LED/light)")
            print(f"  2. Is USB cable connected? (ls /dev/ttyUSB*)")
            print(f"  3. Is lidar_node running? (ros2 node list)")
            return
        
        min_angle = min(self.angles_seen[-50:] if len(self.angles_seen) > 50 else self.angles_seen)
        max_angle = max(self.angles_seen[-50:] if len(self.angles_seen) > 50 else self.angles_seen)
        coverage = max_angle - min_angle
        
        print(f"{BOLD}Angle Coverage (last 50 frames):{RESET}")
        print(f"  Min angle: {min_angle:.1f}°")
        print(f"  Max angle: {max_angle:.1f}°")
        print(f"  Coverage:  {coverage:.1f}°\n")
        
        # Diagnosis
        if coverage > 300:
            print(f"{GREEN}{BOLD}✅ LiDAR IS SPINNING FULL 360°!{RESET}")
            print(f"{GREEN}Motor is working correctly.{RESET}\n")
        elif coverage > 180:
            print(f"{YELLOW}{BOLD}⚠️  LiDAR partial rotation ({coverage:.0f}°){RESET}")
            print(f"{YELLOW}Motor might be slow or obstructed.{RESET}\n")
        elif coverage > 90:
            print(f"{YELLOW}{BOLD}⚠️  LiDAR limited rotation ({coverage:.0f}°){RESET}")
            print(f"{YELLOW}Motor appears to be stuck or very slow.{RESET}\n")
        else:
            print(f"{RED}{BOLD}❌ LiDAR NOT SPINNING! ({coverage:.0f}° coverage){RESET}")
            print(f"{RED}Motor is stuck or not powered!{RESET}\n")
            print(f"{YELLOW}Troubleshooting:{RESET}")
            print(f"  1. Check if LiDAR motor LED is blinking")
            print(f"  2. Listen for motor spinning sound")
            print(f"  3. Restart LiDAR: unplug USB, wait 5s, replug")
            print(f"  4. Check power supply (should be 5V)")
            print(f"  5. Try: sudo chmod 666 /dev/ttyUSB0")
        
        print(f"\n{CYAN}{'─'*70}{RESET}")
        print(f"\n{CYAN}Press Ctrl+C to exit{RESET}")


def main(args=None):
    rclpy.init(args=args)
    checker = LidarMotorCheck()
    
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        print(f"\n\n{CYAN}Diagnostic stopped.{RESET}\n")
    finally:
        checker.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
