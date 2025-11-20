#!/usr/bin/env python3
"""
Improved LiDAR Visualization Tool
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
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


class LidarViewer(Node):
    def __init__(self):
        super().__init__('lidar_viewer')
        self.declare_parameter('angle_offset_deg', 0.0)
        self.angle_offset = float(self.get_parameter('angle_offset_deg').value)

        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos)

        print(f"\n{CYAN}{'='*70}{RESET}")
        print(f"{BOLD}{CYAN}LiDAR Real-Time Viewer{RESET}")
        print(f"{CYAN}{'='*70}{RESET}\n")
        print(f"{GREEN}Listening to /scan topic...{RESET}\n")

        self.msg_count = 0
        self.timeout_count = 0
        
        # Timer to check if we're receiving data
        self.check_timer = self.create_timer(3.0, self.check_connection)

    def check_connection(self):
        """Check if we're receiving scan data."""
        if self.msg_count == 0:
            self.timeout_count += 1
            print(f"{YELLOW}⚠ Waiting for /scan data... ({self.timeout_count * 3}s){RESET}")
            print(f"{YELLOW}Troubleshooting:{RESET}")
            print(f"  1. Is robot running? Check other terminal")
            print(f"  2. Run: {CYAN}ros2 topic list | grep scan{RESET}")
            print(f"  3. Run: {CYAN}ros2 topic hz /scan{RESET}")
            print(f"  4. Source workspace: {CYAN}source ~/frr_ws/install/setup.bash{RESET}\n")

    def scan_callback(self, msg):
        if self.msg_count == 0:
            # First message received!
            self.check_timer.cancel()
        
        self.msg_count += 1
        print('\033[2J\033[H', end='')

        ranges = list(msg.ranges)
        num_points = len(ranges)

        print(f"{BOLD}{CYAN}LiDAR Scan #{self.msg_count}{RESET}")
        print(f"Angles: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}° (inc: {math.degrees(msg.angle_increment):.2f}°)")
        print(f"Range: {msg.range_min:.2f}m to {msg.range_max:.2f}m\n")

        if num_points == 0:
            print(f"{RED}No LiDAR data!{RESET}")
            return

        # Zone boundaries
        FRONT_MIN, FRONT_MAX = -30.0, 30.0
        LEFT_MIN, LEFT_MAX = 30.0, 150.0
        RIGHT_MIN, RIGHT_MAX = -150.0, -30.0

        front_distances = []
        left_distances = []
        right_distances = []
        back_distances = []
        
        valid_count = 0
        invalid_count = 0
        
        # Debug: track actual angle ranges seen
        angles_seen = []

        for i, distance in enumerate(ranges):
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad) + self.angle_offset
            
            while angle_deg > 180.0:
                angle_deg -= 360.0
            while angle_deg < -180.0:
                angle_deg += 360.0

            # Filter invalid readings
            if math.isinf(distance) or math.isnan(distance) or distance <= 0.05 or distance < msg.range_min or distance > msg.range_max:
                invalid_count += 1
                continue

            valid_count += 1
            angles_seen.append(angle_deg)

            if FRONT_MIN <= angle_deg <= FRONT_MAX:
                front_distances.append(distance)
            elif LEFT_MIN < angle_deg <= LEFT_MAX:
                left_distances.append(distance)
            elif RIGHT_MIN <= angle_deg < RIGHT_MAX:
                right_distances.append(distance)
            else:
                back_distances.append(distance)

        front_min = min(front_distances) if front_distances else float('inf')
        left_min = min(left_distances) if left_distances else float('inf')
        right_min = min(right_distances) if right_distances else float('inf')
        back_min = min(back_distances) if back_distances else float('inf')

        print(f"{BOLD}{CYAN}{'='*70}{RESET}")
        print(f"{BOLD}{CYAN}LiDAR Distance View{RESET}")
        print(f"{CYAN}{'='*70}{RESET}\n")

        print(f"              {self.format_distance(front_min, 'FRONT')}")
        print(f"                    ↑")
        print(f"                    |")
        print(f"  {self.format_distance(left_min, 'LEFT')} ← [ROBOT] → {self.format_distance(right_min, 'RIGHT')}")
        print(f"                    |")
        print(f"                    ↓")
        print(f"              {self.format_distance(back_min, 'BACK')}")

        print(f"\n{CYAN}{'─'*70}{RESET}\n")
        print(f"{BOLD}Zone Status:{RESET}\n")

        self.print_zone("FRONT", front_min, front_distances)
        self.print_zone("LEFT ", left_min, left_distances)
        self.print_zone("RIGHT", right_min, right_distances)
        self.print_zone("BACK ", back_min, back_distances)

        print(f"\n{CYAN}{'─'*70}{RESET}\n")
        print(f"Points: {valid_count} valid, {invalid_count} invalid ({num_points} total)")
        
        # Show actual angle coverage
        if angles_seen:
            min_angle = min(angles_seen)
            max_angle = max(angles_seen)
            print(f"{BOLD}Actual angles seen: {min_angle:.1f}° to {max_angle:.1f}° (coverage: {max_angle-min_angle:.1f}°){RESET}")
            print(f"{YELLOW}Zone definitions: F=±30°, L=30-150°, R=-150 to -30°, B=rest{RESET}")
        print()

        danger = 0.35
        warn = 0.50
        warnings = []
        
        if front_min < danger:
            warnings.append(f"{RED}{BOLD}⚠ STOP! Front {front_min:.2f}m{RESET}")
        elif front_min < warn:
            warnings.append(f"{YELLOW}⚠ WARNING: Front {front_min:.2f}m{RESET}")

        if warnings:
            print(f"{BOLD}ALERTS:{RESET}")
            for w in warnings:
                print(f"  {w}")
        else:
            print(f"{GREEN}{BOLD}✓ All clear{RESET}")

        print(f"\n{CYAN}Press Ctrl+C to exit{RESET}")

    def format_distance(self, dist, label):
        if math.isinf(dist) or dist > 5.0:
            return f"{GREEN}{label}: >5.0m{RESET}"
        elif dist < 0.35:
            return f"{RED}{BOLD}{label}: {dist:.2f}m{RESET}"
        elif dist < 0.50:
            return f"{YELLOW}{label}: {dist:.2f}m{RESET}"
        else:
            return f"{GREEN}{label}: {dist:.2f}m{RESET}"

    def print_zone(self, label, min_dist, distances):
        if not distances:
            print(f"  {label}: {RED}No data{RESET}")
            return

        avg = sum(distances) / len(distances)
        num = len(distances)

        if min_dist < 0.35:
            color, status = RED, "DANGER"
        elif min_dist < 0.50:
            color, status = YELLOW, "WARNING"
        else:
            color, status = GREEN, "CLEAR"

        print(f"  {label}: {color}min={min_dist:.2f}m, avg={avg:.2f}m, pts={num} [{status}]{RESET}")


def main(args=None):
    rclpy.init(args=args)
    viewer = LidarViewer()

    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        print(f"\n\n{CYAN}Stopped.{RESET}\n")
    finally:
        viewer.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
