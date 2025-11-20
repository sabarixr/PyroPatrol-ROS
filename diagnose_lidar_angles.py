#!/usr/bin/env python3
"""
LiDAR Angle Diagnostic Tool
Shows what angles the LiDAR is actually reporting
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarAngleDiagnostic(Node):
    def __init__(self):
        super().__init__('lidar_angle_diagnostic')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        print("\n=== LiDAR Angle Diagnostic ===")
        print("Listening to /scan topic...\n")
        self.received = False
        
    def scan_callback(self, msg):
        if self.received:
            return
        self.received = True
        
        print(f"LiDAR Configuration:")
        print(f"  angle_min: {math.degrees(msg.angle_min):.2f}°")
        print(f"  angle_max: {math.degrees(msg.angle_max):.2f}°")
        print(f"  angle_increment: {math.degrees(msg.angle_increment):.2f}°")
        print(f"  range_min: {msg.range_min:.2f}m")
        print(f"  range_max: {msg.range_max:.2f}m")
        print(f"  Total points: {len(msg.ranges)}")
        print(f"\nAngle distribution:")
        
        ranges = list(msg.ranges)
        
        # Count points in different angle ranges
        angles_0_90 = 0
        angles_90_180 = 0
        angles_neg90_0 = 0
        angles_neg180_neg90 = 0
        
        valid_ranges = []
        
        for i, distance in enumerate(ranges):
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)
            
            # Normalize to -180 to 180
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360
            
            if not (math.isinf(distance) or math.isnan(distance) or distance < 0.05):
                valid_ranges.append((angle_deg, distance))
            
            if 0 <= angle_deg < 90:
                angles_0_90 += 1
            elif 90 <= angle_deg <= 180:
                angles_90_180 += 1
            elif -90 <= angle_deg < 0:
                angles_neg90_0 += 1
            elif -180 <= angle_deg < -90:
                angles_neg180_neg90 += 1
        
        print(f"  0° to 90° (front-left): {angles_0_90} points")
        print(f"  90° to 180° (back-left): {angles_90_180} points")
        print(f"  -90° to 0° (front-right): {angles_neg90_0} points")
        print(f"  -180° to -90° (back-right): {angles_neg180_neg90} points")
        
        if valid_ranges:
            print(f"\nValid readings with distances:")
            # Show some sample points
            samples = [valid_ranges[0], valid_ranges[len(valid_ranges)//4], 
                      valid_ranges[len(valid_ranges)//2], valid_ranges[3*len(valid_ranges)//4],
                      valid_ranges[-1]]
            for angle, dist in samples:
                print(f"  {angle:7.1f}° → {dist:5.2f}m")
        
        print("\n=== Done ===\n")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    diagnostic = LidarAngleDiagnostic()
    
    try:
        rclpy.spin(diagnostic)
    except KeyboardInterrupt:
        pass
    finally:
        diagnostic.destroy_node()

if __name__ == '__main__':
    main()
