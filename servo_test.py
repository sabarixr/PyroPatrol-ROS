#!/usr/bin/env python3
"""
Simple servo test script for camera tilt
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

def test_servo():
    rclpy.init()
    node = rclpy.create_node('servo_test')
    
    # Create publisher for camera tilt
    camera_tilt_pub = node.create_publisher(Float64, '/camera_tilt', 10)
    
    print("🚒 Camera Servo Test")
    print("=" * 30)
    
    # Wait for publisher to be ready
    time.sleep(1)
    
    # Test sequence
    angles = [0, 45, -45, 90, -90, 0]
    
    for angle in angles:
        print(f"Setting servo to {angle}°")
        msg = Float64()
        msg.data = float(angle)
        camera_tilt_pub.publish(msg)
        time.sleep(2)
    
    print("Servo test complete!")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    test_servo()
