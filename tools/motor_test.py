#!/usr/bin/env python3
"""
Motor Test Script - Test turning left and right independently
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def test_motors():
    rclpy.init()
    node = rclpy.create_node('motor_test')
    
    # Create publisher for motor commands
    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    print("🚒 Motor Turn Test")
    print("=" * 30)
    
    # Wait for publisher to be ready
    time.sleep(1)
    
    tests = [
        ("Stop", 0.0, 0.0),
        ("Turn Right (negative angular)", 0.0, -0.5),
        ("Stop", 0.0, 0.0), 
        ("Turn Left (positive angular)", 0.0, 0.5),
        ("Stop", 0.0, 0.0),
        ("Forward slow", 0.2, 0.0),
        ("Stop", 0.0, 0.0),
        ("Forward + Turn Right", 0.2, -0.5),
        ("Stop", 0.0, 0.0),
        ("Forward + Turn Left", 0.2, 0.5),
        ("Stop", 0.0, 0.0),
    ]
    
    for description, linear, angular in tests:
        print(f"Testing: {description} (linear={linear}, angular={angular})")
        
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        cmd_vel_pub.publish(twist)
        
        time.sleep(2)  # Run for 2 seconds
    
    # Final stop
    twist = Twist()
    cmd_vel_pub.publish(twist)
    print("Motor test complete!")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    test_motors()
