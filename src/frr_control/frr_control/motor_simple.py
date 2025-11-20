#!/usr/bin/env python3
"""ULTRA SIMPLE MOTOR TEST"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class SimpleMotor(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Left motor (A)
        GPIO.setup(17, GPIO.OUT)  # IN1
        GPIO.setup(27, GPIO.OUT)  # IN2
        GPIO.setup(12, GPIO.OUT)  # PWM
        
        # Right motor (B)
        GPIO.setup(22, GPIO.OUT)  # IN1
        GPIO.setup(23, GPIO.OUT)  # IN2
        GPIO.setup(13, GPIO.OUT)  # PWM
        
        # PWM objects
        self.left = GPIO.PWM(12, 1000)
        self.right = GPIO.PWM(13, 1000)
        self.left.start(0)
        self.right.start(0)
        
        # Subscribe
        self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.get_logger().info('SIMPLE MOTOR READY - Send PWM 0-100 in linear.x')
    
    def cb(self, msg):
        pwm = abs(msg.linear.x)
        fwd = msg.linear.x > 0
        
        self.get_logger().info(f'PWM={pwm:.0f}% FWD={fwd}')
        
        # LEFT
        GPIO.output(17, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(27, GPIO.LOW if fwd else GPIO.HIGH)
        self.left.ChangeDutyCycle(pwm)
        
        # RIGHT  
        GPIO.output(22, GPIO.HIGH if fwd else GPIO.LOW)
        GPIO.output(23, GPIO.LOW if fwd else GPIO.HIGH)
        self.right.ChangeDutyCycle(pwm)
        
        self.get_logger().info(f'BOTH MOTORS: {"FWD" if fwd else "REV"} @ {pwm:.0f}%')

def main():
    rclpy.init()
    node = SimpleMotor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
