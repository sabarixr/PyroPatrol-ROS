#!/usr/bin/env python3
"""
Ultrasonic Sensor Node (HC-SR04) - Raspberry Pi GPIO
Publishes front distance for obstacle detection when LIDAR unavailable
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import time

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("WARNING: RPi.GPIO not available. Ultrasonic sensor will not work.")


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        
        # Parameters
        self.declare_parameter('trigger_pin', 23)  # GPIO 23 (Pin 16)
        self.declare_parameter('echo_pin', 24)     # GPIO 24 (Pin 18)
        self.declare_parameter('publish_rate', 10.0)  # 10 Hz
        self.declare_parameter('max_distance', 4.0)   # 4 meters max range
        self.declare_parameter('min_distance', 0.02)  # 2 cm min range
        
        self.trigger_pin = self.get_parameter('trigger_pin').value
        self.echo_pin = self.get_parameter('echo_pin').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.max_distance = self.get_parameter('max_distance').value
        self.min_distance = self.get_parameter('min_distance').value
        
        # Publisher
        self.range_pub = self.create_publisher(Range, '/ultrasonic/front', 10)
        
        # Initialize GPIO
        self.gpio_initialized = False
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.trigger_pin, GPIO.OUT)
                GPIO.setup(self.echo_pin, GPIO.IN)
                GPIO.output(self.trigger_pin, False)
                time.sleep(0.1)  # Let sensor settle
                self.gpio_initialized = True
                self.get_logger().info(f'[OK] Ultrasonic sensor initialized: TRIG={self.trigger_pin}, ECHO={self.echo_pin}')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize GPIO: {e}')
        else:
            self.get_logger().warn('RPi.GPIO not available - sensor disabled')
        
        # Timer for publishing
        if self.gpio_initialized:
            self.timer = self.create_timer(1.0 / self.publish_rate, self.measure_and_publish)
        
    def measure_distance(self):
        """Measure distance using HC-SR04 ultrasonic sensor"""
        if not self.gpio_initialized:
            return None
        
        try:
            # Send 10us pulse to trigger
            GPIO.output(self.trigger_pin, True)
            time.sleep(0.00001)  # 10 microseconds
            GPIO.output(self.trigger_pin, False)
            
            # Initialize timing variables
            pulse_start = time.time()
            pulse_end = time.time()
            
            # Wait for echo start (with timeout)
            timeout = time.time() + 0.1  # 100ms timeout
            while GPIO.input(self.echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return None
            
            # Wait for echo end (with timeout - extended for longer range)
            timeout = time.time() + 0.025  # 25ms timeout (enough for 4m range)
            while GPIO.input(self.echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return None
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Speed of sound = 34300 cm/s, divide by 2
            distance = distance / 100.0  # Convert to meters
            
            # Validate range
            if self.min_distance <= distance <= self.max_distance:
                return distance
            else:
                return None
                
        except Exception as e:
            self.get_logger().warn(f'Measurement error: {e}')
            return None
    
    def measure_and_publish(self):
        """Take measurement and publish Range message"""
        distance = self.measure_distance()
        
        # Create Range message
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_front'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 degrees in radians
        msg.min_range = float(self.min_distance)
        msg.max_range = float(self.max_distance)
        
        if distance is not None:
            msg.range = float(distance)
        else:
            msg.range = float('inf')  # Invalid reading
        
        self.range_pub.publish(msg)
        
        # Log occasionally (every 2 seconds)
        if not hasattr(self, '_last_log_time'):
            self._last_log_time = 0
        
        now = time.time()
        if now - self._last_log_time > 2.0:
            if distance is not None:
                self.get_logger().info(f'Front distance: {distance:.2f}m')
            else:
                self.get_logger().debug('No valid reading')
            self._last_log_time = now
    
    def destroy_node(self):
        """Cleanup GPIO on shutdown"""
        if self.gpio_initialized:
            try:
                GPIO.cleanup()
                self.get_logger().info('GPIO cleaned up')
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
