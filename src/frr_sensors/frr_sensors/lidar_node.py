#!/usr/bin/env python3
"""Simple LiDAR Node - Fixed version"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import serial
import struct
import math

FRAME_LEN = 22

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.raw_pub = self.create_publisher(Float32MultiArray, '/lidar/raw', 10)
        
        try:
            self.serial = serial.Serial(port=port, baudrate=baud, timeout=0.5)
            self.serial.reset_input_buffer()
            self.get_logger().info(f'✅ LiDAR connected on {port}')
        except Exception as e:
            self.get_logger().error(f'❌ Serial error: {e}')
            raise
        
        self.angles = []
        self.ranges = []
        self.last_angle = None
        
        self.timer = self.create_timer(0.01, self.read_lidar)
        self.get_logger().info('LiDAR node started')

    def read_lidar(self):
        try:
            data = self.serial.read(FRAME_LEN)
            if len(data) != FRAME_LEN or data[0] != 0xAA or data[1] != 0x55:
                return
            
            angle_start = struct.unpack_from("<H", data, 2)[0] / 100.0
            distances = [struct.unpack_from("<H", data, i)[0] / 1000.0 for i in range(4, 22, 2)]
            
            # Detect 360° wraparound OR if we have enough points
            if self.last_angle and angle_start < self.last_angle - 180 and len(self.angles) > 20:
                self.publish_scan()
            elif len(self.angles) > 80:  # Publish if buffer gets too large (safety)
                self.publish_scan()
            
            self.last_angle = angle_start
            angle_inc = 40.0 / 9.0
            
            for i, dist in enumerate(distances):
                angle = (angle_start + i * angle_inc) % 360.0
                self.angles.append(math.radians(angle))
                self.ranges.append(dist)
            
            raw_msg = Float32MultiArray()
            raw_msg.data = [angle_start] + distances
            self.raw_pub.publish(raw_msg)
        except:
            pass

    def publish_scan(self):
        if not self.angles:
            return
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = min(self.angles)
        scan.angle_max = max(self.angles)
        scan.angle_increment = (scan.angle_max - scan.angle_min) / max(1, len(self.angles) - 1)
        scan.scan_time = 0.1
        scan.range_min = 0.15
        scan.range_max = 12.0
        scan.ranges = self.ranges
        
        self.scan_pub.publish(scan)
        self.angles.clear()
        self.ranges.clear()

    def destroy_node(self):
        if hasattr(self, 'serial'):
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
