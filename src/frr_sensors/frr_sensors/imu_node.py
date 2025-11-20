#!/usr/bin/env python3
# NOTE: MMA8452 accelerometer is removed as it's burned/not working
# This file is kept as a placeholder - use mpu6050_node.py instead

import rclpy
from rclpy.node import Node

class PlaceholderNode(Node):
    def __init__(self):
        super().__init__('imu_node_placeholder')
        self.get_logger().warn('MMA8452 sensor is not working. Use mpu6050_node instead.')
        self.get_logger().info('This node is deprecated - please use mpu6050_node')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PlaceholderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()