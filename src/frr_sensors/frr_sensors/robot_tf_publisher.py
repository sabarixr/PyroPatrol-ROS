#!/usr/bin/env python3
"""
Static TF Publisher for Fire-Fighting Robot
Publishes static transforms between robot frames
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math


class RobotTFPublisher(Node):
    def __init__(self):
        super().__init__('robot_tf_publisher')
        
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish all static transforms
        self.publish_static_transforms()
        
        self.get_logger().info('Robot TF publisher initialized')
    
    def publish_static_transforms(self):
        """Publish all static transforms for the robot"""
        
        transforms = []
        
        # base_link -> base_footprint (robot center on ground)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = -0.05  # Robot height above ground
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # base_link -> laser (LIDAR position)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        t.transform.translation.x = 0.0    # Center of robot
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05   # 5cm above base_link
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # base_link -> imu_link (MPU6050 IMU position)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.03   # 3cm above base_link
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # base_link -> camera_link (Camera position)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = 0.08   # 8cm forward
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.10   # 10cm above base_link
        # Camera points forward
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # base_link -> left_wheel
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'left_wheel'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.10   # Half of wheel base (20cm / 2)
        t.transform.translation.z = -0.0325  # Wheel radius (6.5cm / 2)
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # base_link -> right_wheel
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'right_wheel'
        t.transform.translation.x = 0.0
        t.transform.translation.y = -0.10  # Half of wheel base (20cm / 2)
        t.transform.translation.z = -0.0325  # Wheel radius (6.5cm / 2)
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # Broadcast all transforms
        self.tf_broadcaster.sendTransform(transforms)
        
        self.get_logger().info('Published static transforms:')
        self.get_logger().info('  base_link -> base_footprint')
        self.get_logger().info('  base_link -> laser')
        self.get_logger().info('  base_link -> imu_link')
        self.get_logger().info('  base_link -> camera_link')
        self.get_logger().info('  base_link -> left_wheel')
        self.get_logger().info('  base_link -> right_wheel')


def main(args=None):
    rclpy.init(args=args)
    node = RobotTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
