#!/usr/bin/env python3
"""
Simple Camera Node - Direct V4L2 access for Ubuntu 22.04
Optimized for Raspberry Pi camera with minimal memory usage
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class SimpleCameraNode(Node):
    def __init__(self):
        super().__init__('simple_camera_node')
        
        # Parameters - conservative settings for stability
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('frame_width', 320)
        self.declare_parameter('frame_height', 240)
        self.declare_parameter('fps', 10)
        self.declare_parameter('enable_streaming', True)
        
        