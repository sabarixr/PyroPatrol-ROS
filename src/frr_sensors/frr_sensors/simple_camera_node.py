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

        # Parameters
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_streaming', True)

        # Get parameters
        self.camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.enable_streaming = self.get_parameter('enable_streaming').get_parameter_value().bool_value

        # Check if camera exists
        if not os.path.exists(self.camera_device):
            self.get_logger().error(f'Camera device {self.camera_device} not found')
            return

        # Initialize camera
        self.cap = None
        self.initialize_camera()

        if not self.cap or not self.cap.isOpened():
            self.get_logger().error('Failed to initialize camera')
            return

        # CV Bridge
        self.bridge = CvBridge()

        # Publishers
        if self.enable_streaming:
            self.image_pub = self.create_publisher(Image, '/camera/image_raw', 1)
        self.aruco_pose_pub = self.create_publisher(Pose, '/aruco/pose', 1)

        # ArUco detector setup
        self.setup_aruco_detector()

        # Timer
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.capture_frame)

        self.frame_count = 0

        self.get_logger().info(f'Simple camera node started - {self.frame_width}x{self.frame_height} @ {self.fps} FPS')
        self.get_logger().info(f'Using device: {self.camera_device}')

    def initialize_camera(self):
        """Initialize camera with optimized settings for Ubuntu 22.04"""
        try:
            self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.get_logger().error('Failed to open camera device')
                return

            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(f'Camera initialized: {actual_width}x{actual_height} @ {actual_fps} FPS')

            ret, test_frame = self.cap.read()
            if not ret or test_frame is None:
                self.get_logger().error('Failed to capture test frame')
                return

            self.get_logger().info('Camera test capture successful')

        except Exception as e:
            self.get_logger().error(f'Camera initialization failed: {str(e)}')
            if self.cap:
                self.cap.release()
                self.cap = None

    def setup_aruco_detector(self):
        """Setup ArUco marker detector (compatible with old and new OpenCV versions)"""
        try:
            if hasattr(cv2.aruco, 'ArucoDetector'):
                # ✅ New API (OpenCV ≥ 4.7)
                self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                self.aruco_params = cv2.aruco.DetectorParameters()
                self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
                self.get_logger().info('Using new OpenCV ArUcoDetector API')
            else:
                # ✅ Old API (OpenCV ≤ 4.6)
                self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
                self.aruco_params = cv2.aruco.DetectorParameters_create()
                self.aruco_detector = None
                self.get_logger().info('Using legacy OpenCV detectMarkers API')

            # Rough camera calibration
            self.camera_matrix = np.array([
                [300.0, 0.0, 160.0],
                [0.0, 300.0, 120.0],
                [0.0, 0.0, 1.0]
            ])
            self.dist_coeffs = np.array([0.1, -0.2, 0.0, 0.0, 0.0])
            self.marker_size = 0.05  # 5 cm marker

        except Exception as e:
            self.get_logger().error(f'ArUco detector setup failed: {str(e)}')
            self.aruco_detector = None

    def capture_frame(self):
        """Capture and process camera frame"""
        if not self.cap or not self.cap.isOpened():
            return

        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn('Failed to capture frame')
                return

            # ⬇️ Flip the frame vertically (fix upside-down camera)
            frame = cv2.flip(frame, 0)

            if frame.size == 0:
                self.get_logger().warn('Captured empty frame')
                return

            self.frame_count += 1
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Captured {self.frame_count} frames')

            # Detect ArUco markers
            self.detect_aruco_markers(frame)

            # Publish image
            if self.enable_streaming:
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                    image_msg.header.stamp = self.get_clock().now().to_msg()
                    image_msg.header.frame_id = 'camera_link'
                    self.image_pub.publish(image_msg)
                except Exception as e:
                    self.get_logger().warn(f'Failed to publish image: {str(e)}')

        except Exception as e:
            self.get_logger().error(f'Frame capture failed: {str(e)}')

    def detect_aruco_markers(self, frame):
        """Detect ArUco markers and publish poses"""
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if self.aruco_detector:
                # New API
                corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            else:
                # Old API fallback
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None and len(ids) > 0:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )

                pose_msg = Pose()
                pose_msg.position.x = float(tvecs[0][0][0])
                pose_msg.position.y = float(tvecs[0][0][1])
                pose_msg.position.z = float(tvecs[0][0][2])
                pose_msg.orientation.w = 1.0

                self.aruco_pose_pub.publish(pose_msg)

                if self.frame_count % 50 == 0:
                    self.get_logger().info(f'ArUco marker detected: ID {ids[0][0]}')

        except Exception as e:
            self.get_logger().warn(f'ArUco detection failed: {str(e)}')

    def destroy_node(self):
        """Clean up resources"""
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SimpleCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

