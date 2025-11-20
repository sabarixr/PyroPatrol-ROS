#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 320)
        self.declare_parameter('frame_height', 240)
        self.declare_parameter('fps', 10)
        self.declare_parameter('enable_streaming', True)
        
        # Get parameters
        camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.enable_streaming = self.get_parameter('enable_streaming').get_parameter_value().bool_value
        
        # Initialize camera with V4L2 backend (more reliable on Ubuntu)
        self.cap = None
        backends_to_try = [
            (cv2.CAP_V4L2, "V4L2"),
            (cv2.CAP_ANY, "ANY")
        ]
        
        for backend, name in backends_to_try:
            self.get_logger().info(f'Trying camera backend: {name}')
            self.cap = cv2.VideoCapture(camera_id, backend)
            
            if self.cap.isOpened():
                self.get_logger().info(f'Successfully opened camera with {name} backend')
                break
            else:
                self.get_logger().warn(f'Failed to open camera with {name} backend')
                if self.cap:
                    self.cap.release()
                self.cap = None
        
        if not self.cap or not self.cap.isOpened():
            self.get_logger().error('Failed to open camera with any backend - check camera connection')
            return
            
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer to prevent memory issues
        
        # Test capture to ensure camera is working
        ret, test_frame = self.cap.read()
        if not ret or test_frame is None:
            self.get_logger().error('Failed to capture test frame from camera')
            return
        
        # CV Bridge for ROS image messages
        self.bridge = CvBridge()
        
        # Publishers
        if self.enable_streaming:
            self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.aruco_pose_pub = self.create_publisher(Pose, '/aruco/pose', 10)
        
        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Camera calibration (rough estimates for 320x240)
        # You should calibrate your camera for better accuracy
        self.camera_matrix = np.array([
            [300.0, 0.0, 160.0],
            [0.0, 300.0, 120.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([0.1, -0.2, 0.0, 0.0, 0.0])
        
        # ArUco marker size (in meters) - adjust based on your markers
        self.marker_size = 0.05  # 5cm markers
        
        # Timer for frame capture
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.capture_frame)
        
        self.get_logger().info(f'Camera node started - {self.frame_width}x{self.frame_height} @ {self.fps} FPS')

    def capture_frame(self):
        """Capture and process camera frame"""
        if not self.cap or not self.cap.isOpened():
            self.get_logger().error('Camera not available for capture')
            return
            
        ret, frame = self.cap.read()
        
        if not ret or frame is None:
            self.get_logger().warn('Failed to capture frame from camera')
            return
        
        if frame.size == 0:
            self.get_logger().warn('Captured empty frame')
            return
        
        # Detect ArUco markers
        self.detect_aruco_markers(frame)
        
        # Publish image for streaming if enabled
        if self.enable_streaming:
            try:
                image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'camera_link'
                self.image_pub.publish(image_msg)
            except Exception as e:
                self.get_logger().warn(f'Failed to publish image: {e}')

    def detect_aruco_markers(self, frame):
        """Detect ArUco markers and estimate pose"""
        try:
            # Convert to grayscale for ArUco detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect markers
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            
            if ids is not None and len(ids) > 0:
                # Estimate pose for each detected marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                
                # Publish pose of the first detected marker
                if len(tvecs) > 0:
                    pose_msg = Pose()
                    
                    # Translation (position)
                    pose_msg.position.x = float(tvecs[0][0][0])
                    pose_msg.position.y = float(tvecs[0][0][1])
                    pose_msg.position.z = float(tvecs[0][0][2])
                    
                    # Convert rotation vector to quaternion
                    rvec = rvecs[0][0]
                    rotation_matrix = cv2.Rodrigues(rvec)[0]
                    quat = self.rotation_matrix_to_quaternion(rotation_matrix)
                    
                    pose_msg.orientation.x = quat[0]
                    pose_msg.orientation.y = quat[1]
                    pose_msg.orientation.z = quat[2]
                    pose_msg.orientation.w = quat[3]
                    
                    self.aruco_pose_pub.publish(pose_msg)
                    
                    self.get_logger().debug(f'ArUco marker {ids[0][0]} detected at distance {tvecs[0][0][2]:.3f}m')
        
        except Exception as e:
            self.get_logger().warn(f'ArUco detection failed: {e}')

    def rotation_matrix_to_quaternion(self, rotation_matrix):
        """Convert rotation matrix to quaternion"""
        # Simple conversion - can be improved
        trace = np.trace(rotation_matrix)
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * s
            qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        else:
            if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                s = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2
                qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                qx = 0.25 * s
                qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                s = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2
                qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                qy = 0.25 * s
                qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            else:
                s = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2
                qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                qz = 0.25 * s
        
        return [qx, qy, qz, qw]

    def destroy_node(self):
        """Clean up camera resources"""
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_node = CameraNode()
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()