#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 320)   # Reduced from 640 for less lag
        self.declare_parameter('frame_height', 240)  # Reduced from 480 for less lag
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_streaming', True)
        self.declare_parameter('jpeg_quality', 60)  # JPEG compression quality (0-100)
        
        # Get parameters
        camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.enable_streaming = self.get_parameter('enable_streaming').get_parameter_value().bool_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_id)
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Try to reduce buffer size to prevent memory issues
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Fix high contrast and overexposure issues
        # Disable auto exposure and set manual exposure
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 = manual mode
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -6)         # Lower exposure (-11 to -1, -6 is good)
        
        # Adjust brightness and contrast for better visibility
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)        # 0 = default brightness 
        self.cap.set(cv2.CAP_PROP_CONTRAST, 32)         # Lower contrast (0-100, 32 is moderate)
        
        # Disable auto white balance for consistent colors
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)           # Disable auto white balance
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4000) # Set neutral white balance
        
        # Control gain to prevent noise
        self.cap.set(cv2.CAP_PROP_GAIN, 0)              # Disable auto gain (0-100)
        
        # Set saturation for natural colors
        self.cap.set(cv2.CAP_PROP_SATURATION, 50)       # Moderate saturation (0-100)
        
        self.get_logger().info('Camera exposure and contrast optimized')
        
        # Check if camera opened successfully
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera - check camera connection')
            return
        
        # Test capture to verify camera is working
        ret, test_frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture test frame from camera')
            return
        else:
            self.get_logger().info(f'Camera test successful - captured {test_frame.shape}')
        
        # CV Bridge for ROS image messages
        self.bridge = CvBridge()
        
        # Publishers
        if self.enable_streaming:
            self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
            # Add compressed image publisher for better network performance
            self.compressed_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        self.aruco_pose_pub = self.create_publisher(Pose, '/aruco/pose', 10)
        
        # ArUco detector setup (compatible with different OpenCV versions)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # Try new API first, fallback to old API
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        except AttributeError:
            # Fallback to older OpenCV API
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_new_api = False
        
        # Camera calibration (rough estimates for 320x240 - scaled from 640x480)
        # You should calibrate your camera for better accuracy
        self.camera_matrix = np.array([
            [300.0, 0.0, 160.0],  # Scaled down for 320x240
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
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        
        # Detect ArUco markers
        self.detect_aruco_markers(frame)
        
        # Publish image for streaming if enabled
        if self.enable_streaming:
            try:
                # Publish raw image
                image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'camera_link'
                self.image_pub.publish(image_msg)
                
                # Publish compressed image for better performance
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = image_msg.header.stamp
                compressed_msg.header.frame_id = 'camera_link'
                compressed_msg.format = 'jpeg'
                
                # Encode as JPEG with specified quality (lower = more compression, less bandwidth)
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                _, compressed_data = cv2.imencode('.jpg', frame, encode_param)
                compressed_msg.data = compressed_data.tobytes()
                
                self.compressed_pub.publish(compressed_msg)
                
            except Exception as e:
                self.get_logger().warn(f'Failed to publish image: {e}')

    def detect_aruco_markers(self, frame):
        """Detect ArUco markers and estimate pose"""
        try:
            # Convert to grayscale for ArUco detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect markers (compatible with different OpenCV versions)
            if self.use_new_api:
                corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
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