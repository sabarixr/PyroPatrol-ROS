#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from flask import Flask, Response
import numpy as np

class VideoStreamerNode(Node):
    def __init__(self):
        super().__init__('streamer_node')
        
        # Parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('jpeg_quality', 80)
        
        # Get parameters
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Current frame
        self.current_frame = None
        self.frame_lock = threading.Lock()
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        
        # Flask app setup
        self.app = Flask(__name__)
        self.setup_flask_routes()
        
        # Start Flask server in separate thread
        self.server_thread = threading.Thread(
            target=self.run_flask_server, daemon=True
        )
        self.server_thread.start()
        
        self.get_logger().info(f'Video streamer started on http://{self.host}:{self.port}')
        self.get_logger().info(f'Stream available at: http://{self.host}:{self.port}/stream.mjpg')

    def image_callback(self, msg):
        """Handle incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.frame_lock:
                self.current_frame = cv_image.copy()
                
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')

    def setup_flask_routes(self):
        """Setup Flask web server routes"""
        
        @self.app.route('/')
        def index():
            return '''
            <html>
            <head>
                <title>Fire Fighter Rover Camera Stream</title>
                <style>
                    body { 
                        font-family: Arial, sans-serif; 
                        margin: 40px; 
                        background-color: #f0f0f0;
                    }
                    .container { 
                        max-width: 800px; 
                        margin: 0 auto; 
                        background-color: white;
                        padding: 20px;
                        border-radius: 10px;
                        box-shadow: 0 2px 10px rgba(0,0,0,0.1);
                    }
                    h1 { 
                        color: #d32f2f; 
                        text-align: center;
                    }
                    img { 
                        display: block; 
                        margin: 20px auto; 
                        border: 2px solid #ddd;
                        border-radius: 5px;
                    }
                    .info {
                        text-align: center;
                        color: #666;
                        margin-top: 10px;
                    }
                </style>
            </head>
            <body>
                <div class="container">
                    <h1>🚒 Fire Fighter Rover Camera Stream</h1>
                    <img src="/stream.mjpg" alt="Camera Stream">
                    <div class="info">
                        Live camera feed from the rover<br>
                        Resolution: 320x240 | Format: MJPEG
                    </div>
                </div>
            </body>
            </html>
            '''
        
        @self.app.route('/stream.mjpg')
        def video_feed():
            return Response(
                self.generate_frames(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

    def generate_frames(self):
        """Generate MJPEG frames for streaming"""
        while True:
            with self.frame_lock:
                if self.current_frame is not None:
                    # Encode frame as JPEG
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                    ret, buffer = cv2.imencode('.jpg', self.current_frame, encode_param)
                    
                    if ret:
                        frame_bytes = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + 
                               frame_bytes + b'\r\n')
                else:
                    # Send a placeholder image if no frame available
                    placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
                    cv2.putText(placeholder, 'No Signal', (80, 120), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                    ret, buffer = cv2.imencode('.jpg', placeholder, encode_param)
                    
                    if ret:
                        frame_bytes = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + 
                               frame_bytes + b'\r\n')
            
            # Small delay to prevent excessive CPU usage
            import time
            time.sleep(0.033)  # ~30 FPS max

    def run_flask_server(self):
        """Run Flask server"""
        try:
            # Disable Flask's request logging to reduce CPU usage
            import logging
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.ERROR)
            
            self.app.run(host=self.host, port=self.port, debug=False, threaded=True)
        except Exception as e:
            self.get_logger().error(f'Flask server error: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        streamer_node = VideoStreamerNode()
        rclpy.spin(streamer_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
