#!/usr/bin/env python3
"""
ESP32 Teleop Node - Enhanced control for ESP32-based rover
Includes fire scanning and water pump control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
import sys
import select
import termios
import tty

# Movement bindings
move_bindings = {
    'w': (1, 0, 0, 0),     # Forward
    's': (-1, 0, 0, 0),    # Backward  
    'a': (0, 0, 0, 1),     # Turn left
    'd': (0, 0, 0, -1),    # Turn right
    'q': (1, 0, 0, 1),     # Forward + Turn left
    'e': (1, 0, 0, -1),    # Forward + Turn right
    'z': (-1, 0, 0, 1),    # Backward + Turn left
    'c': (-1, 0, 0, -1),   # Backward + Turn right
}

# Camera tilt bindings
camera_bindings = {
    'i': 5,     # Tilt up
    'k': -5,    # Tilt down
    'I': 15,    # Tilt up fast
    'K': -15,   # Tilt down fast
    'o': 45,    # Look up
    'l': -45,   # Look down  
    'u': 0,     # Center camera
}

# Speed change bindings
speed_bindings = {
    'r': (1.05, 1.05),    # Increase both speeds (5%)
    'f': (0.95, 0.95),    # Decrease both speeds (5%) 
    'R': (1.2, 1.2),      # Increase both speeds fast
    'F': (0.8, 0.8),      # Decrease both speeds fast
    't': (1.05, 1.0),     # Increase only linear speed
    'g': (0.95, 1.0),     # Decrease only linear speed
    'T': (1.2, 1.0),      # Increase linear speed fast
    'G': (0.8, 1.0),      # Decrease linear speed fast
    'y': (1.0, 1.05),     # Increase only angular speed
    'h': (1.0, 0.95),     # Decrease only angular speed
    'Y': (1.0, 1.2),      # Increase angular speed fast
    'H': (1.0, 0.8),      # Decrease angular speed fast
}

msg = """
🚒 ESP32 Fire Fighter Rover Teleop Control 🚒
=============================================

Movement Controls:
   w         Forward
   s         Backward  
a  s  d      Turn Left / Backward / Turn Right
   x         Stop

Combined Movement:
q  w  e      Forward+Left / Forward / Forward+Right
z  x  c      Backward+Left / Stop / Backward+Right

Camera Tilt Controls:
   i/k       Tilt up/down (±5°)
   I/K       Tilt up/down fast (±15°) [Shift+i/k]
   o/l       Look up/down (±45°)
   u         Center camera (0°)

Fire Fighting:
   m         Toggle fire scanning mode
   p         Toggle water pump

Speed Controls (Real-time):
r/f : increase/decrease speeds (±5%)
R/F : increase/decrease speeds fast (±20%)
t/g : linear speed only (±5%)
y/h : angular speed only (±5%)

Presets:
1 : Slow mode    (linear=0.2, angular=0.5)
2 : Normal mode  (linear=0.4, angular=0.8) 
3 : Fast mode    (linear=0.6, angular=1.2)

Emergency: SPACE to stop everything
Exit: Ctrl+C or ESC

Current: linear=%.2f, angular=%.2f, camera=%.1f°, scan=%s, pump=%s
"""

class ESP32TeleopNode(Node):
    def __init__(self):
        super().__init__('esp32_teleop_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.camera_tilt_pub = self.create_publisher(Float64, '/camera_tilt', 1)
        self.fire_scan_pub = self.create_publisher(Bool, '/fire_scan', 1)
        self.pump_pub = self.create_publisher(Bool, '/water_pump', 1)
        
        # Movement parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.6
        self.camera_angle = 0.0
        
        # Fire fighting state
        self.scanning_enabled = False
        self.pump_enabled = False
        
        # Speed limits
        self.max_linear = 0.8
        self.max_angular = 1.5
        self.min_speed = 0.05
        
        # Camera limits  
        self.max_camera_angle = 90.0
        self.min_camera_angle = -90.0
        
        self.get_logger().info('ESP32 Teleop node started')
        self.print_usage()

    def print_usage(self):
        scan_status = "ON" if self.scanning_enabled else "OFF"
        pump_status = "ON" if self.pump_enabled else "OFF"
        print(msg % (self.linear_speed, self.angular_speed, self.camera_angle, scan_status, pump_status))

    def get_key(self):
        """Get a single keypress from stdin"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def constrain_speeds(self):
        """Keep speeds within reasonable limits"""
        self.linear_speed = max(self.min_speed, min(self.max_linear, self.linear_speed))
        self.angular_speed = max(self.min_speed, min(self.max_angular, self.angular_speed))

    def constrain_camera(self):
        """Keep camera angle within limits"""
        self.camera_angle = max(self.min_camera_angle, min(self.max_camera_angle, self.camera_angle))

    def publish_twist(self, x, y, z, th):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = x * self.linear_speed
        twist.linear.y = y * self.linear_speed  
        twist.linear.z = z * self.linear_speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th * self.angular_speed
        
        self.cmd_vel_pub.publish(twist)

    def publish_camera_tilt(self, angle_change=None, absolute_angle=None):
        """Publish camera tilt command"""
        if absolute_angle is not None:
            self.camera_angle = absolute_angle
        elif angle_change is not None:
            self.camera_angle += angle_change
        
        self.constrain_camera()
        
        camera_msg = Float64()
        camera_msg.data = float(self.camera_angle)
        self.camera_tilt_pub.publish(camera_msg)
        
        print(f'📹 Camera tilt: {self.camera_angle:.1f}°')

    def toggle_fire_scan(self):
        """Toggle fire scanning mode"""
        self.scanning_enabled = not self.scanning_enabled
        scan_msg = Bool()
        scan_msg.data = self.scanning_enabled
        self.fire_scan_pub.publish(scan_msg)
        
        status = "ENABLED" if self.scanning_enabled else "DISABLED"
        print(f'🔥 Fire scanning {status}')

    def toggle_pump(self):
        """Toggle water pump"""
        self.pump_enabled = not self.pump_enabled
        pump_msg = Bool()
        pump_msg.data = self.pump_enabled
        self.pump_pub.publish(pump_msg)
        
        status = "ON" if self.pump_enabled else "OFF"
        print(f'💦 Water pump {status}')

    def stop_robot(self):
        """Stop all movement"""
        self.publish_twist(0, 0, 0, 0)

    def set_speed_preset(self, preset):
        """Set predefined speed combinations"""
        if preset == '1':  # Slow
            self.linear_speed = 0.2
            self.angular_speed = 0.5
        elif preset == '2':  # Normal
            self.linear_speed = 0.4 
            self.angular_speed = 0.8
        elif preset == '3':  # Fast
            self.linear_speed = 0.6
            self.angular_speed = 1.2
        
        self.constrain_speeds()
        print(f'⚡ Speed preset {preset}: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}')

    def run(self):
        """Main teleop loop"""
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while True:
                key = self.get_key()
                
                # Handle movement commands
                if key in move_bindings.keys():
                    x, y, z, th = move_bindings[key]
                    self.publish_twist(x, y, z, th)
                    
                    if x != 0 or th != 0:
                        linear_actual = x * self.linear_speed
                        angular_actual = th * self.angular_speed
                        print(f'🚗 Move: linear={linear_actual:.2f}, angular={angular_actual:.2f}')
                
                # Handle camera tilt commands
                elif key in camera_bindings.keys():
                    angle_change = camera_bindings[key]
                    if abs(angle_change) > 20:
                        self.publish_camera_tilt(absolute_angle=angle_change)
                    else:
                        self.publish_camera_tilt(angle_change=angle_change)
                
                # Handle speed changes
                elif key in speed_bindings.keys():
                    linear_mult, angular_mult = speed_bindings[key]
                    old_linear = self.linear_speed
                    old_angular = self.angular_speed
                    
                    self.linear_speed *= linear_mult
                    self.angular_speed *= angular_mult
                    self.constrain_speeds()
                    
                    print(f'⚡ Speed: {old_linear:.2f}→{self.linear_speed:.2f} (linear), {old_angular:.2f}→{self.angular_speed:.2f} (angular)')
                
                # Handle speed presets
                elif key in ['1', '2', '3']:
                    self.set_speed_preset(key)
                
                # Fire fighting controls
                elif key == 'm':
                    self.toggle_fire_scan()
                elif key == 'p':
                    self.toggle_pump()
                
                # Emergency stop
                elif key == ' ':
                    self.stop_robot()
                    print('🛑 EMERGENCY STOP!')
                
                # Regular stop
                elif key == 'x':
                    self.stop_robot()
                    print('⏹️  Robot stopped')
                
                # Quit
                elif key == '\x1b':  # ESC key
                    print('👋 ESC pressed - exiting')
                    break
                elif key == '\x03':  # Ctrl+C
                    break
                
                # Help
                elif key == '?':
                    self.print_usage()
                
                # Invalid key
                else:
                    if ord(key) < 32:  # Control character
                        continue
                    
        except Exception as e:
            self.get_logger().error(f'Teleop error: {e}')
            
        finally:
            # Stop robot and restore terminal
            self.stop_robot()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ESP32TeleopNode()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()