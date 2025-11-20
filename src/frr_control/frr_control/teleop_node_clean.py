#!/usr/bin/env python3
"""
Clean Teleoperation Node - Simplified and organized output
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
import sys
import select
import termios
import tty
import os

# Clear screen command
def clear_screen():
    os.system('clear' if os.name == 'posix' else 'cls')

# Movement bindings
move_bindings = {
    'w': (1, 0, 0, 0),     # Forward
    's': (-1, 0, 0, 0),    # Backward  
    'a': (0, 0, 0, 1),     # Turn left
    'd': (0, 0, 0, -1),    # Turn right
    'q': (1, 0, 0, 1),     # Forward + left
    'e': (1, 0, 0, -1),    # Forward + right
    'z': (-1, 0, 0, 1),    # Backward + left
    'c': (-1, 0, 0, -1),   # Backward + right
}

# Camera tilt bindings
camera_bindings = {
    'i': 5,     # Tilt up
    'k': -5,    # Tilt down
    'I': 15,    # Tilt up fast
    'K': -15,   # Tilt down fast
    'o': 45,    # Look up
    'l': -45,   # Look down  
    'u': 0,     # Center
}

# Speed bindings
speed_bindings = {
    'r': (1.1, 1.1),    # Increase both
    'f': (0.9, 0.9),    # Decrease both 
    't': (1.1, 1.0),    # Increase linear
    'g': (0.9, 1.0),    # Decrease linear
    'y': (1.0, 1.1),    # Increase angular
    'h': (1.0, 0.9),    # Decrease angular
}

class CleanTeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Check if we're using obstacle avoidance
        self.declare_parameter('with_avoidance', False)
        self.with_avoidance = self.get_parameter('with_avoidance').value
        
        # Publishers
        if self.with_avoidance:
            # Publish to /cmd_vel_teleop when using obstacle avoidance
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 1)
            self.get_logger().info('Publishing to /cmd_vel_teleop (with obstacle avoidance)')
        else:
            # Direct control - publish to /cmd_vel
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
            self.get_logger().info('Publishing to /cmd_vel (direct control)')
        
        self.camera_tilt_pub = self.create_publisher(Float64, '/camera_tilt', 1)
        
        # Subscribe to obstacle detection if using avoidance
        if self.with_avoidance:
            self.obstacle_sub = self.create_subscription(
                Bool, '/obstacle_detected', self.obstacle_callback, 10
            )
            self.obstacle_detected = False
        
        # Speed parameters
        self.linear_speed = 3.0     # Initial speed: 3.0 m/s (minimum to move motors)
        self.angular_speed = 1.0    # Initial angular speed: 1.0 rad/s
        self.camera_angle = 0.0
        
        # Limits
        # Speed limits
        self.max_linear = 20.0   # Very high to allow full PWM range
        self.max_angular = 10.0  # High rotation speed
        self.min_speed = 0.05
        self.max_camera_angle = 90.0
        self.min_camera_angle = -90.0
        
        # Status display
        self.last_command = "Ready"
        self.show_help = True
        
        # Current movement command
        self.current_x = 0.0
        self.current_th = 0.0
        
        # Publish timer - continuous publishing prevents safety timeout
        self.publish_timer = self.create_timer(0.1, self.publish_current_velocity)
        
        # Current velocity command (for continuous publishing)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_th = 0.0
        
        # Timer to continuously publish velocity (prevents safety timeout)
        self.publish_timer = self.create_timer(0.1, self.publish_current_velocity)  # 10Hz

    def publish_current_velocity(self):
        """Continuously publish current velocity command"""
        twist = Twist()
        twist.linear.x = self.current_x * self.linear_speed
        twist.linear.y = self.current_y * self.linear_speed
        twist.linear.z = self.current_z * self.linear_speed
        twist.angular.z = self.current_th * self.angular_speed
        self.cmd_vel_pub.publish(twist)

    def obstacle_callback(self, msg):
        """Update obstacle status"""
        self.obstacle_detected = msg.data

    def print_status(self):
        """Print clean status display"""
        clear_screen()
        print("=" * 70)
        print(" FIRE FIGHTER ROVER - TELEOPERATION")
        print("=" * 70)
        
        if self.with_avoidance:
            status = "BLOCKED" if self.obstacle_detected else "CLEAR"
            status_symbol = "[!]" if self.obstacle_detected else "[✓]"
            print(f" Mode: SAFE (Obstacle Avoidance ON)  {status_symbol} Path: {status}")
        else:
            print(" Mode: DIRECT (No Obstacle Avoidance)  [⚠]  Use with caution!")
        
        print("-" * 70)
        print(f" Linear Speed:  {self.linear_speed:.2f} m/s  (max: {self.max_linear})")
        print(f" Angular Speed: {self.angular_speed:.2f} rad/s (max: {self.max_angular})")
        print(f" Camera Angle:  {self.camera_angle:.1f}°")
        print("-" * 70)
        print(f" Last Command: {self.last_command}")
        print("=" * 70)
        
        if self.show_help:
            print("\nCONTROLS:")
            print("  Movement:  W/S=fwd/back  A/D=left/right  Q/E/Z/C=diagonals  X=stop")
            print("  Camera:    I/K=tilt up/down  O/L=look up/down  U=center")
            print("  Speed:     R/F=all  T/G=linear  Y/H=angular  1/2/3=presets")
            print("  Special:   SPACE=emergency stop  ?=toggle help  ESC/Ctrl+C=quit")
            print("\nPress ? to hide/show help")
        else:
            print("\n(Press ? to show controls)")

    def get_key(self):
        """Get single keypress"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def constrain_speeds(self):
        """Keep speeds within limits"""
        self.linear_speed = max(self.min_speed, min(self.max_linear, self.linear_speed))
        self.angular_speed = max(self.min_speed, min(self.max_angular, self.angular_speed))

    def constrain_camera(self):
        """Keep camera within limits"""
        self.camera_angle = max(self.min_camera_angle, min(self.max_camera_angle, self.camera_angle))

    def publish_twist(self, x, y, z, th):
        """Publish velocity command"""
        # Store current command for continuous publishing
        self.current_x = x
        self.current_th = th
        
        twist = Twist()
        twist.linear.x = x * self.linear_speed
        twist.linear.y = y * self.linear_speed  
        twist.linear.z = z * self.linear_speed
        twist.angular.z = th * self.angular_speed
        self.cmd_vel_pub.publish(twist)
    
    def publish_current_velocity(self):
        """Continuously publish current velocity to prevent safety timeout"""
        twist = Twist()
        twist.linear.x = self.current_x * self.linear_speed
        twist.angular.z = self.current_th * self.angular_speed
        self.cmd_vel_pub.publish(twist)

    def publish_camera_tilt(self, angle_change=None, absolute_angle=None):
        """Publish camera tilt"""
        if absolute_angle is not None:
            self.camera_angle = absolute_angle
        elif angle_change is not None:
            self.camera_angle += angle_change
        
        self.constrain_camera()
        camera_msg = Float64()
        camera_msg.data = float(self.camera_angle)
        self.camera_tilt_pub.publish(camera_msg)

    def stop_robot(self):
        """Stop all movement"""
        self.publish_twist(0, 0, 0, 0)

    def set_speed_preset(self, preset):
        """Set speed preset"""
        presets = {
            '1': (0.2, 0.5, "Slow"),
            '2': (0.4, 0.8, "Normal"),
            '3': (0.6, 1.2, "Fast")
        }
        if preset in presets:
            self.linear_speed, self.angular_speed, name = presets[preset]
            self.constrain_speeds()
            self.last_command = f"Preset: {name}"

    def run(self):
        """Main loop"""
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            self.print_status()
            
            while True:
                key = self.get_key()
                
                # Movement
                if key in move_bindings:
                    x, y, z, th = move_bindings[key]
                    self.current_x = x
                    self.current_th = th
                    self.publish_twist(x, y, z, th)
                    
                    direction = []
                    if x > 0: direction.append("Forward")
                    elif x < 0: direction.append("Backward")
                    if th > 0: direction.append("Left")
                    elif th < 0: direction.append("Right")
                    self.last_command = " + ".join(direction) if direction else "Moving"
                    self.print_status()
                
                # Camera
                elif key in camera_bindings:
                    angle = camera_bindings[key]
                    if abs(angle) > 20:
                        self.publish_camera_tilt(absolute_angle=angle)
                        self.last_command = f"Camera: {self.camera_angle:.0f}°"
                    else:
                        self.publish_camera_tilt(angle_change=angle)
                        self.last_command = f"Camera tilt {'+' if angle > 0 else ''}{angle}°"
                    self.print_status()
                
                # Speed adjustment
                elif key in speed_bindings:
                    linear_mult, angular_mult = speed_bindings[key]
                    self.linear_speed *= linear_mult
                    self.angular_speed *= angular_mult
                    self.constrain_speeds()
                    self.last_command = "Speed adjusted"
                    self.print_status()
                
                # Presets
                elif key in ['1', '2', '3']:
                    self.set_speed_preset(key)
                    self.print_status()
                
                # Stop
                elif key == ' ':
                    self.current_x = 0.0
                    self.current_th = 0.0
                    self.stop_robot()
                    self.last_command = "EMERGENCY STOP"
                    self.print_status()
                
                elif key == 'x':
                    self.current_x = 0.0
                    self.current_th = 0.0
                    self.stop_robot()
                    self.last_command = "Stopped"
                    self.print_status()
                
                # Help toggle
                elif key == '?':
                    self.show_help = not self.show_help
                    self.last_command = "Help " + ("shown" if self.show_help else "hidden")
                    self.print_status()
                
                # Quit
                elif key == '\x1b' or key == '\x03':  # ESC or Ctrl+C
                    break
                    
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            
        finally:
            self.stop_robot()
            clear_screen()
            print("Teleop stopped. Robot is stationary.")
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_node = CleanTeleopNode()
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
