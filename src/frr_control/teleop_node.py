#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import threading

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('speed_increment', 0.1)
        
        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.speed_increment = self.get_parameter('speed_increment').get_parameter_value().double_value
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Current velocity
        self.twist = Twist()
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Key bindings
        self.key_bindings = {
            'w': (1, 0, 0, 0),     # Forward
            'x': (-1, 0, 0, 0),    # Backward
            'a': (0, 0, 0, 1),     # Turn left
            'd': (0, 0, 0, -1),    # Turn right
            'q': (1, 0, 0, 1),     # Forward + left
            'e': (1, 0, 0, -1),    # Forward + right
            'z': (-1, 0, 0, -1),   # Backward + left
            'c': (-1, 0, 0, 1),    # Backward + right
            's': (0, 0, 0, 0),     # Stop
            't': 'increase_linear',
            'g': 'decrease_linear',
            'r': 'increase_angular',
            'f': 'decrease_angular'
        }
        
        # Publishing timer (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # Keyboard input thread
        self.running = True
        self.input_thread = threading.Thread(target=self.get_key_input, daemon=True)
        self.input_thread.start()
        
        self.print_usage()
        self.get_logger().info('Teleop node started')

    def print_usage(self):
        """Print usage instructions"""
        msg = """
Fire Fighter Rover Teleoperation
---------------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity (turn left/right)
q/e : forward diagonal movement
z/c : backward diagonal movement
s   : stop

Speed control:
t/g : increase/decrease linear speed limit
r/f : increase/decrease angular speed limit

CTRL-C to quit
"""
        print(msg)

    def get_key(self):
        """Get a single keypress"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def get_key_input(self):
        """Keyboard input handler running in separate thread"""
        while self.running:
            try:
                key = self.get_key()
                if key:
                    self.process_key(key.lower())
            except Exception as e:
                self.get_logger().error(f'Key input error: {e}')
                break

    def process_key(self, key):
        """Process keyboard input"""
        if key in self.key_bindings:
            binding = self.key_bindings[key]
            
            if isinstance(binding, tuple):
                # Movement command
                linear_x, linear_y, linear_z, angular_z = binding
                self.twist.linear.x = linear_x * self.linear_speed
                self.twist.linear.y = linear_y * self.linear_speed
                self.twist.linear.z = linear_z * self.linear_speed
                self.twist.angular.z = angular_z * self.angular_speed
                
            elif binding == 'increase_linear':
                self.linear_speed += self.speed_increment
                self.get_logger().info(f'Linear speed: {self.linear_speed:.2f} m/s')
                
            elif binding == 'decrease_linear':
                self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
                self.get_logger().info(f'Linear speed: {self.linear_speed:.2f} m/s')
                
            elif binding == 'increase_angular':
                self.angular_speed += self.speed_increment
                self.get_logger().info(f'Angular speed: {self.angular_speed:.2f} rad/s')
                
            elif binding == 'decrease_angular':
                self.angular_speed = max(0.1, self.angular_speed - self.speed_increment)
                self.get_logger().info(f'Angular speed: {self.angular_speed:.2f} rad/s')
        
        elif key == '\x03':  # Ctrl-C
            self.running = False
            rclpy.shutdown()
            
        else:
            # Unknown key - stop
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.0
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = 0.0

    def publish_velocity(self):
        """Publish current velocity command"""
        if self.running:
            self.cmd_vel_pub.publish(self.twist)

    def destroy_node(self):
        """Clean up terminal settings"""
        self.running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_node = TeleopNode()
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        # Restore terminal settings
        if 'teleop_node' in locals():
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, teleop_node.settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
