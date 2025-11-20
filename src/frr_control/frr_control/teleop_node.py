#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
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
        self.declare_parameter('servo_angle_increment', 5.0)
        
        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.speed_increment = self.get_parameter('speed_increment').get_parameter_value().double_value
        self.servo_angle_increment = self.get_parameter('servo_angle_increment').get_parameter_value().double_value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.camera_tilt_pub = self.create_publisher(Float64, '/camera_tilt', 10)
        
        # Current camera servo angle
        self.camera_angle = 0.0
        
        # Maximum speed values for percentage calculations
        self.max_speed = 1.0  # m/s
        self.max_angular_speed = 2.0  # rad/s
        
        # Current velocity
        self.twist = Twist()
        
        # Terminal settings - may not be available when launched via ros2 launch
        self.tty = True
        try:
            self.settings = termios.tcgetattr(sys.stdin)
        except Exception as e:
            # No controlling terminal (e.g. launched under a launch file). Disable keyboard input.
            self.get_logger().warn(f'No TTY available for teleop (keyboard disabled): {e}')
            self.tty = False
        
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
            'f': 'decrease_angular',
            'i': 'camera_up',      # Tilt camera up
            'k': 'camera_down',    # Tilt camera down
            '1': 'set_speed_10',   # Set speed to 10%
            '2': 'set_speed_20',   # Set speed to 20%
            '3': 'set_speed_30',   # Set speed to 30%
            '4': 'set_speed_40',   # Set speed to 40%
            '5': 'set_speed_50',   # Set speed to 50%
            '6': 'set_speed_60',   # Set speed to 60%
            '7': 'set_speed_70',   # Set speed to 70%
            '8': 'set_speed_80',   # Set speed to 80%
            '9': 'set_speed_90',   # Set speed to 90%
            '0': 'set_speed_100'   # Set speed to 100%
        }
        
        # Publishing timer (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_velocity)

        # Keyboard input thread (only if a TTY is available)
        self.running = True
        if self.tty:
            self.input_thread = threading.Thread(target=self.get_key_input, daemon=True)
            self.input_thread.start()
            self.print_usage()
        else:
            # Running headless under launch - inform user to use topics to control
            self.get_logger().info('Teleop started in headless mode: keyboard input disabled. Publish /cmd_vel to control the robot.')
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

w/x : forward/backward
a/d : turn left/right
q/e : forward diagonal movement
z/c : backward diagonal movement
s   : stop

Speed control:
t/g : increase/decrease linear speed limit
r/f : increase/decrease angular speed limit
0-9 : set speed to 10%-100% (1=10%, 2=20%, ..., 0=100%)

Camera control:
i/k : tilt camera up/down

CTRL-C to quit
"""
        print(msg)

    def get_key(self):
        """Get a single keypress"""
        # Only called when TTY is available
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
                
            elif binding == 'camera_up':
                self.camera_angle = min(90.0, self.camera_angle + self.servo_angle_increment)
                self.publish_camera_angle()
                self.get_logger().info(f'Camera angle: {self.camera_angle:.1f}°')
                
            elif binding == 'camera_down':
                self.camera_angle = max(-90.0, self.camera_angle - self.servo_angle_increment)
                self.publish_camera_angle()
                self.get_logger().info(f'Camera angle: {self.camera_angle:.1f}°')
                
            # Dynamic speed presets (1-0 keys for 10%-100%)
            elif binding.startswith('set_speed_'):
                speed_percent = int(binding.split('_')[-1])
                self.linear_speed = (speed_percent / 100.0) * 1.0  # Max 1.0 m/s
                self.angular_speed = (speed_percent / 100.0) * 2.0  # Max 2.0 rad/s
                self.get_logger().info(f'Speed set to {speed_percent}%: linear={self.linear_speed:.2f} m/s, angular={self.angular_speed:.2f} rad/s')
        
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
    
    def publish_camera_angle(self):
        """Publish camera servo angle"""
        angle_msg = Float64()
        angle_msg.data = self.camera_angle
        self.camera_tilt_pub.publish(angle_msg)

    def destroy_node(self):
        """Clean up terminal settings"""
        self.running = False
        if self.tty and hasattr(self, 'settings'):
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    teleop_node = None
    try:
        teleop_node = TeleopNode()
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        # Restore terminal settings only if TTY was used
        if teleop_node is not None and getattr(teleop_node, 'tty', False) and hasattr(teleop_node, 'settings'):
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, teleop_node.settings)
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
