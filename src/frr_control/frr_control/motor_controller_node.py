#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64, Float32MultiArray
import math

# Try to import pigpio first, fallback to RPi.GPIO
try:
    import pigpio
    USE_PIGPIO = True
except ImportError:
    import RPi.GPIO as GPIO
    USE_PIGPIO = False

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # Motor parameters
        self.declare_parameter('wheel_base', 0.2)  # Distance between wheels (m)
        self.declare_parameter('max_speed', 1.0)   # Maximum linear speed (m/s)
        self.declare_parameter('max_angular_speed', 2.0)  # Maximum angular speed (rad/s)
        
        # Servo parameters
        self.declare_parameter('servo_min_angle', -180.0)  # Minimum servo angle (degrees)
        self.declare_parameter('servo_max_angle', 180.0)   # Maximum servo angle (degrees)
        self.declare_parameter('servo_center_angle', 0.0) # Center servo angle (degrees)
        
        # Get parameters
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        
        # Get servo parameters
        self.servo_min_angle = self.get_parameter('servo_min_angle').get_parameter_value().double_value
        self.servo_max_angle = self.get_parameter('servo_max_angle').get_parameter_value().double_value
        self.servo_center_angle = self.get_parameter('servo_center_angle').get_parameter_value().double_value
        
        # Current servo angle
        self.current_servo_angle = self.servo_center_angle
        
        # Motor PWM scaling - full 0-100% range, NO smoothing
        self.MIN_PWM_PERCENT = 0
        self.MAX_PWM_PERCENT = 100
        
        # GPIO pin definitions (L298N connections) - USING HARDWARE PWM PINS
        self.MOTOR_A_IN1 = 27  # Left motor direction 1
        self.MOTOR_A_IN2 = 22  # Left motor direction 2
        self.MOTOR_B_IN1 = 23  # Right motor direction 1
        self.MOTOR_B_IN2 = 24  # Right motor direction 2
        self.MOTOR_A_ENA = 12  # Left motor PWM (speed) - HARDWARE PWM!
        self.MOTOR_B_ENB = 13  # Right motor PWM (speed) - HARDWARE PWM!
        
        # Servo pin definition (Camera tilt)
        self.SERVO_PIN = 25    # Camera tilt servo
        
        # Initialize GPIO
        self.setup_gpio()
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.servo_sub = self.create_subscription(
            Float64, '/camera_tilt', self.servo_callback, 10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/motor_status', 10)
        
        # Status timer (5 Hz)
        self.status_timer = self.create_timer(0.2, self.publish_status)
        
        # Safety timer - stop motors if no command received
        self.safety_timeout = 1.0  # seconds
        self.last_cmd_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        # Current motor speeds for status
        self.left_speed = 0.0
        self.right_speed = 0.0
        
        self.get_logger().info('Motor driver node started')

    def setup_gpio(self):
        """Initialize GPIO pins for motor control"""
        try:
            if USE_PIGPIO:
                # Use pigpio for better GPIO access
                self.pi = pigpio.pi()
                if not self.pi.connected:
                    raise Exception("Failed to connect to pigpiod daemon")
                
                # Set up direction pins as outputs
                self.pi.set_mode(self.MOTOR_A_IN1, pigpio.OUTPUT)
                self.pi.set_mode(self.MOTOR_A_IN2, pigpio.OUTPUT)
                self.pi.set_mode(self.MOTOR_B_IN1, pigpio.OUTPUT)
                self.pi.set_mode(self.MOTOR_B_IN2, pigpio.OUTPUT)
                
                # Set up PWM pins
                self.pi.set_mode(self.MOTOR_A_ENA, pigpio.OUTPUT)
                self.pi.set_mode(self.MOTOR_B_ENB, pigpio.OUTPUT)
                
                # Initialize PWM frequency for motors (1000 Hz)
                self.pi.set_PWM_frequency(self.MOTOR_A_ENA, 1000)
                self.pi.set_PWM_frequency(self.MOTOR_B_ENB, 1000)
                
                # Set PWM range (0-255 for smoother control)
                self.pi.set_PWM_range(self.MOTOR_A_ENA, 255)
                self.pi.set_PWM_range(self.MOTOR_B_ENB, 255)
                
                # Set up servo pin
                self.pi.set_mode(self.SERVO_PIN, pigpio.OUTPUT)
                
                # Initialize all motors to stopped state
                self.stop_motors()
                
                # Initialize servo to center position
                self.set_servo_angle(self.servo_center_angle)
                
                self.get_logger().info('GPIO initialized successfully using pigpio')
                
            else:
                # Fallback to RPi.GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                
                # Set up direction pins as outputs
                GPIO.setup(self.MOTOR_A_IN1, GPIO.OUT)
                GPIO.setup(self.MOTOR_A_IN2, GPIO.OUT)
                GPIO.setup(self.MOTOR_B_IN1, GPIO.OUT)
                GPIO.setup(self.MOTOR_B_IN2, GPIO.OUT)
                
                # Set up PWM pins
                GPIO.setup(self.MOTOR_A_ENA, GPIO.OUT)
                GPIO.setup(self.MOTOR_B_ENB, GPIO.OUT)
                
                # Set up servo pin
                GPIO.setup(self.SERVO_PIN, GPIO.OUT)
                
                # Create PWM instances (1000 Hz frequency for motors, 50 Hz for servo)
                self.pwm_left = GPIO.PWM(self.MOTOR_A_ENA, 1000)
                self.pwm_right = GPIO.PWM(self.MOTOR_B_ENB, 1000)
                self.servo_pwm = GPIO.PWM(self.SERVO_PIN, 50)  # 50 Hz for servo
                
                # Start PWM with 0% duty cycle for motors
                self.pwm_left.start(0)
                self.pwm_right.start(0)
                self.servo_pwm.start(0)
                
                # Initialize all motors to stopped state
                self.stop_motors()
                
                # Initialize servo to center position
                self.set_servo_angle(self.servo_center_angle)
                
                self.get_logger().info('GPIO initialized successfully using RPi.GPIO')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO: {e}')
            self.get_logger().info('Try running: sudo ./fix_permissions.sh')

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        self.last_cmd_time = self.get_clock().now()
        
        # Extract linear and angular velocities
        linear_vel = msg.linear.x  # Forward/backward
        angular_vel = msg.angular.z  # Rotation
        
        # Limit velocities
        linear_vel = max(-self.max_speed, min(self.max_speed, linear_vel))
        angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
        
        # Convert to differential drive
        left_vel, right_vel = self.differential_drive(linear_vel, angular_vel)
        
        # Set motor speeds
        self.set_motor_speeds(left_vel, right_vel)
        
        self.left_speed = left_vel
        self.right_speed = right_vel

    def servo_callback(self, msg):
        """Handle incoming servo angle commands"""
        angle = msg.data
        self.set_servo_angle(angle)

    def set_servo_angle(self, angle):
        """Set servo angle in degrees"""
        # Clamp angle to allowed range
        angle = max(self.servo_min_angle, min(self.servo_max_angle, angle))
        self.current_servo_angle = angle
        
        if USE_PIGPIO and hasattr(self, 'pi'):
            # Convert angle to pulse width (500-2500 microseconds for -90 to +90 degrees)
            # Standard servo: 1500us = center, 1000us = -90°, 2000us = +90°
            pulse_width = 1500 + (angle / 90.0) * 500
            pulse_width = max(1000, min(2000, pulse_width))  # Clamp to safe range
            
            self.pi.set_servo_pulsewidth(self.SERVO_PIN, pulse_width)
            
        else:
            # RPi.GPIO method - convert angle to duty cycle
            # For 50Hz PWM: 1ms = 5% duty cycle, 1.5ms = 7.5%, 2ms = 10%
            duty_cycle = 7.5 + (angle / 90.0) * 2.5  # 5% to 10% duty cycle range
            duty_cycle = max(5.0, min(10.0, duty_cycle))  # Clamp to safe range
            
            if hasattr(self, 'servo_pwm'):
                self.servo_pwm.ChangeDutyCycle(duty_cycle)
        
        self.get_logger().info(f'Servo angle set to: {angle:.1f}°')

    def differential_drive(self, linear_vel, angular_vel):
        """Convert linear and angular velocity to left/right wheel velocities"""
        # Simple differential drive - no power limiting
        left_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)
        
        return left_vel, right_vel

    def set_motor_speeds(self, left_vel, right_vel):
        """Set motor speeds and directions - DIRECT, NO SMOOTHING"""
        if USE_PIGPIO:
            # Direct PWM scaling: 0.0-max_speed maps to 0-255
            def scale_pwm(vel):
                if abs(vel) < 0.01:  # Dead zone
                    return 0
                # Direct mapping - full power
                vel_percent = min(1.0, abs(vel) / self.max_speed)
                return int(vel_percent * 255)  # 0-255 for pigpio
            
            left_pwm = scale_pwm(left_vel)
            right_pwm = scale_pwm(right_vel)
            
            # Set left motor direction
            if left_vel > 0.01:
                self.pi.write(self.MOTOR_A_IN1, 0)
                self.pi.write(self.MOTOR_A_IN2, 1)
            elif left_vel < -0.01:
                self.pi.write(self.MOTOR_A_IN1, 1)
                self.pi.write(self.MOTOR_A_IN2, 0)
            else:
                self.pi.write(self.MOTOR_A_IN1, 0)
                self.pi.write(self.MOTOR_A_IN2, 0)
                left_pwm = 0
            
            # Set right motor direction
            if right_vel > 0.01:
                self.pi.write(self.MOTOR_B_IN1, 0)
                self.pi.write(self.MOTOR_B_IN2, 1)
            elif right_vel < -0.01:
                self.pi.write(self.MOTOR_B_IN1, 1)
                self.pi.write(self.MOTOR_B_IN2, 0)
            else:
                self.pi.write(self.MOTOR_B_IN1, 0)
                self.pi.write(self.MOTOR_B_IN2, 0)
                right_pwm = 0
            
            # Set PWM speeds - INSTANT, no smoothing
            self.pi.set_PWM_dutycycle(self.MOTOR_A_ENA, left_pwm)
            self.pi.set_PWM_dutycycle(self.MOTOR_B_ENB, right_pwm)
            
        else:
            # RPi.GPIO fallback - direct 0-100%
            def scale_pwm_gpio(vel):
                if abs(vel) < 0.01:
                    return 0
                vel_percent = min(1.0, abs(vel) / self.max_speed)
                return vel_percent * 100.0
            
            left_pwm = scale_pwm_gpio(left_vel)
            right_pwm = scale_pwm_gpio(right_vel)
            
            # Set left motor
            if left_vel > 0.01:
                GPIO.output(self.MOTOR_A_IN1, GPIO.LOW)
                GPIO.output(self.MOTOR_A_IN2, GPIO.HIGH)
            elif left_vel < -0.01:
                GPIO.output(self.MOTOR_A_IN1, GPIO.HIGH)
                GPIO.output(self.MOTOR_A_IN2, GPIO.LOW)
            else:
                GPIO.output(self.MOTOR_A_IN1, GPIO.LOW)
                GPIO.output(self.MOTOR_A_IN2, GPIO.LOW)
                left_pwm = 0
            
            # Set right motor
            if right_vel > 0.01:
                GPIO.output(self.MOTOR_B_IN1, GPIO.LOW)
                GPIO.output(self.MOTOR_B_IN2, GPIO.HIGH)
            elif right_vel < -0.01:
                GPIO.output(self.MOTOR_B_IN1, GPIO.HIGH)
                GPIO.output(self.MOTOR_B_IN2, GPIO.LOW)
            else:
                GPIO.output(self.MOTOR_B_IN1, GPIO.LOW)
                GPIO.output(self.MOTOR_B_IN2, GPIO.LOW)
                right_pwm = 0
            
            # Set PWM duty cycles
            if hasattr(self, 'pwm_left'):
                self.pwm_left.ChangeDutyCycle(left_pwm)
            if hasattr(self, 'pwm_right'):
                self.pwm_right.ChangeDutyCycle(right_pwm)

    def stop_motors(self):
        """Stop all motors"""
        if USE_PIGPIO and hasattr(self, 'pi'):
            self.pi.write(self.MOTOR_A_IN1, 0)
            self.pi.write(self.MOTOR_A_IN2, 0)
            self.pi.write(self.MOTOR_B_IN1, 0)
            self.pi.write(self.MOTOR_B_IN2, 0)
            self.pi.set_PWM_dutycycle(self.MOTOR_A_ENA, 0)
            self.pi.set_PWM_dutycycle(self.MOTOR_B_ENB, 0)
        else:
            GPIO.output(self.MOTOR_A_IN1, GPIO.LOW)
            GPIO.output(self.MOTOR_A_IN2, GPIO.LOW)
            GPIO.output(self.MOTOR_B_IN1, GPIO.LOW)
            GPIO.output(self.MOTOR_B_IN2, GPIO.LOW)
            
            if hasattr(self, 'pwm_left'):
                self.pwm_left.ChangeDutyCycle(0)
            if hasattr(self, 'pwm_right'):
                self.pwm_right.ChangeDutyCycle(0)
        
        self.left_speed = 0.0
        self.right_speed = 0.0

    def safety_check(self):
        """Safety check - stop motors if no command received recently"""
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        if time_diff > self.safety_timeout:
            if self.left_speed != 0.0 or self.right_speed != 0.0:
                self.get_logger().warn('Safety timeout - stopping motors')
                self.stop_motors()

    def publish_status(self):
        """Publish motor status"""
        status_msg = String()
        status_msg.data = f'Left: {self.left_speed:.2f} m/s, Right: {self.right_speed:.2f} m/s, Servo: {self.current_servo_angle:.1f}°'
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean up GPIO resources"""
        self.get_logger().info('Cleaning up GPIO')
        self.stop_motors()
        
        if USE_PIGPIO and hasattr(self, 'pi'):
            self.pi.stop()
        else:
            if hasattr(self, 'pwm_left'):
                self.pwm_left.stop()
            if hasattr(self, 'pwm_right'):
                self.pwm_right.stop()
            if hasattr(self, 'servo_pwm'):
                self.servo_pwm.stop()
            GPIO.cleanup()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        motor_node = MotorDriverNode()
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()