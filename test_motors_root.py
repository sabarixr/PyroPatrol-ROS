#!/usr/bin/env python3
"""
Root Motor Diagnostics - Test each motor forward/backward individually
Uses the same GPIO pins as motor_controller_node.py (ROS driver)

This script helps identify:
- GPIO connection issues
- Motor wiring problems
- Direction/polarity issues
- Power supply problems
- L298N enable pin issues
"""

import sys
import time

# Try pigpio first, fallback to RPi.GPIO
USE_PIGPIO = False
try:
    import pigpio
    USE_PIGPIO = True
    print("✓ Using pigpio library (preferred)")
except ImportError:
    try:
        import RPi.GPIO as GPIO
        print("✓ Using RPi.GPIO library (fallback)")
    except ImportError:
        print("❌ ERROR: Neither pigpio nor RPi.GPIO found!")
        print("Install with: sudo apt install python3-pigpio python3-rpi.gpio")
        sys.exit(1)

# GPIO pin definitions - UPDATED TO USE HARDWARE PWM
MOTOR_A_IN1 = 27  # Left motor direction 1 (BCM 27, Pin 13)
MOTOR_A_IN2 = 22  # Left motor direction 2 (BCM 22, Pin 15)
MOTOR_B_IN1 = 23  # Right motor direction 1 (BCM 23, Pin 16)
MOTOR_B_IN2 = 24  # Right motor direction 2 (BCM 24, Pin 18)
MOTOR_A_ENA = 12  # Left motor PWM (speed) - CHANGED TO GPIO 12 (HARDWARE PWM!)
MOTOR_B_ENB = 13  # Right motor PWM (speed) - CHANGED TO GPIO 13 (HARDWARE PWM!)

# Test parameters
PWM_FREQUENCY = 1000  # Hz - matches motor_controller_node
TEST_SPEED_LOW = 30   # % duty cycle for low speed test
TEST_SPEED_MED = 50   # % duty cycle for medium speed test
TEST_SPEED_HIGH = 70  # % duty cycle for high speed test
TEST_DURATION = 3     # seconds per test

class MotorTester:
    def __init__(self):
        self.pi = None
        self.pwm_left = None
        self.pwm_right = None
        self.setup_gpio()
    
    def setup_gpio(self):
        """Initialize GPIO - same method as motor_controller_node"""
        print("\n" + "="*70)
        print("  GPIO INITIALIZATION")
        print("="*70)
        
        if USE_PIGPIO:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                print("❌ Failed to connect to pigpiod daemon!")
                print("Start it with: sudo pigpiod")
                sys.exit(1)
            
            print("✓ Connected to pigpiod daemon")
            
            # Set up direction pins
            self.pi.set_mode(MOTOR_A_IN1, pigpio.OUTPUT)
            self.pi.set_mode(MOTOR_A_IN2, pigpio.OUTPUT)
            self.pi.set_mode(MOTOR_B_IN1, pigpio.OUTPUT)
            self.pi.set_mode(MOTOR_B_IN2, pigpio.OUTPUT)
            
            # Set up PWM pins
            self.pi.set_mode(MOTOR_A_ENA, pigpio.OUTPUT)
            self.pi.set_mode(MOTOR_B_ENB, pigpio.OUTPUT)
            
            # *** FIX: Initialize PWM frequency and range for both motors ***
            print("  Setting up PWM channels...")
            self.pi.set_PWM_frequency(MOTOR_A_ENA, PWM_FREQUENCY)
            self.pi.set_PWM_frequency(MOTOR_B_ENB, PWM_FREQUENCY)
            print(f"    Left motor PWM:  GPIO{MOTOR_A_ENA} @ {PWM_FREQUENCY} Hz")
            print(f"    Right motor PWM: GPIO{MOTOR_B_ENB} @ {PWM_FREQUENCY} Hz")
            
            # Set PWM range (0-255 for smoother control)
            self.pi.set_PWM_range(MOTOR_A_ENA, 255)
            self.pi.set_PWM_range(MOTOR_B_ENB, 255)
            print(f"    PWM range: 0-255")
            
            # Initialize to stopped state
            self.stop_all_motors()
            
            print(f"✓ GPIO pins configured:")
            print(f"  Left Motor:  IN1=GPIO{MOTOR_A_IN1}, IN2=GPIO{MOTOR_A_IN2}, ENA=GPIO{MOTOR_A_ENA}")
            print(f"  Right Motor: IN1=GPIO{MOTOR_B_IN1}, IN2=GPIO{MOTOR_B_IN2}, ENB=GPIO{MOTOR_B_ENB}")
            
        else:
            # RPi.GPIO fallback
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Set up direction pins
            GPIO.setup(MOTOR_A_IN1, GPIO.OUT)
            GPIO.setup(MOTOR_A_IN2, GPIO.OUT)
            GPIO.setup(MOTOR_B_IN1, GPIO.OUT)
            GPIO.setup(MOTOR_B_IN2, GPIO.OUT)
            
            # Set up PWM pins
            GPIO.setup(MOTOR_A_ENA, GPIO.OUT)
            GPIO.setup(MOTOR_B_ENB, GPIO.OUT)
            
            # Create PWM instances
            self.pwm_left = GPIO.PWM(MOTOR_A_ENA, PWM_FREQUENCY)
            self.pwm_right = GPIO.PWM(MOTOR_B_ENB, PWM_FREQUENCY)
            
            # Start PWM at 0%
            self.pwm_left.start(0)
            self.pwm_right.start(0)
            
            # Initialize to stopped state
            self.stop_all_motors()
            
            print(f"✓ GPIO pins configured (RPi.GPIO):")
            print(f"  Left Motor:  IN1=GPIO{MOTOR_A_IN1}, IN2=GPIO{MOTOR_A_IN2}, ENA=GPIO{MOTOR_A_ENA}")
            print(f"  Right Motor: IN1=GPIO{MOTOR_B_IN1}, IN2=GPIO{MOTOR_B_IN2}, ENB=GPIO{MOTOR_B_ENB}")
    
    def stop_all_motors(self):
        """Stop both motors completely"""
        if USE_PIGPIO:
            self.pi.write(MOTOR_A_IN1, 0)
            self.pi.write(MOTOR_A_IN2, 0)
            self.pi.write(MOTOR_B_IN1, 0)
            self.pi.write(MOTOR_B_IN2, 0)
            self.pi.set_PWM_dutycycle(MOTOR_A_ENA, 0)
            self.pi.set_PWM_dutycycle(MOTOR_B_ENB, 0)
        else:
            GPIO.output(MOTOR_A_IN1, GPIO.LOW)
            GPIO.output(MOTOR_A_IN2, GPIO.LOW)
            GPIO.output(MOTOR_B_IN1, GPIO.LOW)
            GPIO.output(MOTOR_B_IN2, GPIO.LOW)
            if self.pwm_left:
                self.pwm_left.ChangeDutyCycle(0)
            if self.pwm_right:
                self.pwm_right.ChangeDutyCycle(0)
    
    def set_left_motor(self, speed_percent, forward=True):
        """
        Set left motor speed and direction
        speed_percent: 0-100
        forward: True for forward, False for backward
        """
        if USE_PIGPIO:
            # First set direction
            if forward:
                # Forward: IN2=1, IN1=0 (matches motor_controller_node)
                self.pi.write(MOTOR_A_IN1, 0)
                self.pi.write(MOTOR_A_IN2, 1)
            else:
                # Backward: IN1=1, IN2=0
                self.pi.write(MOTOR_A_IN1, 1)
                self.pi.write(MOTOR_A_IN2, 0)
            
            # Small delay for L298N to settle
            time.sleep(0.001)
            
            # Then set PWM (pigpio uses 0-255 duty cycle)
            duty = int((speed_percent / 100.0) * 255)
            self.pi.set_PWM_dutycycle(MOTOR_A_ENA, duty)
            
            # Debug output
            print(f"    [DEBUG] Left motor: duty={duty}/255 ({speed_percent}%), dir={'FWD' if forward else 'BWD'}")
        else:
            # RPi.GPIO uses 0-100% duty cycle
            if forward:
                GPIO.output(MOTOR_A_IN1, GPIO.LOW)
                GPIO.output(MOTOR_A_IN2, GPIO.HIGH)
            else:
                GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
                GPIO.output(MOTOR_A_IN2, GPIO.LOW)
            
            if self.pwm_left:
                self.pwm_left.ChangeDutyCycle(speed_percent)
    
    def set_right_motor(self, speed_percent, forward=True):
        """
        Set right motor speed and direction
        speed_percent: 0-100
        forward: True for forward, False for backward
        """
        if USE_PIGPIO:
            # First set direction
            if forward:
                # Forward: IN2=1, IN1=0 (matches motor_controller_node)
                self.pi.write(MOTOR_B_IN1, 0)
                self.pi.write(MOTOR_B_IN2, 1)
            else:
                # Backward: IN1=1, IN2=0
                self.pi.write(MOTOR_B_IN1, 1)
                self.pi.write(MOTOR_B_IN2, 0)
            
            # Small delay for L298N to settle
            time.sleep(0.001)
            
            # Then set PWM (pigpio uses 0-255 duty cycle)
            duty = int((speed_percent / 100.0) * 255)
            self.pi.set_PWM_dutycycle(MOTOR_B_ENB, duty)
            
            # Debug output
            print(f"    [DEBUG] Right motor: duty={duty}/255 ({speed_percent}%), dir={'FWD' if forward else 'BWD'}")
        else:
            # RPi.GPIO uses 0-100% duty cycle
            if forward:
                GPIO.output(MOTOR_B_IN1, GPIO.LOW)
                GPIO.output(MOTOR_B_IN2, GPIO.HIGH)
            else:
                GPIO.output(MOTOR_B_IN1, GPIO.HIGH)
                GPIO.output(MOTOR_B_IN2, GPIO.LOW)
            
            if self.pwm_right:
                self.pwm_right.ChangeDutyCycle(speed_percent)
    
    def test_left_motor_forward(self, speed=TEST_SPEED_MED):
        """Test left motor forward"""
        print(f"\n🔧 LEFT MOTOR - FORWARD ({speed}% PWM)")
        print(f"   GPIO: IN1={MOTOR_A_IN1}→0, IN2={MOTOR_A_IN2}→1, ENA={MOTOR_A_ENA}→{speed}%")
        print(f"   Expected: Left wheel should spin FORWARD")
        print(f"   Duration: {TEST_DURATION} seconds")
        
        self.stop_all_motors()
        time.sleep(0.5)
        self.set_left_motor(speed, forward=True)
        
        for i in range(TEST_DURATION):
            print(f"   ... {TEST_DURATION - i} seconds remaining", end='\r')
            time.sleep(1)
        
        self.stop_all_motors()
        print("   ✓ Test complete" + " "*30)
        time.sleep(1)
    
    def test_left_motor_backward(self, speed=TEST_SPEED_MED):
        """Test left motor backward"""
        print(f"\n🔧 LEFT MOTOR - BACKWARD ({speed}% PWM)")
        print(f"   GPIO: IN1={MOTOR_A_IN1}→1, IN2={MOTOR_A_IN2}→0, ENA={MOTOR_A_ENA}→{speed}%")
        print(f"   Expected: Left wheel should spin BACKWARD")
        print(f"   Duration: {TEST_DURATION} seconds")
        
        self.stop_all_motors()
        time.sleep(0.5)
        self.set_left_motor(speed, forward=False)
        
        for i in range(TEST_DURATION):
            print(f"   ... {TEST_DURATION - i} seconds remaining", end='\r')
            time.sleep(1)
        
        self.stop_all_motors()
        print("   ✓ Test complete" + " "*30)
        time.sleep(1)
    
    def test_right_motor_forward(self, speed=TEST_SPEED_MED):
        """Test right motor forward"""
        print(f"\n🔧 RIGHT MOTOR - FORWARD ({speed}% PWM)")
        print(f"   GPIO: IN1={MOTOR_B_IN1}→0, IN2={MOTOR_B_IN2}→1, ENB={MOTOR_B_ENB}→{speed}%")
        print(f"   Expected: Right wheel should spin FORWARD")
        print(f"   Duration: {TEST_DURATION} seconds")
        
        self.stop_all_motors()
        time.sleep(0.5)
        self.set_right_motor(speed, forward=True)
        
        for i in range(TEST_DURATION):
            print(f"   ... {TEST_DURATION - i} seconds remaining", end='\r')
            time.sleep(1)
        
        self.stop_all_motors()
        print("   ✓ Test complete" + " "*30)
        time.sleep(1)
    
    def test_right_motor_backward(self, speed=TEST_SPEED_MED):
        """Test right motor backward"""
        print(f"\n🔧 RIGHT MOTOR - BACKWARD ({speed}% PWM)")
        print(f"   GPIO: IN1={MOTOR_B_IN1}→1, IN2={MOTOR_B_IN2}→0, ENB={MOTOR_B_ENB}→{speed}%")
        print(f"   Expected: Right wheel should spin BACKWARD")
        print(f"   Duration: {TEST_DURATION} seconds")
        
        self.stop_all_motors()
        time.sleep(0.5)
        self.set_right_motor(speed, forward=False)
        
        for i in range(TEST_DURATION):
            print(f"   ... {TEST_DURATION - i} seconds remaining", end='\r')
            time.sleep(1)
        
        self.stop_all_motors()
        print("   ✓ Test complete" + " "*30)
        time.sleep(1)
    
    def test_both_forward(self, speed=TEST_SPEED_MED):
        """Test both motors forward simultaneously"""
        print(f"\n🔧 BOTH MOTORS - FORWARD ({speed}% PWM)")
        print(f"   Expected: Robot should move FORWARD")
        print(f"   Duration: {TEST_DURATION} seconds")
        
        self.stop_all_motors()
        time.sleep(0.5)
        self.set_left_motor(speed, forward=True)
        self.set_right_motor(speed, forward=True)
        
        for i in range(TEST_DURATION):
            print(f"   ... {TEST_DURATION - i} seconds remaining", end='\r')
            time.sleep(1)
        
        self.stop_all_motors()
        print("   ✓ Test complete" + " "*30)
        time.sleep(1)
    
    def test_both_backward(self, speed=TEST_SPEED_MED):
        """Test both motors backward simultaneously"""
        print(f"\n🔧 BOTH MOTORS - BACKWARD ({speed}% PWM)")
        print(f"   Expected: Robot should move BACKWARD")
        print(f"   Duration: {TEST_DURATION} seconds")
        
        self.stop_all_motors()
        time.sleep(0.5)
        self.set_left_motor(speed, forward=False)
        self.set_right_motor(speed, forward=False)
        
        for i in range(TEST_DURATION):
            print(f"   ... {TEST_DURATION - i} seconds remaining", end='\r')
            time.sleep(1)
        
        self.stop_all_motors()
        print("   ✓ Test complete" + " "*30)
        time.sleep(1)
    
    def test_both_motors_detailed(self):
        """Test both motors at various speeds to diagnose power/timing issues"""
        print("\n" + "="*70)
        print("  DETAILED BOTH-MOTORS TEST")
        print("="*70)
        print("\nThis will test both motors together at different speeds.")
        print("Watch carefully which motor spins during each test.\n")
        
        speeds = [30, 50, 70]
        
        for speed in speeds:
            print(f"\n🔧 BOTH MOTORS FORWARD @ {speed}% PWM")
            print(f"   Left:  IN1={MOTOR_A_IN1}→0, IN2={MOTOR_A_IN2}→1, ENA={MOTOR_A_ENA}→{speed}%")
            print(f"   Right: IN1={MOTOR_B_IN1}→0, IN2={MOTOR_B_IN2}→1, ENB={MOTOR_B_ENB}→{speed}%")
            print(f"   Expected: BOTH wheels should spin forward")
            print(f"   Watch: Which wheel(s) are actually spinning?")
            
            self.stop_all_motors()
            time.sleep(0.5)
            self.set_left_motor(speed, forward=True)
            self.set_right_motor(speed, forward=True)
            
            for i in range(TEST_DURATION):
                print(f"   ... {TEST_DURATION - i} seconds remaining", end='\r')
                time.sleep(1)
            
            self.stop_all_motors()
            print("   ✓ Test complete" + " "*30)
            time.sleep(1)
        
        for speed in speeds:
            print(f"\n🔧 BOTH MOTORS BACKWARD @ {speed}% PWM")
            print(f"   Left:  IN1={MOTOR_A_IN1}→1, IN2={MOTOR_A_IN2}→0, ENA={MOTOR_A_ENA}→{speed}%")
            print(f"   Right: IN1={MOTOR_B_IN1}→1, IN2={MOTOR_B_IN2}→0, ENB={MOTOR_B_ENB}→{speed}%")
            print(f"   Expected: BOTH wheels should spin backward")
            print(f"   Watch: Which wheel(s) are actually spinning?")
            
            self.stop_all_motors()
            time.sleep(0.5)
            self.set_left_motor(speed, forward=False)
            self.set_right_motor(speed, forward=False)
            
            for i in range(TEST_DURATION):
                print(f"   ... {TEST_DURATION - i} seconds remaining", end='\r')
                time.sleep(1)
            
            self.stop_all_motors()
            print("   ✓ Test complete" + " "*30)
            time.sleep(1)
    
    def run_full_diagnostic(self):
        """Run complete diagnostic test sequence"""
        print("\n" + "="*70)
        print("  ROOT MOTOR DIAGNOSTIC TEST - FOCUS ON BOTH MOTORS")
        print("="*70)
        print("\n⚠️  SAFETY WARNING:")
        print("  - Ensure robot is ELEVATED (wheels off ground)")
        print("  - Check power supply is connected (7-12V to L298N)")
        print("  - Watch for overheating or smoke - STOP IMMEDIATELY if seen")
        print("\n📋 Test sequence:")
        print("  Testing BOTH motors together at 30%, 50%, 70% speeds")
        print("  Watch which motor(s) spin during each test!")
        print("\n" + "="*70)
        
        response = input("\nPress ENTER to start, or Ctrl+C to cancel: ")
        
        try:
            # Focus on both motors test
            self.test_both_motors_detailed()
            
            # Final stop
            print("\n" + "="*70)
            print("  TESTS COMPLETE - ANALYSIS")
            print("="*70)
            
            print("\n❓ WHAT DID YOU SEE?")
            print("─"*70)
            print("\nIf ONLY LEFT motor spun during forward:")
            print("  → Right motor PWM not being applied")
            print("  → Could be pigpio PWM initialization issue")
            print("  → Could be GPIO 18 (right PWM) conflict")
            print("")
            print("If ONLY RIGHT motor spun during forward:")
            print("  → Left motor PWM not being applied")
            print("  → Could be pigpio PWM initialization issue")
            print("  → Could be GPIO 17 (left PWM) conflict")
            print("")
            print("If motors ALTERNATE (left forward, right backward):")
            print("  → This is the BUG you described!")
            print("  → pigpio PWM channels interfering with each other")
            print("  → Need to fix PWM initialization in motor_controller_node.py")
            print("")
            print("If BOTH motors work here but not in ROS:")
            print("  → Hardware is fine")
            print("  → Bug is in motor_controller_node.py ROS code")
            print("")
            self.print_diagnostic_summary()
            
        except KeyboardInterrupt:
            print("\n\n⚠️  Test interrupted by user!")
        finally:
            self.stop_all_motors()
    
    def print_diagnostic_summary(self):
        """Print troubleshooting guide"""
        print("\n📊 DIAGNOSTIC GUIDE:")
        print("─"*70)
        print("\n✓ If all motors work correctly:")
        print("  → GPIO wiring is correct")
        print("  → Issue is likely in ROS motor_driver_node configuration")
        print("  → Check motor inversion parameters in ROS")
        print("")
        print("❌ If NO motors work:")
        print("  → Check L298N power supply (need 7-12V on Vs)")
        print("  → Verify L298N and Pi share common ground")
        print("  → Check if pigpiod is running: sudo systemctl status pigpiod")
        print("  → Try: sudo usermod -aG gpio $USER (then logout/login)")
        print("")
        print("❌ If only LEFT motor works:")
        print("  → Check right motor wiring to L298N OUT3/OUT4")
        print("  → Verify GPIO 23, 24, 18 connections")
        print("  → Test right motor by swapping wires with left")
        print("")
        print("❌ If only RIGHT motor works:")
        print("  → Check left motor wiring to L298N OUT1/OUT2")
        print("  → Verify GPIO 27, 22, 17 connections")
        print("  → Test left motor by swapping wires with right")
        print("")
        print("❌ If motor spins WRONG direction:")
        print("  → Either swap motor wires at L298N output, OR")
        print("  → In ROS, set inversion parameter:")
        print("    ros2 param set /motor_driver_node left_inverted true")
        print("    ros2 param set /motor_driver_node right_inverted true")
        print("")
        print("❌ If motors work individually but NOT together:")
        print("  → POWER SUPPLY INSUFFICIENT!")
        print("  → Need higher current capacity")
        print("  → Check battery voltage under load")
        print("")
        print("❌ If motors are weak or stuttering:")
        print("  → L298N Enable jumpers may be missing (need jumpers OR PWM)")
        print("  → Increase PWM duty cycle (edit TEST_SPEED_MED in script)")
        print("  → Check for loose connections")
        print("")
        print("="*70)
        print("GPIO PIN REFERENCE (BCM numbering):")
        print("─"*70)
        print("Left Motor:  IN1=GPIO27 (Pin13), IN2=GPIO22 (Pin15), ENA=GPIO17 (Pin11)")
        print("Right Motor: IN1=GPIO23 (Pin16), IN2=GPIO24 (Pin18), ENB=GPIO18 (Pin12)")
        print("="*70)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        print("\nCleaning up GPIO...")
        self.stop_all_motors()
        
        if USE_PIGPIO and self.pi:
            self.pi.stop()
        else:
            if self.pwm_left:
                self.pwm_left.stop()
            if self.pwm_right:
                self.pwm_right.stop()
            GPIO.cleanup()
        
        print("✓ GPIO cleanup complete")

def main():
    """Main entry point"""
    tester = None
    try:
        tester = MotorTester()
        tester.run_full_diagnostic()
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if tester:
            tester.cleanup()

if __name__ == '__main__':
    main()
