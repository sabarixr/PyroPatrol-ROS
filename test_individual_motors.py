#!/usr/bin/env python3
"""
Simple motor test script - tests each motor individually
This helps diagnose power supply or L298N enable pin issues
"""

import RPi.GPIO as GPIO
import time

# GPIO pin definitions (L298N connections)
MOTOR_A_IN1 = 17  # Left motor direction 1
MOTOR_A_IN2 = 27  # Left motor direction 2
MOTOR_B_IN1 = 22  # Right motor direction 1
MOTOR_B_IN2 = 23  # Right motor direction 2
MOTOR_A_ENA = 12  # Left motor PWM (speed)
MOTOR_B_ENB = 13  # Right motor PWM (speed)

def setup_gpio():
    """Initialize GPIO"""
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
    pwm_left = GPIO.PWM(MOTOR_A_ENA, 1000)
    pwm_right = GPIO.PWM(MOTOR_B_ENB, 1000)
    
    # Start PWM with 0%
    pwm_left.start(0)
    pwm_right.start(0)
    
    return pwm_left, pwm_right

def stop_all():
    """Stop all motors"""
    GPIO.output(MOTOR_A_IN1, GPIO.LOW)
    GPIO.output(MOTOR_A_IN2, GPIO.LOW)
    GPIO.output(MOTOR_B_IN1, GPIO.LOW)
    GPIO.output(MOTOR_B_IN2, GPIO.LOW)

def test_left_motor(pwm_left, speed=50):
    """Test left motor only"""
    print(f"\n🔧 Testing LEFT motor (MOTOR_A) at {speed}% PWM...")
    print("   GPIO Pins: IN1=17, IN2=27, ENA=12")
    
    # Stop right motor completely
    GPIO.output(MOTOR_B_IN1, GPIO.LOW)
    GPIO.output(MOTOR_B_IN2, GPIO.LOW)
    
    # Forward
    print("   → Forward (3 seconds)...")
    GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_A_IN2, GPIO.LOW)
    pwm_left.ChangeDutyCycle(speed)
    time.sleep(3)
    
    # Stop
    print("   → Stopping...")
    stop_all()
    pwm_left.ChangeDutyCycle(0)
    time.sleep(1)
    
    # Backward
    print("   → Backward (3 seconds)...")
    GPIO.output(MOTOR_A_IN1, GPIO.LOW)
    GPIO.output(MOTOR_A_IN2, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(speed)
    time.sleep(3)
    
    # Stop
    print("   → Stopping...")
    stop_all()
    pwm_left.ChangeDutyCycle(0)
    time.sleep(1)

def test_right_motor(pwm_right, speed=50):
    """Test right motor only"""
    print(f"\n🔧 Testing RIGHT motor (MOTOR_B) at {speed}% PWM...")
    print("   GPIO Pins: IN1=22, IN2=23, ENB=13")
    
    # Stop left motor completely
    GPIO.output(MOTOR_A_IN1, GPIO.LOW)
    GPIO.output(MOTOR_A_IN2, GPIO.LOW)
    
    # Forward
    print("   → Forward (3 seconds)...")
    GPIO.output(MOTOR_B_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_B_IN2, GPIO.LOW)
    pwm_right.ChangeDutyCycle(speed)
    time.sleep(3)
    
    # Stop
    print("   → Stopping...")
    stop_all()
    pwm_right.ChangeDutyCycle(0)
    time.sleep(1)
    
    # Backward
    print("   → Backward (3 seconds)...")
    GPIO.output(MOTOR_B_IN1, GPIO.LOW)
    GPIO.output(MOTOR_B_IN2, GPIO.HIGH)
    pwm_right.ChangeDutyCycle(speed)
    time.sleep(3)
    
    # Stop
    print("   → Stopping...")
    stop_all()
    pwm_right.ChangeDutyCycle(0)
    time.sleep(1)

def test_both_motors(pwm_left, pwm_right, speed=50):
    """Test both motors together"""
    print(f"\n🔧 Testing BOTH motors simultaneously at {speed}% PWM...")
    
    # Forward
    print("   → Both forward (3 seconds)...")
    GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_A_IN2, GPIO.LOW)
    GPIO.output(MOTOR_B_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_B_IN2, GPIO.LOW)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)
    time.sleep(3)
    
    # Stop
    print("   → Stopping...")
    stop_all()
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    time.sleep(1)

def main():
    print("=" * 60)
    print("  INDIVIDUAL MOTOR TEST")
    print("=" * 60)
    print("\nThis will test each motor separately, then together.")
    print("Watch which motors spin during each test.\n")
    print("⚠️  WARNING: Make sure robot is elevated or can't move!")
    print()
    input("Press ENTER to start test...")
    
    try:
        # Setup
        pwm_left, pwm_right = setup_gpio()
        
        # Test left motor
        test_left_motor(pwm_left, speed=40)
        
        # Test right motor
        test_right_motor(pwm_right, speed=40)
        
        # Test both together
        test_both_motors(pwm_left, pwm_right, speed=40)
        
        print("\n✓ Test complete!")
        print("\n" + "=" * 60)
        print("  DIAGNOSIS")
        print("=" * 60)
        print("\nIf LEFT motor works alone but not with both:")
        print("  → Power supply issue (not enough current)")
        print("  → Check L298N power input voltage (need 7-12V)")
        print()
        print("If RIGHT motor works alone but not with both:")
        print("  → Power supply issue (not enough current)")
        print("  → Check L298N power input voltage (need 7-12V)")
        print()
        print("If only one motor works in each test:")
        print("  → Wrong GPIO pin assignment")
        print("  → L298N enable jumpers missing")
        print("  → Motor wiring issue")
        print()
        print("If both work separately but NOT together:")
        print("  → POWER SUPPLY INSUFFICIENT!")
        print("  → Need higher current power supply")
        print("  → Check battery voltage under load")
        print()
        
    except KeyboardInterrupt:
        print("\nTest interrupted!")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        stop_all()
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        print("\nGPIO cleaned up")

if __name__ == '__main__':
    main()
