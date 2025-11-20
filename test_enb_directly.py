#!/usr/bin/env python3
"""
Direct test of ENB (GPIO 13) with motor running
This will help verify the wiring is correct
"""
import pigpio
import time

# Pins
MOTOR_B_IN1 = 23  # Right motor direction 1
MOTOR_B_IN2 = 24  # Right motor direction 2
MOTOR_B_ENB = 13  # Right motor PWM - GPIO 13 (Pin 33)

print("="*60)
print("DIRECT ENB (GPIO 13) TEST")
print("="*60)
print(f"\nWiring check:")
print(f"  GPIO 23 (Pin 16) → L298N IN3")
print(f"  GPIO 24 (Pin 18) → L298N IN4")
print(f"  GPIO 13 (Pin 33) → L298N ENB")
print(f"\nMeasure voltage on GPIO 13 (Pin 33) with multimeter")
print("="*60)

pi = pigpio.pi()
if not pi.connected:
    print("❌ pigpiod not running!")
    print("Start with: sudo pigpiod")
    exit(1)

print("\n✓ Connected to pigpiod")

# Setup
pi.set_mode(MOTOR_B_IN1, pigpio.OUTPUT)
pi.set_mode(MOTOR_B_IN2, pigpio.OUTPUT)
pi.set_mode(MOTOR_B_ENB, pigpio.OUTPUT)

# Setup PWM
print("\nSetting up PWM on GPIO 13...")
freq = pi.set_PWM_frequency(MOTOR_B_ENB, 1000)
print(f"  PWM frequency set to: {freq} Hz")

pi.set_PWM_range(MOTOR_B_ENB, 255)
print(f"  PWM range: 0-255")

# Stop everything first
pi.write(MOTOR_B_IN1, 0)
pi.write(MOTOR_B_IN2, 0)
pi.set_PWM_dutycycle(MOTOR_B_ENB, 0)
time.sleep(1)

print("\n" + "="*60)
print("TEST 1: PWM only (no direction)")
print("="*60)
print("Setting GPIO 13 to 50% PWM (duty=128/255)")
print("Measure voltage on GPIO 13 - should be ~1.65V")

pi.set_PWM_dutycycle(MOTOR_B_ENB, 128)
input("\nPress ENTER when measured...")

pi.set_PWM_dutycycle(MOTOR_B_ENB, 0)
time.sleep(0.5)

print("\n" + "="*60)
print("TEST 2: Direction + PWM (motor should spin)")
print("="*60)
print("Setting direction forward + 70% PWM")
print("Right motor should SPIN now")

# Set direction
pi.write(MOTOR_B_IN1, 0)
pi.write(MOTOR_B_IN2, 1)
time.sleep(0.01)

# Set PWM
duty = int(0.7 * 255)
pi.set_PWM_dutycycle(MOTOR_B_ENB, duty)
print(f"  IN1=0, IN2=1, ENB={duty}/255 (70%)")
print(f"\nMeasure voltage on GPIO 13 - should be ~2.3V (70% of 3.3V)")

for i in range(5, 0, -1):
    print(f"  Motor running... {i} seconds", end='\r')
    time.sleep(1)

print("\n\nStopping motor...")
pi.set_PWM_dutycycle(MOTOR_B_ENB, 0)
pi.write(MOTOR_B_IN1, 0)
pi.write(MOTOR_B_IN2, 0)

print("\n" + "="*60)
print("TEST 3: Different PWM levels")
print("="*60)

levels = [25, 50, 75, 100]
for percent in levels:
    duty = int((percent / 100.0) * 255)
    print(f"\nSetting {percent}% PWM (duty={duty}/255)")
    
    pi.write(MOTOR_B_IN1, 0)
    pi.write(MOTOR_B_IN2, 1)
    pi.set_PWM_dutycycle(MOTOR_B_ENB, duty)
    
    expected_voltage = (percent / 100.0) * 3.3
    print(f"  Expected voltage: {expected_voltage:.2f}V")
    print(f"  Motor should spin at {percent}% speed")
    
    time.sleep(3)

# Cleanup
print("\n\nCleaning up...")
pi.set_PWM_dutycycle(MOTOR_B_ENB, 0)
pi.write(MOTOR_B_IN1, 0)
pi.write(MOTOR_B_IN2, 0)
pi.stop()

print("✓ Test complete!")
print("\n" + "="*60)
print("DIAGNOSIS:")
print("="*60)
print("\nIf voltage was correct but motor didn't spin:")
print("  → Check L298N ENB wiring to GPIO 13 (Pin 33)")
print("  → Verify L298N has power (7-12V on Vs)")
print("  → Check motor wires connected to OUT3/OUT4")
print("\nIf no voltage on GPIO 13 during test:")
print("  → GPIO 13 might be in use by another process")
print("  → Try: sudo killall pigpiod && sudo pigpiod")
print("="*60)
