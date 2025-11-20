#!/usr/bin/env python3
"""Test all motor GPIO pins individually"""
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup all pins
pins = {
    'LEFT_IN1': 17,
    'LEFT_IN2': 27,
    'LEFT_PWM': 12,
    'RIGHT_IN1': 22,
    'RIGHT_IN2': 23,
    'RIGHT_PWM': 13
}

for name, pin in pins.items():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

print("GPIO PIN TEST")
print("=" * 60)

# Test each pin
for name, pin in pins.items():
    print(f"\nTesting {name} (GPIO {pin})")
    print("Setting HIGH for 2 seconds...")
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(2)
    GPIO.output(pin, GPIO.LOW)
    print("Set back to LOW")
    input("Press Enter to test next pin...")

print("\nTest complete!")
GPIO.cleanup()
