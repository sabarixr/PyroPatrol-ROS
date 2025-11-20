#!/usr/bin/env python3
"""
Quick GPIO PWM test - verify hardware PWM pins work
"""
import sys
import time

try:
    import pigpio
    USE_PIGPIO = True
    print("✓ Using pigpio")
except ImportError:
    import RPi.GPIO as GPIO
    USE_PIGPIO = False
    print("✓ Using RPi.GPIO")

# Test pins
TEST_PINS = [12, 13, 18, 19]  # Hardware PWM pins

def test_pigpio():
    pi = pigpio.pi()
    if not pi.connected:
        print("❌ pigpiod not running. Start with: sudo pigpiod")
        return
    
    print("\n" + "="*60)
    print("Testing Hardware PWM pins with pigpio")
    print("="*60)
    
    for pin in TEST_PINS:
        print(f"\nTesting GPIO {pin}...")
        print(f"  Measure voltage on Pin {get_physical_pin(pin)}")
        
        # Set as output
        pi.set_mode(pin, pigpio.OUTPUT)
        
        # Set PWM frequency
        pi.set_PWM_frequency(pin, 1000)
        pi.set_PWM_range(pin, 255)
        
        # Test 50% duty cycle
        pi.set_PWM_dutycycle(pin, 128)
        print(f"  Set to 50% PWM (128/255)")
        print(f"  You should read ~1.65V on GPIO {pin}")
        
        input(f"  Press ENTER to test next pin...")
        
        # Turn off
        pi.set_PWM_dutycycle(pin, 0)
    
    # Cleanup
    for pin in TEST_PINS:
        pi.set_PWM_dutycycle(pin, 0)
    pi.stop()
    print("\n✓ Test complete")

def test_rpi_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    print("\n" + "="*60)
    print("Testing PWM pins with RPi.GPIO")
    print("="*60)
    
    for pin in TEST_PINS:
        print(f"\nTesting GPIO {pin}...")
        print(f"  Measure voltage on Pin {get_physical_pin(pin)}")
        
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, 1000)
        pwm.start(50)
        
        print(f"  Set to 50% PWM")
        print(f"  You should read ~1.65V on GPIO {pin}")
        
        input(f"  Press ENTER to test next pin...")
        
        pwm.stop()
    
    GPIO.cleanup()
    print("\n✓ Test complete")

def get_physical_pin(bcm):
    """Convert BCM to physical pin number"""
    mapping = {12: 32, 13: 33, 18: 12, 19: 35}
    return mapping.get(bcm, "?")

if __name__ == '__main__':
    print("\n⚠️  IMPORTANT: Connect multimeter to test each GPIO pin")
    print("Measure voltage between GPIO pin and GND\n")
    
    if USE_PIGPIO:
        test_pigpio()
    else:
        test_rpi_gpio()
