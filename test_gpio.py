#!/usr/bin/env python3
"""Test GPIO access methods"""

import sys

def test_rpi_gpio():
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        print("✓ RPi.GPIO working")
        GPIO.cleanup()
        return True
    except Exception as e:
        print(f"✗ RPi.GPIO failed: {e}")
        return False

def test_pigpio():
    try:
        import pigpio
        pi = pigpio.pi()
        if pi.connected:
            print("✓ pigpio working")
            pi.stop()
            return True
        else:
            print("✗ pigpio daemon not running")
            return False
    except Exception as e:
        print(f"✗ pigpio failed: {e}")
        return False

def test_smbus():
    try:
        import smbus
        bus = smbus.SMBus(1)
        print("✓ SMBus working")
        return True
    except Exception as e:
        print(f"✗ SMBus failed: {e}")
        return False

if __name__ == "__main__":
    print("Testing GPIO access methods...")
    rpi_ok = test_rpi_gpio()
    pigpio_ok = test_pigpio()
    smbus_ok = test_smbus()
    
    if rpi_ok:
        print("\n✓ RPi.GPIO is available - rover should work!")
    elif pigpio_ok:
        print("\n✓ pigpio is available - rover should work!")
    else:
        print("\n✗ No GPIO access available")
        sys.exit(1)
        
    if smbus_ok:
        print("✓ I2C is available - IMU should work!")
    else:
        print("✗ I2C not available - IMU will fail")
