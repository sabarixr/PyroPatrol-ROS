#!/usr/bin/env python3
"""
Diagnostic script for MPU6050 I2C issues
This will help identify why the MPU6050 fails when running with motors
"""

import smbus
import time
import os
import sys

def check_i2c_permissions():
    """Check if user has I2C permissions"""
    print("\n=== Checking I2C Permissions ===")
    i2c_groups = os.popen('groups').read()
    if 'i2c' in i2c_groups or 'gpio' in i2c_groups:
        print("✓ User is in i2c/gpio group")
    else:
        print("✗ User is NOT in i2c/gpio group")
        print("  Run: sudo usermod -a -G i2c,gpio $USER")
        print("  Then logout and login again")
    
    # Check device permissions
    i2c_perms = os.popen('ls -l /dev/i2c-1').read()
    print(f"  /dev/i2c-1 permissions: {i2c_perms.strip()}")

def check_i2c_bus():
    """Check if I2C bus is accessible"""
    print("\n=== Checking I2C Bus ===")
    try:
        bus = smbus.SMBus(1)
        print("✓ I2C bus 1 opened successfully")
        bus.close()
        return True
    except Exception as e:
        print(f"✗ Failed to open I2C bus: {e}")
        return False

def detect_mpu6050():
    """Try to detect MPU6050 on I2C bus"""
    print("\n=== Detecting MPU6050 ===")
    try:
        bus = smbus.SMBus(1)
        addr = 0x68
        
        # Try to read WHO_AM_I register (0x75)
        who_am_i = bus.read_byte_data(addr, 0x75)
        print(f"✓ MPU6050 detected at 0x{addr:02X}")
        print(f"  WHO_AM_I register: 0x{who_am_i:02X} (should be 0x68)")
        
        bus.close()
        return True
    except Exception as e:
        print(f"✗ Failed to detect MPU6050: {e}")
        return False

def test_mpu6050_init():
    """Test MPU6050 initialization sequence"""
    print("\n=== Testing MPU6050 Initialization ===")
    try:
        bus = smbus.SMBus(1)
        addr = 0x68
        
        # Wake up the sensor
        print("  Waking up MPU6050...")
        bus.write_byte_data(addr, 0x6B, 0x00)
        time.sleep(0.1)
        
        # Read power management register
        pwr_mgmt = bus.read_byte_data(addr, 0x6B)
        print(f"  Power Management 1: 0x{pwr_mgmt:02X}")
        
        # Configure accelerometer
        print("  Configuring accelerometer...")
        bus.write_byte_data(addr, 0x1C, 0x00)
        
        # Configure gyroscope
        print("  Configuring gyroscope...")
        bus.write_byte_data(addr, 0x1B, 0x00)
        
        print("✓ MPU6050 initialized successfully")
        bus.close()
        return True
    except Exception as e:
        print(f"✗ Failed to initialize MPU6050: {e}")
        return False

def test_continuous_reading():
    """Test continuous reading to simulate ROS node behavior"""
    print("\n=== Testing Continuous Reading (10 samples) ===")
    try:
        bus = smbus.SMBus(1)
        addr = 0x68
        
        # Initialize sensor
        bus.write_byte_data(addr, 0x6B, 0x00)
        time.sleep(0.1)
        
        success_count = 0
        fail_count = 0
        
        for i in range(10):
            try:
                # Read accelerometer X-axis
                high = bus.read_byte_data(addr, 0x3B)
                low = bus.read_byte_data(addr, 0x3C)
                value = (high << 8) | low
                if value > 32767:
                    value -= 65536
                accel_x = value / 16384.0
                
                print(f"  Sample {i+1}: accel_x = {accel_x:+.3f}g")
                success_count += 1
                time.sleep(0.01)  # 100Hz like in ROS node
                
            except Exception as e:
                print(f"  Sample {i+1}: FAILED - {e}")
                fail_count += 1
                time.sleep(0.01)
        
        bus.close()
        
        print(f"\nResults: {success_count} successful, {fail_count} failed")
        return fail_count == 0
        
    except Exception as e:
        print(f"✗ Continuous reading test failed: {e}")
        return False

def test_with_retries():
    """Test I2C communication with retry logic"""
    print("\n=== Testing with Retry Logic ===")
    try:
        bus = smbus.SMBus(1)
        addr = 0x68
        
        # Initialize sensor
        bus.write_byte_data(addr, 0x6B, 0x00)
        time.sleep(0.1)
        
        max_retries = 3
        success_count = 0
        retry_count = 0
        
        for i in range(10):
            for attempt in range(max_retries):
                try:
                    high = bus.read_byte_data(addr, 0x3B)
                    low = bus.read_byte_data(addr, 0x3C)
                    value = (high << 8) | low
                    if value > 32767:
                        value -= 65536
                    accel_x = value / 16384.0
                    
                    if attempt > 0:
                        print(f"  Sample {i+1}: Success after {attempt} retries")
                        retry_count += attempt
                    success_count += 1
                    break
                    
                except Exception as e:
                    if attempt == max_retries - 1:
                        print(f"  Sample {i+1}: FAILED after {max_retries} retries - {e}")
                    time.sleep(0.001)  # Small delay before retry
            
            time.sleep(0.01)
        
        bus.close()
        
        print(f"\nResults: {success_count}/10 successful, {retry_count} total retries needed")
        return True
        
    except Exception as e:
        print(f"✗ Retry test failed: {e}")
        return False

def check_i2c_bus_speed():
    """Check I2C bus speed configuration"""
    print("\n=== Checking I2C Bus Speed ===")
    try:
        # Check /boot/config.txt or /boot/firmware/config.txt
        config_files = ['/boot/config.txt', '/boot/firmware/config.txt']
        
        for config_file in config_files:
            if os.path.exists(config_file):
                print(f"  Checking {config_file}...")
                with open(config_file, 'r') as f:
                    content = f.read()
                    if 'i2c_arm' in content or 'i2c_baudrate' in content or 'i2c_arm_baudrate' in content:
                        for line in content.split('\n'):
                            if 'i2c' in line.lower() and not line.strip().startswith('#'):
                                print(f"    {line}")
                    else:
                        print(f"    No I2C configuration found")
        
        print("\n  Recommended setting: dtparam=i2c_arm_baudrate=400000")
        print("  (400kHz is a good balance between speed and reliability)")
        
    except Exception as e:
        print(f"  Could not check I2C config: {e}")

def main():
    print("="*60)
    print("MPU6050 Diagnostic Tool")
    print("="*60)
    
    # Run all checks
    check_i2c_permissions()
    
    if not check_i2c_bus():
        print("\n❌ Cannot access I2C bus. Fix permissions first!")
        sys.exit(1)
    
    if not detect_mpu6050():
        print("\n❌ MPU6050 not detected. Check wiring!")
        sys.exit(1)
    
    if not test_mpu6050_init():
        print("\n❌ MPU6050 initialization failed!")
        sys.exit(1)
    
    test_continuous_reading()
    test_with_retries()
    check_i2c_bus_speed()
    
    print("\n" + "="*60)
    print("COMMON ISSUES AND SOLUTIONS:")
    print("="*60)
    print("1. I2C Bus Contention (Multiple processes accessing I2C)")
    print("   - Make sure only ONE program accesses MPU6050 at a time")
    print("   - Check: ps aux | grep python")
    print()
    print("2. I2C Bus Speed Too High")
    print("   - Add to /boot/config.txt: dtparam=i2c_arm_baudrate=100000")
    print("   - Reboot after changing")
    print()
    print("3. Power Issues")
    print("   - MPU6050 needs clean 3.3V power")
    print("   - Add 0.1uF capacitor near VCC/GND pins if possible")
    print()
    print("4. Timing Issues")
    print("   - Add delays in initialization code")
    print("   - Use retry logic for I2C reads")
    print()
    print("5. Pull-up Resistors")
    print("   - Check if SDA/SCL have 4.7kΩ pull-ups")
    print("   - Raspberry Pi has internal pull-ups but external may help")
    print("="*60)

if __name__ == '__main__':
    main()
