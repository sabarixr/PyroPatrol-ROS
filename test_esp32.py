#!/usr/bin/env python3
"""
Test ESP32 Serial Communication
"""

import serial
import time
import json

def test_esp32_communication():
    print("🚒 ESP32 Communication Test")
    print("=" * 50)
    
    # Try to find ESP32 port
    ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1']
    ser = None
    
    for port in ports:
        try:
            print(f"Trying {port}...")
            ser = serial.Serial(port, 115200, timeout=1)
            print(f"✅ Connected to ESP32 on {port}")
            break
        except Exception as e:
            print(f"❌ {port}: {e}")
            continue
    
    if not ser:
        print("\n❌ Could not connect to ESP32!")
        print("Check:")
        print("  1. ESP32 is connected via USB")
        print("  2. USB cable supports data transfer")
        print("  3. User has permissions (sudo usermod -a -G dialout $USER)")
        return
    
    time.sleep(2)  # Wait for ESP32 to stabilize
    
    print("\n📡 Testing Communication...")
    print("-" * 50)
    
    # Test 1: Request sensor data
    print("\n1️⃣  Testing sensor data request...")
    cmd = {"cmd": "get_sensors"}
    ser.write((json.dumps(cmd) + '\n').encode())
    time.sleep(0.1)
    
    if ser.in_waiting:
        response = ser.readline().decode().strip()
        print(f"Response: {response}")
    else:
        print("No response from ESP32")
    
    # Test 2: Motor control
    print("\n2️⃣  Testing motor control...")
    test_commands = [
        {"cmd": "motors", "left": 50, "right": 50},   # Forward
        {"cmd": "motors", "left": 0, "right": 0},     # Stop
        {"cmd": "motors", "left": -50, "right": -50}, # Backward
        {"cmd": "motors", "left": 0, "right": 0},     # Stop
    ]
    
    for cmd in test_commands:
        print(f"Sending: {cmd}")
        ser.write((json.dumps(cmd) + '\n').encode())
        time.sleep(1)
        
        if ser.in_waiting:
            response = ser.readline().decode().strip()
            print(f"Response: {response}")
    
    # Test 3: Servo control
    print("\n3️⃣  Testing servo control...")
    servo_angles = [0, 45, -45, 0]
    
    for angle in servo_angles:
        cmd = {"cmd": "servo", "angle": angle}
        print(f"Setting servo to {angle}°")
        ser.write((json.dumps(cmd) + '\n').encode())
        time.sleep(1)
        
        if ser.in_waiting:
            response = ser.readline().decode().strip()
            print(f"Response: {response}")
    
    # Test 4: Continuous sensor reading
    print("\n4️⃣  Reading sensors for 5 seconds...")
    start_time = time.time()
    
    while time.time() - start_time < 5:
        cmd = {"cmd": "get_sensors"}
        ser.write((json.dumps(cmd) + '\n').encode())
        time.sleep(0.2)
        
        if ser.in_waiting:
            response = ser.readline().decode().strip()
            try:
                data = json.loads(response)
                print(f"Sensors: {data}")
            except:
                print(f"Raw: {response}")
    
    # Stop motors
    print("\n🛑 Stopping motors...")
    cmd = {"cmd": "motors", "left": 0, "right": 0}
    ser.write((json.dumps(cmd) + '\n').encode())
    
    ser.close()
    print("\n✅ Test complete!")

if __name__ == '__main__':
    try:
        test_esp32_communication()
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")