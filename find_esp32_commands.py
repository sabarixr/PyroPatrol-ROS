#!/usr/bin/env python3
"""
Simple ESP32 Command Tester - Figure out what commands work
"""

import serial
import time

print("🔍 ESP32 Command Discovery")
print("=" * 50)

# Connect to ESP32
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

print("Connected! Type commands to test (Ctrl+C to exit)")
print("Try commands like:")
print("  FORWARD")
print("  BACKWARD")
print("  LEFT")
print("  RIGHT")
print("  STOP")
print("  L 100")
print("  R 100")
print()

try:
    while True:
        # Send command
        cmd = input("Command: ")
        ser.write((cmd + '\n').encode())
        time.sleep(0.1)
        
        # Read response
        if ser.in_waiting:
            response = ser.readline().decode().strip()
            print(f"ESP32 says: {response}")
        else:
            print("(no response)")
        print()
        
except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()
