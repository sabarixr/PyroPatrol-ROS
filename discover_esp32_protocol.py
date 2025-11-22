#!/usr/bin/env python3
"""
ESP32 Protocol Discovery - Figure out what commands your ESP32 actually accepts
"""

import serial
import time

def discover_protocol():
    print("🔍 ESP32 Protocol Discovery")
    print("=" * 50)
    
    # Connect to ESP32
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(2)
    
    # Clear buffer
    ser.reset_input_buffer()
    
    print("\n📝 Testing different command formats:")
    print("-" * 50)
    
    # Test different command formats
    test_commands = [
        # Text-based commands
        ("STOP", "Simple text command"),
        ("L 50", "Left motor text"),
        ("R 50", "Right motor text"),
        ("MOTORS 50 50", "Both motors text"),
        
        # JSON with lowercase
        ('{"cmd":"motors","left":50,"right":50}', "JSON lowercase"),
        ('{"CMD":"MOTORS","LEFT":50,"RIGHT":50}', "JSON uppercase"),
        
        # JSON with command types
        ('{"type":"motor","left":50,"right":50}', "JSON type motor"),
        ('{"action":"move","left":50,"right":50}', "JSON action move"),
    ]
    
    for cmd, description in test_commands:
        print(f"\n✉️  Testing: {description}")
        print(f"   Sending: {cmd}")
        
        # Clear input buffer
        ser.reset_input_buffer()
        
        # Send command
        ser.write((cmd + '\n').encode())
        time.sleep(0.5)
        
        # Read response
        responses = []
        while ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                responses.append(line)
        
        if responses:
            print(f"   ✅ Response:")
            for resp in responses:
                print(f"      {resp}")
        else:
            print(f"   ❌ No response")
    
    # Stop motors
    ser.write(b"STOP\n")
    
    ser.close()
    print("\n✅ Protocol discovery complete!")

if __name__ == '__main__':
    try:
        discover_protocol()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")