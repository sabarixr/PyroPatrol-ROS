#!/usr/bin/env python3
"""
Direct ESP32 Serial Monitor
Read and display everything the ESP32 sends
"""

import serial
import time
import sys

# Try common ports
PORTS = ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyUSB1']

def find_esp32():
    """Find ESP32 on available ports"""
    for port in PORTS:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            print(f"✓ Found ESP32 on {port}")
            return ser
        except Exception as e:
            print(f"✗ {port}: {e}")
    return None

def main():
    print("=" * 60)
    print("ESP32 Serial Monitor")
    print("=" * 60)
    
    ser = find_esp32()
    if not ser:
        print("\n❌ Could not find ESP32 on any port!")
        print("Tried:", PORTS)
        sys.exit(1)
    
    print(f"\n📡 Listening on {ser.port}...")
    print("Press Ctrl+C to stop\n")
    print("-" * 60)
    
    try:
        while True:
            if ser.in_waiting > 0:
                try:
                    # Read line
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        timestamp = time.strftime('%H:%M:%S')
                        print(f"[{timestamp}] {line}")
                        
                        # Highlight important messages
                        if 'error' in line.lower() or 'not_ready' in line.lower():
                            print("    ⚠️  ERROR DETECTED")
                        elif 'ready' in line.lower():
                            print("    ✓ READY STATE")
                        elif 'ping' in line.lower():
                            print("    🔔 PING")
                        elif 'handshake' in line.lower():
                            print("    🤝 HANDSHAKE")
                            
                except UnicodeDecodeError:
                    pass
            else:
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\n\n" + "=" * 60)
        print("Stopped")
        print("=" * 60)
        ser.close()

if __name__ == '__main__':
    main()
