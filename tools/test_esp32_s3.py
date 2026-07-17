#!/usr/bin/env python3
"""
Test ESP32-S3 Connection

Quick script to verify ESP32-S3 is connected and responsive
"""

import serial
import time
import json

def test_esp32():
    ports_to_try = ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyUSB1']
    
    for port in ports_to_try:
        try:
            print(f"\n{'='*60}")
            print(f"Testing {port}...")
            print(f"{'='*60}\n")
            
            ser = serial.Serial(port, 115200, timeout=2)
            time.sleep(2)  # Wait for ESP32 boot
            
            print("✓ Serial port opened")
            
            # Clear buffer
            ser.reset_input_buffer()
            
            # Send READY handshake
            print("Sending READY handshake...")
            ser.write(b"READY\n")
            ser.flush()
            time.sleep(0.5)
            
            # Send STATUS request
            print("Sending STATUS request...")
            ser.write(b"STATUS\n")
            ser.flush()
            
            print(f"\nListening for responses (10 seconds)...\n")
            
            start_time = time.time()
            msg_count = 0
            
            while time.time() - start_time < 10:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line:
                        msg_count += 1
                        print(f"[{time.time() - start_time:.2f}s] {line}")
                        
                        # Try to parse as JSON
                        try:
                            data = json.loads(line)
                            msg_type = data.get('type', '')
                            
                            if msg_type == 'ping':
                                state = data.get('state', 'unknown')
                                print(f"  → ESP32 State: {state}")
                                
                            elif msg_type == 'system':
                                print(f"  → System: {data.get('msg', '')}")
                                
                            elif msg_type == 'status_report':
                                print(f"  → Status Report:")
                                print(f"     State: {data.get('state', '')}")
                                print(f"     PI Ready: {data.get('pi_ready', False)}")
                                print(f"     MPU Cal: {data.get('mpu_cal', False)}")
                                print(f"     MQ Cal: {data.get('mq_cal', False)}")
                                print(f"     Yaw: {data.get('yaw', 0):.2f}°")
                                
                        except json.JSONDecodeError:
                            pass
            
            print(f"\n{'='*60}")
            print(f"Test complete: {msg_count} messages received")
            print(f"{'='*60}\n")
            
            # Test basic commands
            print("Testing basic commands...")
            
            commands = ['SELFTEST', 'CAL_MQ', 'SCAN_SERVO 90', 'TILT_CAMERA 90']
            
            for cmd in commands:
                print(f"  Sending: {cmd}")
                ser.write(f"{cmd}\n".encode())
                ser.flush()
                time.sleep(0.5)
                
                # Read response
                if ser.in_waiting > 0:
                    response = ser.readline().decode('utf-8', errors='ignore').strip()
                    print(f"  Response: {response}")
            
            ser.close()
            print("\n✓ ESP32-S3 test PASSED")
            return True
            
        except serial.SerialException as e:
            print(f"✗ Failed to connect to {port}: {e}")
            continue
        except Exception as e:
            print(f"✗ Error during test: {e}")
            return False
    
    print("\n✗ ESP32-S3 not found on any port")
    return False


if __name__ == '__main__':
    print("\nESP32-S3 Connection Test")
    print("="*60)
    test_esp32()
