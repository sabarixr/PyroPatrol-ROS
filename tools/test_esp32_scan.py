#!/usr/bin/env python3
"""
Test ESP32 SCAN command via serial
Connect to ESP32 and send SCAN command to see what it returns
"""

import serial
import time
import json

# Configure serial port
SERIAL_PORT = '/dev/ttyACM0'  # Change if needed
BAUD_RATE = 115200

def main():
    print(f"Connecting to ESP32 on {SERIAL_PORT} at {BAUD_RATE} baud...")
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        
        # Clear any existing data
        ser.reset_input_buffer()
        
        print("✓ Connected to ESP32")
        print("\n" + "="*60)
        print("Sending SCAN command...")
        print("="*60 + "\n")
        
        # Send SCAN command
        ser.write(b"SCAN\n")
        ser.flush()
        
        print("Listening for ESP32 responses (30 seconds)...\n")
        
        start_time = time.time()
        scan_complete = False
        
        while time.time() - start_time < 30:  # Listen for 30 seconds
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line:
                    print(f"[{time.time() - start_time:.2f}s] {line}")
                    
                    # Try to parse as JSON
                    try:
                        data = json.loads(line)
                        msg_type = data.get('type', 'unknown')
                        
                        if msg_type == 'ack' and data.get('cmd') == 'scan':
                            print("  ✓ SCAN command acknowledged")
                        
                        elif msg_type == 'scan_sample':
                            angle = data.get('angle', '?')
                            mq2 = data.get('mq2', '?')
                            mq5 = data.get('mq5', '?')
                            temp = data.get('temp', '?')
                            flame = data.get('flame', '?')
                            print(f"  📊 Sample: angle={angle}° mq2={mq2} mq5={mq5} temp={temp}°C flame={flame}")
                        
                        elif msg_type == 'scan_result':
                            angle = data.get('angle', '?')
                            score = data.get('score', '?')
                            dominant = data.get('dominant', '?')
                            print(f"  🎯 Result: angle={angle}° score={score} dominant={dominant}")
                        
                        elif msg_type == 'scan_complete':
                            direction = data.get('direction', 'none')
                            reason = data.get('reason', '?')
                            left_score = data.get('left', {}).get('score', '?')
                            right_score = data.get('right', {}).get('score', '?')
                            
                            print("\n" + "="*60)
                            print("🏁 SCAN COMPLETE!")
                            print("="*60)
                            print(f"  ESP32 Direction: {direction}")
                            print(f"  Reason: {reason}")
                            print(f"  Left Score: {left_score}")
                            print(f"  Right Score: {right_score}")
                            print(f"\n  ⚠️  REMINDER: ESP32 'left' = Robot RIGHT")
                            print(f"              ESP32 'right' = Robot LEFT")
                            
                            if direction == 'left':
                                print(f"\n  🤖 Robot should turn: RIGHT")
                            elif direction == 'right':
                                print(f"\n  🤖 Robot should turn: LEFT")
                            else:
                                print(f"\n  🤖 Robot action: {direction.upper()}")
                            
                            print("="*60 + "\n")
                            scan_complete = True
                        
                        elif msg_type == 'alert':
                            print(f"  🔥 ALERT: {data.get('msg', 'Fire detected!')}")
                        
                        elif msg_type == 'pump':
                            status = data.get('status', '?')
                            print(f"  💧 Pump: {status.upper()}")
                    
                    except json.JSONDecodeError:
                        # Not JSON, just regular output
                        pass
            
            if scan_complete:
                print("Waiting for any additional messages (5 more seconds)...")
                time.sleep(5)
                break
        
        if not scan_complete:
            print("\n⚠️  SCAN did not complete within 30 seconds")
        
        print("\n" + "="*60)
        print("Test complete. Closing connection...")
        print("="*60)
        
        ser.close()
    
    except serial.SerialException as e:
        print(f"❌ Serial connection error: {e}")
        print(f"\nTroubleshooting:")
        print(f"  1. Check if ESP32 is connected: ls /dev/ttyUSB*")
        print(f"  2. Check permissions: sudo chmod 666 /dev/ttyUSB0")
        print(f"  3. Try different port: /dev/ttyACM0 or /dev/ttyUSB1")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        ser.close()

if __name__ == '__main__':
    main()
