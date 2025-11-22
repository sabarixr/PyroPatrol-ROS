# 🤖 FIREBOT - Raspberry Pi Configuration Guide

## 📋 Hardware Overview

### ESP32 Pin Connections
```
MOTORS:
  Left Motor:  AIN1=Pin10, AIN2=Pin11, PWMA=Pin12
  Right Motor: BIN1=Pin16, BIN2=Pin17, PWMB=Pin18

SENSORS:
  MQ2 Smoke:      Pin 1 (Analog)
  MQ5 Gas:        Pin 2 (Analog)
  Flame Sensor:   Pin 7 (Digital, LOW=flame detected)
  Temperature:    TMP117 on I2C (SDA=Pin8, SCL=Pin9)
  
ODOMETRY:
  Left Encoder:   Pin 13 (IR sensor)
  Right Encoder:  Pin 14 (IR sensor)

ACTUATORS:
  Scan Servo:     Pin 38 (sweeps 30°-150°)
  Turret Servo:   Pin 39 (aims at target)
  Water Pump:     Pin 40 (relay control)

SERIAL:
  Baudrate: 115200
  Connection: USB to Raspberry Pi
```

---

## 🔌 Raspberry Pi Setup

### 1. Install Python Serial Library
```bash
sudo apt update
sudo apt install python3-serial python3-pip
pip3 install pyserial
```

### 2. Find ESP32 USB Port
```bash
# List connected devices
ls /dev/ttyUSB* /dev/ttyACM*

# Usually it's:
# /dev/ttyUSB0 or /dev/ttyACM0
```

### 3. Set Permissions
```bash
# Add user to dialout group (one time)
sudo usermod -a -G dialout $USER

# Reboot for changes to take effect
sudo reboot
```

---

## 🐍 Python Control Script

### Basic Control Example
```python
#!/usr/bin/env python3
import serial
import time

# Connect to ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for connection

def send_command(cmd):
    """Send command to ESP32"""
    ser.write(f"{cmd}\n".encode())
    time.sleep(0.1)
    
    # Read response
    while ser.in_waiting:
        response = ser.readline().decode().strip()
        print(f"ESP32: {response}")

# Example commands
send_command("STATUS")        # Check status
send_command("FORWARD")       # Drive forward
time.sleep(2)
send_command("STOP")          # Stop

send_command("LEFT")          # Turn left
time.sleep(1)
send_command("STOP")

send_command("SCAN")          # Start fire scanning
time.sleep(10)
send_command("DISABLE")       # Stop scanning

ser.close()
```

---

## 📡 Command Reference

### Movement Commands
| Command | Description | Example |
|---------|-------------|---------|
| `FORWARD` | Drive forward at 50% | `FORWARD` |
| `BACKWARD` | Drive backward at 50% | `BACKWARD` |
| `LEFT` | Turn left (right motor only) | `LEFT` |
| `RIGHT` | Turn right (left motor only) | `RIGHT` |
| `STOP` | Stop all motors | `STOP` |

### Speed Control
| Command | Description | Example |
|---------|-------------|---------|
| `SPEED <0-100>` | Set both motors | `SPEED 75` |
| `L <0-100>` | Set left motor | `L 50` |
| `R <0-100>` | Set right motor | `R 80` |
| `DRIVE <L> <R>` | Set both individually | `DRIVE 30 70` |

### Fire Detection
| Command | Description |
|---------|-------------|
| `SCAN` | Enable autonomous fire scanning |
| `DISABLE` | Disable fire scanning |
| `PUMP_ON` | Turn pump on manually |
| `PUMP_OFF` | Turn pump off manually |

### Testing
| Command | Description |
|---------|-------------|
| `STATUS` | Show motor speeds and RPM |
| `TEST_BOTH` | Test both motors for 1 sec |
| `TEST_LEFT` | Test left motor |
| `TEST_RIGHT` | Test right motor |

---

## 🔥 Fire Detection Modes

### Autonomous Mode
```python
send_command("SCAN")
# Robot will:
# - Sweep servo from 30° to 150°
# - Detect flames automatically
# - Aim turret and spray water
# - Continue scanning
```

### Manual Control While Scanning
```python
send_command("SCAN")      # Start scanning
send_command("SPEED 30")  # Drive forward while scanning
send_command("LEFT")      # Turn left while scanning
send_command("DISABLE")   # Stop scanning
```

---

## 📊 Reading Telemetry Data

### JSON Output Format
The ESP32 sends JSON telemetry every 800ms when scanning:
```json
{
  "scan": 90,
  "mq2": 450,
  "mq5": 380,
  "flame": 1,
  "temp": 28.5,
  "l_rpm": 45.2,
  "r_rpm": 47.8
}
```

### Python Parser
```python
import json

def read_telemetry():
    while True:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            
            # Check if it's JSON
            if line.startswith('{'):
                try:
                    data = json.loads(line)
                    print(f"Scan Angle: {data['scan']}°")
                    print(f"Smoke (MQ2): {data['mq2']}")
                    print(f"Temperature: {data['temp']}°C")
                    print(f"Left RPM: {data['l_rpm']}")
                except json.JSONDecodeError:
                    pass
            else:
                # Regular text messages
                print(f"MSG: {line}")
```

---

## 🎮 Complete Robot Control Example

```python
#!/usr/bin/env python3
import serial
import time
import json

class Firebot:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        print("Firebot connected!")
    
    def send(self, cmd):
        self.ser.write(f"{cmd}\n".encode())
        time.sleep(0.1)
    
    def forward(self, duration=1):
        self.send("FORWARD")
        time.sleep(duration)
        self.send("STOP")
    
    def turn_left(self, duration=0.5):
        self.send("LEFT")
        time.sleep(duration)
        self.send("STOP")
    
    def turn_right(self, duration=0.5):
        self.send("RIGHT")
        time.sleep(duration)
        self.send("STOP")
    
    def start_scanning(self):
        self.send("SCAN")
        print("Fire scanning enabled")
    
    def stop_scanning(self):
        self.send("DISABLE")
        print("Fire scanning disabled")
    
    def patrol(self):
        """Simple patrol pattern"""
        for _ in range(4):
            self.forward(2)
            self.turn_right(0.5)
    
    def close(self):
        self.send("STOP")
        self.ser.close()

# Usage
robot = Firebot()

# Manual control
robot.forward(2)
robot.turn_left(1)
robot.forward(1)

# Autonomous fire detection
robot.start_scanning()
time.sleep(30)  # Scan for 30 seconds
robot.stop_scanning()

# Patrol mode
robot.patrol()

robot.close()
```

---

## 🔧 Troubleshooting

### ESP32 Not Responding
```bash
# Check connection
ls -l /dev/ttyUSB*

# Test with screen
screen /dev/ttyUSB0 115200
# Type: STATUS
# Press Ctrl+A then K to exit
```

### Permission Denied
```bash
# Check groups
groups

# Add to dialout if missing
sudo usermod -a -G dialout $USER
sudo reboot
```

### Commands Not Working
1. Check baud rate is 115200
2. Commands auto-execute after 1 second
3. Or press Enter after each command
4. Type `STATUS` to verify connection

---

## 🚀 Quick Start Checklist

- [ ] ESP32 powered and connected via USB
- [ ] Motor driver has 6-12V battery power
- [ ] GND connected between ESP32 and motor driver
- [ ] Python serial installed: `pip3 install pyserial`
- [ ] User in dialout group
- [ ] Test with: `python3 -m serial.tools.miniterm /dev/ttyUSB0 115200`
- [ ] Type `STATUS` to verify

---

## 📝 Notes

- **Motor Direction Fix Applied**: Motors now spin in correct direction
- **Non-Blocking**: Commands respond instantly even during scanning
- **Auto-Execute**: Commands run 1 second after typing (no Enter needed)
- **Fire Override**: Flame sensor triggers immediate spray
- **Telemetry**: JSON data sent every 800ms during scanning
- **Odometry**: Wheel encoders track RPM on both motors

---

## 💡 Advanced Features

### Sensor Thresholds (in code)
```cpp
#define MQ2_THRESHOLD 600    // Smoke detection
#define MQ5_THRESHOLD 550    // Gas detection  
#define TEMP_RISE_MIN 2.0    // Temperature rise in °C
```

### Scan Parameters
```cpp
SCAN_MIN = 30°         // Left scan limit
SCAN_MAX = 150°        // Right scan limit
SWEEP_STEP = 3°        // Angle increment
SWEEP_INTERVAL = 33ms  // Sweep speed
```

### Pump Duration
```cpp
PUMP_DURATION = 2000ms  // Water spray time
```

---

**Ready to control your Firebot! 🔥🤖💦**