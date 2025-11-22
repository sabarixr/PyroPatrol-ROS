# 🚒 ESP32-Based Fire Fighter Rover - Complete Setup Guide

## 📋 Overview

This is a complete rewrite of the rover control system to use ESP32 for motor control and sensor reading, eliminating jerking and providing smooth operation.

### Architecture:
- **ESP32**: Motor control, encoders, fire sensors, actuators
- **Raspberry Pi**: Camera, high-level navigation, ROS 2 coordination
- **Serial Communication**: Commands via USB (115200 baud)

---

## 🔧 Hardware Setup

### ESP32 Connections (from your documentation):
```
MOTORS:
  Left Motor:  AIN1=Pin10, AIN2=Pin11, PWMA=Pin12
  Right Motor: BIN1=Pin16, BIN2=Pin17, PWMB=Pin18

SENSORS:
  MQ2 Smoke:      Pin 1 (Analog)
  MQ5 Gas:        Pin 2 (Analog)
  Flame Sensor:   Pin 7 (Digital)
  Temperature:    TMP117 on I2C (SDA=Pin8, SCL=Pin9)
  
ODOMETRY:
  Left Encoder:   Pin 13
  Right Encoder:  Pin 14

ACTUATORS:
  Scan Servo:     Pin 38 (sweeps for fire)
  Turret Servo:   Pin 39 (aims at target)
  Water Pump:     Pin 40 (relay)

SERIAL:
  Baudrate: 115200
  Connection: USB to Raspberry Pi
```

### Raspberry Pi Connections:
- **Camera**: CSI ribbon cable or USB webcam
- **ESP32**: USB connection (usually `/dev/ttyUSB0` or `/dev/ttyACM0`)

---

## 📦 Software Installation

### 1. Install Dependencies on Raspberry Pi

```bash
cd /home/alibaba/frr_ws

# Install Python serial library
sudo apt update
sudo apt install python3-serial

# Set permissions for ESP32
sudo usermod -a -G dialout $USER

# Reboot to apply permissions
sudo reboot
```

### 2. Build the New ROS 2 Packages

```bash
cd /home/alibaba/frr_ws

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

---

## 🚀 Usage

### Launch the ESP32-Based Rover

```bash
# Source workspace
cd /home/alibaba/frr_ws
source install/setup.bash

# Launch the rover (will start ESP32 bridge, sensors, and camera)
ros2 launch frr_bringup esp32_rover_bringup.launch.py

# If ESP32 is on a different port:
ros2 launch frr_bringup esp32_rover_bringup.launch.py serial_port:=/dev/ttyACM0
```

### Control the Rover

In a new terminal:
```bash
cd /home/alibaba/frr_ws
source install/setup.bash

# Run ESP32 teleop
ros2 run frr_navigation esp32_teleop_node
```

---

## 🎮 Control Reference

### Movement Controls:
- **W/S**: Forward/Backward
- **A/D**: Turn Left/Right  
- **Q/E**: Forward+Turn combinations
- **X**: Stop, **SPACE**: Emergency stop

### Camera Controls:
- **I/K**: Tilt up/down (±5°)
- **Shift+I/K**: Fast tilt (±15°)
- **O/L**: Look up/down (±45°)
- **U**: Center camera

### Fire Fighting:
- **M**: Toggle fire scanning mode (autonomous detection)
- **P**: Toggle water pump

### Speed Controls (Real-time):
- **R/F**: Increase/decrease all speeds (±5%)
- **Shift+R/F**: Fast speed change (±20%)
- **T/G**: Linear speed only
- **Y/H**: Angular speed only
- **1/2/3**: Speed presets (Slow/Normal/Fast)

---

## 📡 ROS 2 Topics

### Published by ESP32 Bridge:
- `/motor_status` (String): Current motor speeds
- `/esp32_telemetry` (String): Raw JSON telemetry
- `/sensors/smoke` (Range): MQ2 smoke sensor reading
- `/sensors/gas` (Range): MQ5 gas sensor reading
- `/sensors/flame` (Bool): Flame detection status
- `/sensors/temperature` (Temperature): TMP117 temperature

### Published by ESP32 Sensor Node:
- `/fire_detected` (Bool): Fire detection status
- `/hazard_level` (String): "LOW", "MODERATE", "HIGH", "CRITICAL"

### Subscribed by ESP32 Bridge:
- `/cmd_vel` (Twist): Velocity commands
- `/camera_tilt` (Float64): Camera servo angle
- `/water_pump` (Bool): Water pump on/off
- `/fire_scan` (Bool): Fire scanning mode on/off

---

## 🔥 Fire Detection System

The ESP32 automatically:
1. Sweeps servo from 30° to 150°
2. Monitors flame sensor, smoke, gas, temperature
3. When fire detected:
   - Aims turret servo at target
   - Activates water pump
   - Continues scanning

### Enable Autonomous Fire Detection:
```bash
# Via teleop: Press 'M'

# Or via command line:
ros2 topic pub /fire_scan std_msgs/msg/Bool "{data: true}" --once
```

---

## 🔧 Troubleshooting

### ESP32 Not Connecting:

```bash
# Find ESP32 port
ls /dev/ttyUSB* /dev/ttyACM*

# Test connection
screen /dev/ttyUSB0 115200
# Type: STATUS
# Ctrl+A then K to exit

# Check permissions
groups  # Should include 'dialout'
```

### Motors Not Responding:

Check ESP32 serial output:
```bash
ros2 topic echo /motor_status
```

Send test command:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --once
```

### Camera Issues:

```bash
# Check camera device
ls /dev/video*

# Test camera
ros2 topic echo /camera/image_raw
```

### Video Stream:

Access at: `http://192.168.1.6:8080/stream.mjpg`

---

## 📊 Monitoring

### View Sensor Data:

```bash
# Fire detection status
ros2 topic echo /fire_detected

# Hazard level
ros2 topic echo /hazard_level

# Raw telemetry (JSON)
ros2 topic echo /esp32_telemetry

# Temperature
ros2 topic echo /sensors/temperature

# Smoke level
ros2 topic echo /sensors/smoke
```

### View Motor Status:

```bash
ros2 topic echo /motor_status
```

---

## 🎯 Key Improvements Over Old System

✅ **No Jerking**: ESP32 handles motor control smoothly  
✅ **Better Odometry**: Encoder data from ESP32  
✅ **Integrated Fire Detection**: Autonomous scanning and spraying  
✅ **Real-time Telemetry**: JSON sensor data every 800ms  
✅ **Modular Design**: Easy to add/modify sensors  
✅ **Non-blocking Commands**: Instant response  
✅ **Camera on Pi**: High-quality video streaming  

---

## 🚀 Quick Start Checklist

- [ ] ESP32 programmed and powered
- [ ] ESP32 connected to Raspberry Pi via USB
- [ ] Motor driver powered (6-12V battery)
- [ ] Camera connected to Raspberry Pi
- [ ] Python serial installed
- [ ] User in dialout group (rebooted)
- [ ] ROS 2 packages built
- [ ] Launch file executed successfully
- [ ] Teleop working
- [ ] Video stream accessible

---

## 📝 Development Notes

### Adding New ESP32 Commands:

Edit `esp32_bridge_node.py` and add to `cmd_vel_callback()` or create new subscribers.

### Modifying Fire Detection Thresholds:

Edit `esp32_sensor_node.py`:
```python
self.MQ2_THRESHOLD = 600  # Smoke
self.MQ5_THRESHOLD = 550  # Gas
self.TEMP_RISE_MIN = 2.0  # Temperature rise
```

### Camera Settings:

Edit launch file `esp32_rover_bringup.launch.py`:
```python
'frame_width': 640,
'frame_height': 480,
'fps': 30,
```

---

**Your ESP32-based Fire Fighter Rover is ready! 🔥🤖💦**