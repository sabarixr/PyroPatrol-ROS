# 🚀 Integrated Rover Bringup - Complete Guide

## Overview

The rover bringup system has been fully integrated with LiDAR navigation and obstacle avoidance. All functionality is now accessible through the main `rover_bringup.launch.py` file.

## What's Changed

### ✅ **Removed from rover_bringup.launch.py:**
- ❌ MMA8452 IMU node (sensor burned/not working)
- ❌ Old odometry_node (with sensor fusion)

### ✅ **Added to rover_bringup.launch.py:**
- ✨ **LiDAR Node** - 360° scanning
- ✨ **LiDAR Odometry Node** - Position tracking with triangulation
- ✨ **Obstacle Avoidance Node** - 4-corner safety system
- ⚙️ **enable_lidar** parameter - Enable/disable LiDAR features

### ✅ **Topic Flow Update:**
```
OLD (without LiDAR):
  Teleop → /cmd_vel → Motors

NEW (with LiDAR enabled):
  Teleop → /cmd_vel_teleop → Obstacle Avoidance → /cmd_vel → Motors
```

---

## 🎮 Launch Commands

### **1. Full System (Recommended)**
```bash
cd /home/alibaba/frr_ws
source install/setup.bash

# Launch rover with all features
ros2 launch frr_bringup rover_bringup.launch.py
```

**This starts:**
- ✅ MPU6050 IMU
- ✅ LiDAR scanning
- ✅ LiDAR odometry (starts at 0,0)
- ✅ Obstacle avoidance (4-corner detection)
- ✅ Camera
- ✅ Video streaming
- ✅ Motor control

### **2. Launch Teleop (Separate Terminal)**

**With Obstacle Avoidance (Recommended):**
```bash
ros2 run frr_control teleop_node --ros-args -r /cmd_vel:=/cmd_vel_teleop
```

**OR direct control (no safety):**
```bash
ros2 run frr_control teleop_node
```

> **Note:** Use `ros2 run` instead of `ros2 launch` for teleop to ensure keyboard access works correctly!

---

## ⚙️ Configuration Options

### Disable Video Streaming (Save CPU)
```bash
ros2 launch frr_bringup rover_bringup.launch.py enable_video_stream:=false
```

### Disable LiDAR (If not connected)
```bash
ros2 launch frr_bringup rover_bringup.launch.py enable_lidar:=false
```

### Minimal System (No video, no LiDAR)
```bash
ros2 launch frr_bringup rover_bringup.launch.py \
  enable_video_stream:=false \
  enable_lidar:=false
```

### Combine Options
```bash
# Rover with LiDAR but no video
ros2 launch frr_bringup rover_bringup.launch.py \
  enable_lidar:=true \
  enable_video_stream:=false
```

---

## 📊 System Architecture

### Complete Topic Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    SENSOR LAYER                             │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  LiDAR (/dev/ttyUSB0)          MPU6050 (I2C 0x68)          │
│         │                              │                     │
│         │ /scan                        │ /imu/mpu6050        │
│         │                              │                     │
└─────────┼──────────────────────────────┼─────────────────────┘
          │                              │
          │                              │
┌─────────┼──────────────────────────────┼─────────────────────┐
│         │         PROCESSING LAYER     │                     │
├─────────┼──────────────────────────────┼─────────────────────┤
│         │                              │                     │
│         ├──────────────────────────────┤                     │
│         │                              │                     │
│         ▼                              ▼                     │
│  ┌──────────────────┐      ┌──────────────────┐            │
│  │ lidar_odometry   │      │                  │            │
│  │                  │◄─────┤                  │            │
│  │ • Triangulation  │      │                  │            │
│  │ • Scan matching  │      │                  │            │
│  │ • Starts at 0,0  │      │                  │            │
│  └────────┬─────────┘      └──────────────────┘            │
│           │                                                  │
│           │ /lidar_odom                                      │
│           │                                                  │
│  ┌────────▼─────────────────────────────┐                  │
│  │  obstacle_avoidance_node             │                  │
│  │                                       │◄─── /scan        │
│  │  • Front zone    (-45° to +45°)     │                  │
│  │  • Back zone     (135° to 225°)     │                  │
│  │  • Left zone     (-135° to -45°)    │                  │
│  │  • Right zone    (45° to 135°)      │                  │
│  │  • Emergency stop when trapped       │                  │
│  └──────────────┬───────────────────────┘                  │
│                 │                        ▲                   │
│                 │ /cmd_vel              │ /cmd_vel_teleop   │
│                 │ (safe)                │ (raw)             │
└─────────────────┼────────────────────────┼──────────────────┘
                  │                        │
                  │                        │
┌─────────────────┼────────────────────────┼──────────────────┐
│                 │       CONTROL LAYER    │                  │
├─────────────────┼────────────────────────┼──────────────────┤
│                 │                        │                  │
│                 ▼                        │                  │
│         ┌──────────────┐                │                  │
│         │ Motor Driver │                │                  │
│         │              │                │                  │
│         │ Left Motor   │                │                  │
│         │ Right Motor  │        ┌───────┴─────────┐       │
│         │ Servo        │        │  Teleop Node    │       │
│         └──────────────┘        │  (Keyboard)     │       │
│                                 └─────────────────┘       │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

---

## 🎯 What Each Component Does

### **MPU6050 IMU Node**
- Reads acceleration and gyroscope
- Provides orientation for odometry
- Publishes to `/imu/mpu6050`

### **LiDAR Node**
- Reads 360° LiDAR scans
- Publishes to `/scan` (LaserScan)
- 50 Hz update rate

### **LiDAR Odometry Node**
- **Starts position at (0, 0)**
- Uses scan matching for motion estimation
- Applies triangulation with reference landmarks
- Accounts for LiDAR offset (10cm front, 16cm back, 8cm left, 10.5cm right)
- Fuses with IMU gyro for orientation
- Publishes to `/lidar_odom`

### **Obstacle Avoidance Node**
- Monitors 4 detection zones (front/back/left/right)
- Subscribes to `/cmd_vel_teleop` (raw commands from teleop)
- Checks for obstacles in relevant zones
- Blocks unsafe motions
- Publishes safe commands to `/cmd_vel` (to motors)
- Publishes `/obstacle_detected` (Bool flag)

### **Camera & Video**
- Captures video from camera
- Streams to web interface on port 8080
- Can be disabled to save CPU

### **Motor Driver**
- Receives safe commands from `/cmd_vel`
- Controls left/right motors and servo
- Provides speed feedback

---

## 🔍 Monitoring & Debugging

### Check All Active Topics
```bash
ros2 topic list
```

**Expected topics with LiDAR enabled:**
```
/scan                    # LiDAR data
/imu/mpu6050            # IMU data
/lidar_odom             # Position from (0,0)
/cmd_vel_teleop         # Raw commands from teleop
/cmd_vel                # Safe commands to motors
/obstacle_detected      # Boolean obstacle flag
/camera/image_raw       # Camera feed
```

### Monitor Odometry
```bash
ros2 topic echo /lidar_odom
```
- Should start near (0, 0, 0)
- Updates as rover moves

### Monitor Obstacles
```bash
ros2 topic echo /obstacle_detected
```
- `data: false` - Clear path
- `data: true` - Obstacle detected

### Check Command Flow
```bash
# Terminal 1: Raw commands from teleop
ros2 topic echo /cmd_vel_teleop

# Terminal 2: Safe commands after filtering
ros2 topic echo /cmd_vel
```

### Monitor LiDAR Scan
```bash
ros2 topic echo /scan --once
```

### Check Node Status
```bash
ros2 node list
```

**Expected nodes:**
```
/mpu6050_node
/lidar_node
/lidar_odometry_node
/obstacle_avoidance_node
/camera_node
/streamer_node
/motor_driver_node
/teleop_node (if launched)
```

### Check Node Info
```bash
ros2 node info /obstacle_avoidance_node
ros2 node info /lidar_odometry_node
```

---

## ⚙️ Runtime Parameter Tuning

### Adjust Obstacle Avoidance Sensitivity

**More Conservative (Safer):**
```bash
ros2 param set /obstacle_avoidance_node safety_margin 0.20
ros2 param set /obstacle_avoidance_node stop_distance 0.40
```

**More Aggressive (Tighter):**
```bash
ros2 param set /obstacle_avoidance_node safety_margin 0.10
ros2 param set /obstacle_avoidance_node stop_distance 0.20
```

**Check Current Values:**
```bash
ros2 param get /obstacle_avoidance_node safety_margin
ros2 param get /obstacle_avoidance_node stop_distance
```

**Or use the tuning tool:**
```bash
cd /home/alibaba/frr_ws
./tune_obstacle_avoidance.sh interactive
```

---

## 🧪 Testing Procedure

### 1. Launch System
```bash
# Terminal 1: Launch rover
ros2 launch frr_bringup rover_bringup.launch.py

# Terminal 2: Launch teleop
ros2 launch frr_bringup teleop_with_avoidance.launch.py
```

### 2. Verify Startup
Check logs for:
- ✅ "LiDAR connected"
- ✅ "MPU6050 initialized"
- ✅ "Calibration complete"
- ✅ "LiDAR Odometry Node started"
- ✅ "Obstacle Avoidance Node started"

### 3. Test Odometry
```bash
# Terminal 3: Monitor position
ros2 topic echo /lidar_odom
```
- Position should start near (0, 0, 0)
- Move rover forward slightly
- X coordinate should increase

### 4. Test Obstacle Detection

**Place object in front:**
```bash
ros2 topic echo /obstacle_detected
# Should show: data: true
```

**Try to move forward (with teleop):**
- Rover should NOT move
- Safe command should be zero

**Remove object:**
```bash
ros2 topic echo /obstacle_detected
# Should show: data: false
```

**Try to move forward:**
- Rover should move normally

### 5. Test All Directions
- Front obstacle → blocks forward
- Back obstacle → blocks reverse
- Left obstacle → blocks left turn
- Right obstacle → blocks right turn

---

## 🚨 Troubleshooting

### LiDAR Not Detected
```bash
ls /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0
```

### IMU Not Responding
```bash
sudo i2cdetect -y 1
# Should see 0x68
```

### Obstacle Avoidance Not Working
- Check if LiDAR is enabled: `enable_lidar:=true`
- Verify teleop is using remapping: `teleop_with_avoidance.launch.py`
- Check topic flow:
  ```bash
  ros2 topic echo /cmd_vel_teleop  # Should have values
  ros2 topic echo /cmd_vel         # Should be filtered
  ```

### Position Drifts Too Much
- Ensure environment has features (walls, objects)
- Check reference landmarks initialized (in logs)
- Triangulation needs distinct features to work

### Nodes Not Starting
```bash
# Check build
colcon build --packages-select frr_sensors frr_bringup

# Source workspace
source install/setup.bash

# Check for errors
ros2 node list
```

---

## 📋 Quick Reference

### Launch Options Matrix

| Command | LiDAR | Video | Obstacle Avoidance |
|---------|-------|-------|-------------------|
| `rover_bringup.launch.py` | ✅ | ✅ | ✅ |
| `rover_bringup.launch.py enable_video_stream:=false` | ✅ | ❌ | ✅ |
| `rover_bringup.launch.py enable_lidar:=false` | ❌ | ✅ | ❌ |
| `rover_bringup.launch.py enable_lidar:=false enable_video_stream:=false` | ❌ | ❌ | ❌ |

### Teleop Options

| Command | Obstacle Avoidance |
|---------|-------------------|
| `teleop_with_avoidance.launch.py` | ✅ |
| `teleop_with_avoidance.launch.py no_remap:=true` | ❌ |
| `teleop.launch.py` | ❌ |

---

## 🎉 Summary

**Everything is now integrated into ONE main launch file!**

```bash
# Start everything:
ros2 launch frr_bringup rover_bringup.launch.py

# In another terminal, start teleop:
ros2 launch frr_bringup teleop_with_avoidance.launch.py
```

**That's it! Your rover has:**
- ✅ LiDAR scanning
- ✅ Position tracking from (0,0)
- ✅ Triangulation for accuracy
- ✅ 4-corner obstacle avoidance
- ✅ Emergency stop protection
- ✅ Video streaming
- ✅ Full motor control

**All configurable with simple parameters!**

---

**Created:** November 14, 2025  
**Status:** ✅ Fully Integrated and Ready to Use
