# 📚 Fire-Fighting Robot Documentation Index

## 🚀 Getting Started

Start here if you're new to the system:

1. **[QUICK_START.md](QUICK_START.md)** - Fast reference guide for daily operation
   - Interactive launcher usage
   - Basic commands
   - Troubleshooting quick fixes
   - Video streaming access

2. **[SETUP_CHECKLIST.md](SETUP_CHECKLIST.md)** - Complete pre-flight checklist
   - Hardware verification
   - Software installation
   - Sensor calibration
   - Performance targets

3. **[README.md](README.md)** - Original system overview
   - Manual teleoperation
   - Individual node testing
   - Traditional operation mode

## 📖 Detailed Documentation

### System Architecture
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - Complete system design
  - Data flow diagrams
  - Communication protocols
  - TF transform tree
  - File structure

### SLAM System
- **[SLAM_README.md](SLAM_README.md)** - SLAM mapping documentation
  - How SLAM works (mapping vs localization)
  - Sensor fusion details
  - Parameter tuning guide
  - Calibration procedures

## 💻 Code & Firmware

### ESP32 Firmware
- **[ESP32_FIRMWARE_TEMPLATE.ino](ESP32_FIRMWARE_TEMPLATE.ino)** - Arduino code for ESP32
  - Motor control implementation
  - Encoder reading with interrupts
  - MPU6050 IMU integration
  - Serial command protocol
  - JSON telemetry format

### Launch Scripts
- **[launch_slam.sh](launch_slam.sh)** - Interactive SLAM launcher
  - Mapping mode
  - Localization mode
  - Map saving
  - Sensor diagnostics

## 🎯 Quick Reference Cards

### Launch Commands

**SLAM System (Recommended):**
```bash
./launch_slam.sh
```

**Manual Teleoperation:**
```bash
ros2 launch frr_bringup rover_bringup.launch.py
```

**Individual Node Testing:**
```bash
# Sensors
ros2 run frr_sensors mpu6050_node
ros2 run frr_sensors camera_node
ros2 run frr_sensors sensor_fusion_node

# Control
ros2 run frr_control esp32_bridge_node
ros2 run frr_navigation autonomous_firebot_node

# SLAM
ros2 launch frr_bringup slam_mapping.launch.py
ros2 launch frr_bringup slam_localization.launch.py
```

### Key Topics

| Topic | Purpose |
|-------|---------|
| `/scan` | LIDAR data for mapping and obstacle detection |
| `/imu/mpu6050` | IMU orientation and angular velocity |
| `/odom` | Fused odometry (encoders + IMU) |
| `/map` | SLAM-built occupancy grid |
| `/cmd_vel` | Velocity commands to motors |
| `/esp32_telemetry` | Fire sensors and encoder data |
| `/aruco/pose` | ArUco marker localization |
| `/camera/image_raw` | Camera feed |

### Troubleshooting Shortcuts

```bash
# Check all sensors
./launch_slam.sh  # Option 5

# View active topics
ros2 topic list

# Test ESP32 connection
screen /dev/ttyACM0 115200

# View TF tree
ros2 run tf2_tools view_frames

# Monitor specific topic
ros2 topic echo /topic_name
```

## 📂 File Organization

```
/home/alibaba/frr_ws/
│
├── 📚 Documentation
│   ├── README.md                    # Main overview
│   ├── QUICK_START.md               # Fast reference
│   ├── SLAM_README.md               # SLAM details
│   ├── SETUP_CHECKLIST.md           # Pre-flight checks
│   ├── ARCHITECTURE.md              # System design
│   └── INDEX.md                     # This file
│
├── 💾 Code & Scripts
│   ├── launch_slam.sh               # Interactive launcher
│   ├── ESP32_FIRMWARE_TEMPLATE.ino  # ESP32 Arduino code
│   │
│   └── src/
│       ├── frr_bringup/            # Launch files
│       │   └── launch/
│       │       ├── slam_mapping.launch.py
│       │       └── slam_localization.launch.py
│       │
│       ├── frr_sensors/            # Sensor nodes
│       │   └── frr_sensors/
│       │       ├── robot_tf_publisher.py
│       │       ├── sensor_fusion_node.py
│       │       ├── mpu6050_node.py
│       │       └── camera_node.py
│       │
│       ├── frr_control/            # Control nodes
│       │   └── frr_control/
│       │       └── esp32_bridge_node.py
│       │
│       └── frr_navigation/         # Navigation
│           └── frr_navigation/
│               └── autonomous_firebot_node.py
│
└── 🗺️ Data
    └── maps/                        # Saved SLAM maps
        ├── *.posegraph
        ├── *.yaml
        └── *.data
```

## 🎓 Learning Path

### Beginner (First Time Setup)
1. Read [SETUP_CHECKLIST.md](SETUP_CHECKLIST.md)
2. Upload ESP32 firmware from [ESP32_FIRMWARE_TEMPLATE.ino](ESP32_FIRMWARE_TEMPLATE.ino)
3. Follow [QUICK_START.md](QUICK_START.md) to launch system
4. Run `./launch_slam.sh` → Option 5 to verify sensors

### Intermediate (Daily Operation)
1. Use [QUICK_START.md](QUICK_START.md) as reference
2. Launch with `./launch_slam.sh`
3. Create maps in mapping mode (Option 1)
4. Navigate with localization mode (Option 2)

### Advanced (System Understanding)
1. Study [ARCHITECTURE.md](ARCHITECTURE.md) for system design
2. Read [SLAM_README.md](SLAM_README.md) for SLAM details
3. Modify parameters for your environment
4. Extend functionality by adding nodes

## 🔧 Maintenance

### Regular Checks
- [ ] Battery voltage (low voltage = erratic behavior)
- [ ] Encoder connections (GPIO 13, 14)
- [ ] LIDAR spinning freely
- [ ] Camera lens clean
- [ ] MQ sensor warmup time (30-60 sec)

### Periodic Calibration
- [ ] Wheel diameter and wheelbase measurements
- [ ] Fire sensor thresholds
- [ ] Camera intrinsic parameters (for accurate ArUco)
- [ ] IMU vibration filtering parameters

### Performance Monitoring
```bash
# Check update rates
ros2 topic hz /scan       # Should be ~10 Hz
ros2 topic hz /odom       # Should be ~10 Hz
ros2 topic hz /imu/mpu6050  # Should be ~10 Hz

# Check SLAM performance
ros2 topic echo /map --once  # Verify map exists

# Monitor CPU usage
htop
```

## 🆘 Help & Support

### Diagnostic Tools
- `./launch_slam.sh` → Option 5: Comprehensive sensor check
- `ros2 topic list`: See all active topics
- `ros2 node list`: See all running nodes
- `ros2 run tf2_tools view_frames`: Verify TF tree

### Common Issues
- **No map building**: Robot must be moving, check `/odom`
- **ESP32 not responding**: Check `/dev/ttyACM0` permissions
- **Poor localization**: Add ArUco markers, calibrate wheels
- **Fire not detected**: Lower thresholds, wait for sensor warmup
- **Obstacles not avoided**: Check LIDAR `/scan` data

### Where to Look
- Hardware issues → [SETUP_CHECKLIST.md](SETUP_CHECKLIST.md)
- SLAM problems → [SLAM_README.md](SLAM_README.md)
- Quick fixes → [QUICK_START.md](QUICK_START.md)
- System design → [ARCHITECTURE.md](ARCHITECTURE.md)
- ESP32 issues → [ESP32_FIRMWARE_TEMPLATE.ino](ESP32_FIRMWARE_TEMPLATE.ino)

## 📞 Quick Links

| Need | Document | Section |
|------|----------|---------|
| Launch the robot | [QUICK_START.md](QUICK_START.md) | Quick Start |
| First-time setup | [SETUP_CHECKLIST.md](SETUP_CHECKLIST.md) | Complete checklist |
| Calibrate sensors | [SLAM_README.md](SLAM_README.md) | Calibration |
| Understand system | [ARCHITECTURE.md](ARCHITECTURE.md) | System diagrams |
| Modify ESP32 code | [ESP32_FIRMWARE_TEMPLATE.ino](ESP32_FIRMWARE_TEMPLATE.ino) | Full code |
| Teleoperation keys | [README.md](README.md) | Teleoperation Controls |
| Save a map | [SLAM_README.md](SLAM_README.md) | Save Map |
| Tune SLAM | [SLAM_README.md](SLAM_README.md) | SLAM Toolbox Parameters |
| Fire thresholds | [QUICK_START.md](QUICK_START.md) | Parameter Tuning |

---

**Ready to start?** 
- New user? → [SETUP_CHECKLIST.md](SETUP_CHECKLIST.md)
- Daily operation? → [QUICK_START.md](QUICK_START.md)
- Need help? → See troubleshooting section above

**Launch command:** `./launch_slam.sh` 🚀
