# 🚒 Fire Fighter Rover ROS 2 System (No SLAM)

> **Note:** This system does **not** use SLAM or global mapping. LIDAR is used for local obstacle detection and reactive navigation only. See [ARCHITECTURE.md](ARCHITECTURE.md) for a full system diagram and data flow.

A complete ROS 2 system for a Raspberry Pi 4 based fire-fighting rover with camera, IMU, motor control, and robust local obstacle avoidance.

## 🎯 Quick Launch

**Quick Start (all nodes):**
```bash
./start_rover.sh
```
Or, to launch manually:
```bash
ros2 launch frr_bringup esp32_rover_bringup.launch.py
```
See [ARCHITECTURE.md](ARCHITECTURE.md) and [SETUP_CHECKLIST.md](SETUP_CHECKLIST.md) for full documentation.

---

## 📚 Documentation Index

| Document | Description |
|----------|-------------|
| **[README.md](README.md)** | This file - system overview and usage |
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | System architecture and data flow diagrams |
| **[SETUP_CHECKLIST.md](SETUP_CHECKLIST.md)** | Pre-flight checklist before autonomous operation |
| **[ESP32_FIRMWARE_TEMPLATE.ino](ESP32_FIRMWARE_TEMPLATE.ino)** | ESP32 Arduino firmware with MPU6050 support |

---

A complete ROS 2 system for a Raspberry Pi 4 based fire fighter rover with camera, IMU, and motor control capabilities.

## 📂 Repository Structure

```
frr_ws/
├── src/
│   ├── frr_bringup/          # Launch files aggregating system nodes
│   ├── frr_control/          # ESP32 bridge, motor control, teleop
│   ├── frr_navigation/       # Autonomous firebot logic (obstacle avoidance, fire detection)
│   ├── frr_sensors/          # IMU, camera, lidar, sensor fusion, static TFs
│   ├── frr_video/            # Video streaming (Flask MJPEG)
│   ├── ydlidar_ros2_driver/  # Vendor ROS2 driver for YDLidar
│   └── ydlidar_sdk/          # Low-level SDK sources
├── tools/                    # Utility scripts / diagnostics helpers
├── *.sh                      # Setup, launch, diagnostic shell scripts
├── test_*.py / test_*.sh     # Standalone test scripts for components
├── build/ install/ log/      # Colcon build artifacts (ignored)
└── README.md
```

## 🗃️ Git & Contribution Workflow

We maintain a readable history using logical commits:

- `chore:` toolchain / meta changes (gitignore, formatting)
- `docs:` documentation only changes
- `feat:` new node, capability or launch inclusion
- `fix:` bug fixes (sensor reading, motor timing, etc.)
- `perf:` performance or resource optimization
- `refactor:` structural code change without new features

### Branching
`main` holds stable, tested code. For new work create feature branches:
```
git checkout -b feat/<short-description>
```
Open a PR (if remote collaboration) and ensure tests or basic verification steps are described.

### Adding Remote (already initialized locally)
```
git remote add origin https://github.com/sabarixr/PyroPatrol-ROS.git
git push -u origin main
```

### Recommended Commit Order (already underway)
1. Initialize repo & add base docs (done)
2. Add operational scripts & tooling (next)
3. Add source packages under `src/`
4. Add test scripts
5. Subsequent features / fixes in isolated commits

## ✅ Current Status Snapshot

- Git repository initialized (`main` branch)
- `.gitignore` configured for ROS2/Python artifacts
- Base documentation enhanced with structure and workflow sections
- Next commit will add shell scripts & utilities

---


## 🧠 System Overview

The rover integrates:
- **Camera** → ArUco marker detection + MJPEG video streaming
- **MPU6050 IMU** → orientation and acceleration (sensor fusion)
- **YDLidar X2** → 360° LIDAR for local obstacle detection
- **L298N Motor Driver** → differential drive with PWM speed control (via ESP32)
- **ESP32** → Motor control, encoders, fire sensors, IMU, and telemetry
- **Teleoperation** → keyboard/manual control

## ⚙️ Hardware Connections

### L298N Motor Driver
| Signal | GPIO | Function                 |
|--------|------|--------------------------|
| ENA    | 17   | PWM (left motor speed)   |
| IN1    | 27   | Motor A Dir1             |
| IN2    | 22   | Motor A Dir2             |
| IN3    | 23   | Motor B Dir1             |
| IN4    | 24   | Motor B Dir2             |
| ENB    | 18   | PWM (right motor speed)  |
| VCC    | 5V   |                          |
| GND    | GND  |                          |

### MMA8452 Accelerometer (I²C)
| Pin  | GPIO         | Note |
|------|--------------|------|
| VCC  | 3.3V         |      |
| GND  | GND          |      |
| SDA  | 2            |      |
| SCL  | 3            |      |
| ADDR | GND (→ 0x1C) |      |

### Camera
- **CSI Camera** connected to Raspberry Pi camera port
- Accessible via `cv2.VideoCapture(0)`

## 🔄 ROS 2 Topics
Key topics used in the system:

| Topic | Type | Publishers | Subscribers | Purpose |
|-------|------|------------|-------------|---------|
| `/scan` | `sensor_msgs/LaserScan` | ydlidar_ros2_driver | autonomous_firebot_node | 2D lidar scan for obstacle avoidance |
| `/imu/mpu6050` | `sensor_msgs/Imu` | mpu6050_node | sensor_fusion_node | IMU orientation & accel |
| `/odom` | `nav_msgs/Odometry` | sensor_fusion_node | autonomous_firebot_node | Fused odometry (encoders + IMU) |
| `/camera/image_raw` | `sensor_msgs/Image` | camera_node | streamer_node | Camera frames |
| `/aruco/pose` | `geometry_msgs/PoseStamped` | camera_node | autonomous_firebot_node | ArUco marker pose |
| `/esp32_telemetry` | `std_msgs/String` | esp32_bridge_node | autonomous_firebot_node | ESP32 sensor/encoder/fire data |
| `/cmd_vel` | `geometry_msgs/Twist` | autonomous_firebot_node, teleop | esp32_bridge_node | Velocity commands to ESP32 |
- `autonomous_firebot_node.py` – Main autonomous logic (obstacle avoidance, fire detection, pathfinding)

### 6️⃣ `frr_bringup`
### 4️⃣ `frr_video`
- `streamer_node.py` – MJPEG streaming server (subscribes to `/camera/image_raw`)


### 5️⃣ `ydlidar_ros2_driver`
- Vendor lidar driver node (publishes `/scan`)

Comprehensive topic matrix extracted from source code:
### 6️⃣ `frr_bringup`
- Launch files orchestrating all nodes for full system startup
| Topic | Type | Publishers | Subscribers | Purpose |
|-------|------|------------|-------------|---------|
| `/cmd_vel` | `geometry_msgs/Twist` | teleop nodes, obstacle_avoidance_node (corrected) | motor_controller_node, obstacle_avoidance_node (input), motor_simple | Base velocity commands |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | teleop_node_clean (variant) | (internal remap) | Intermediate teleop channel |
| `/camera_tilt` | `std_msgs/Float64` | teleop nodes, camera_servo_node | motor_controller_node (servo control) | Camera servo angle |
| `/imu/data_raw` | `sensor_msgs/Imu` | legacy `imu_node` | odometry/other diagnostic tools | Raw IMU (basic) |
| `/imu/mpu6050` | `sensor_msgs/Imu` | robust MPU6050 nodes | odometry_node, lidar_odometry_node | Filtered IMU orientation & accel |
| `/imu/mpu6050_raw` | `std_msgs/Float32MultiArray` | robust MPU6050 nodes | diagnostics | Raw accelerometer + gyro arrays |
| `/rover/fall_detected` | `std_msgs/Bool` | imu_node | safety monitors | Rover tip/fall event |
| `/camera/image_raw` | `sensor_msgs/Image` | camera_node, simple_camera_node | streamer_node / optimized_streamer_node | Uncompressed camera frames |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | camera_node (if compression enabled) | optimized_streamer_node | Compressed camera frames |
| `/aruco/pose` | `geometry_msgs/Pose` | camera nodes | navigation / localization consumers | 3D ArUco marker pose |
| `/scan` | `sensor_msgs/LaserScan` | lidar_node, ydlidar_ros2_driver | obstacle_avoidance_node, lidar_odometry_node | 2D lidar scan |
| `/lidar/raw` | `std_msgs/Float32MultiArray` | lidar_node | diagnostics | Raw distance/intensity array |
| `/point_cloud` | `sensor_msgs/PointCloud` | ydlidar_ros2_driver | mapping consumers | Point cloud (if enabled) |
| `/lidar_odom` | `nav_msgs/Odometry` | lidar_odometry_node | navigation stack | Odometry from lidar+IMU fusion |
| `/odom` | `nav_msgs/Odometry` | odometry_node | navigation stack, motor control tuning | Fused odometry (IMU + velocity integration) |
| `/velocity_error` | `geometry_msgs/Vector3` | odometry_node | speed_controller_node | Error vector between desired/measured velocity |
| `/speed_correction` | `std_msgs/Float32MultiArray` | odometry_node | motor_controller_node | Wheel speed correction factors |
| `/obstacle_detected` | `std_msgs/Bool` | obstacle_avoidance_node | teleop_node_clean (optional), high-level logic | Indicates obstacle ahead |
| `/motor_status` | `std_msgs/String` | motor_controller_node | diagnostics | Motor/servo status and alerts |

## 🎯 ArUco Marker Navigation

The `/aruco/pose` topic provides **3D position and orientation** of detected ArUco markers:

**Position** (meters from camera):
- `x`: Left/right distance 
- `y`: Up/down distance
- `z`: Forward/backward distance

**Orientation** (quaternion):
- `x, y, z, w`: 3D rotation of the marker

**Use Cases:**
- 🗺️ **Navigation waypoints** - Place markers at known locations
- 📍 **Localization** - Determine rover's position relative to markers  
- 🎯 **Target tracking** - Approach specific markers autonomously
- 🏠 **Return-to-base** - Navigate back to charging/base station

**Marker Setup:** Print 4x4 ArUco markers (dictionary DICT_4X4_50) and place them at strategic locations. The system will automatically detect them and publish their 3D pose.

## 🛠️ Installation

### 1. Install System Dependencies

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop

# Install hardware interface packages
sudo apt install python3-opencv python3-smbus python3-rpi.gpio \
                 ros-humble-rclpy ros-humble-cv-bridge ros-humble-image-transport \
                 ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-std-msgs

# Install Python packages
pip3 install flask numpy
```

### 2. Build the Workspace

```bash
cd /home/alibaba/frr_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Enable I²C (for IMU)

```bash
sudo raspi-config
# Navigate to: Interfacing Options → I2C → Enable
sudo reboot
```

## 🚀 Usage


### Launch All Nodes
```bash
# Source the workspace
source /home/alibaba/frr_ws/install/setup.bash

# Launch all nodes (recommended)
ros2 launch frr_bringup esp32_rover_bringup.launch.py
```


### Manual Control
```bash
# In a separate terminal
ros2 run frr_control teleop_node
```


### Individual Node Testing
```bash
# Test IMU
ros2 run frr_sensors mpu6050_node

# Test camera
ros2 run frr_sensors camera_node

# Test ESP32 bridge
ros2 run frr_control esp32_bridge_node

# Test video streamer
ros2 run frr_video streamer_node
```

## 🎮 Teleoperation Controls

```
Moving around:
   q    w    e
   a    s    d  
   z    x    c

w/x : forward/backward
a/d : turn left/right
q/e : forward diagonal movement
z/c : backward diagonal movement
s   : stop

Speed control:
t/g : increase/decrease linear speed
r/f : increase/decrease angular speed

CTRL-C to quit
```

## 🌐 Video Streaming

Once the system is running, access the video stream at:
- **Web interface**: `http://<raspberry-pi-ip>:8080/`
- **Direct MJPEG stream**: `http://<raspberry-pi-ip>:8080/stream.mjpg`


## 🔍 Monitoring

### View Topics
```bash
# List all active topics
ros2 topic list

# Monitor IMU data
ros2 topic echo /imu/mpu6050

# Monitor ArUco detection
ros2 topic echo /aruco/pose

# Monitor ESP32 telemetry
ros2 topic echo /esp32_telemetry
```

### Node Information
```bash
# List running nodes
ros2 node list

ros2 node info /mpu6050_node
ros2 node info /camera_node
ros2 node info /esp32_bridge_node
```

## 📊 Performance Optimization

The system is designed to run efficiently on Raspberry Pi 4:

- **IMU**: 50 Hz sampling rate
- **Camera**: 640×480 @ 30 FPS (high-quality FPV)
- **Video**: Adjustable JPEG quality (default 80%)
- **Motor Control**: 1 kHz PWM frequency
- **Safety**: 1-second timeout for motor commands

### CPU Usage Tips
- Disable video streaming when not needed: `enable_video_stream:=false`
- Reduce camera FPS: modify `fps` parameter in camera_node
- Lower video quality: adjust `jpeg_quality` parameter in streamer_node

## 🚨 Safety Features

- **Fall Detection**: Automatic detection when rover tips over (Z-accel < 0.3g)
- **Motor Timeout**: Motors automatically stop if no command received for 1 second
- **Error Handling**: Graceful degradation when sensors fail
- **GPIO Cleanup**: Proper cleanup on node shutdown

## 🔧 Troubleshooting

### IMU Issues
```bash
# Check I²C devices
sudo i2cdetect -y 1
# Should show device at address 0x1C
```

### Camera Issues
```bash
# Test camera directly
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera working:', cap.isOpened())"
```

### GPIO Permissions
```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER
# Logout and login again
```

### Build Issues
```bash
# Clean build
cd /home/alibaba/frr_ws
rm -rf build install log
colcon build --symlink-install
```

## 📝 License

Apache License 2.0

## 👨‍💻 Author

[sabarixr](https://github.com/sabarixr)
