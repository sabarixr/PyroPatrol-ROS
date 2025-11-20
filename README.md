# 🚒 Fire Fighter Rover ROS 2 System

> Version control initialized (git) – this README now reflects the structured repository layout and workflow.

A complete ROS 2 system for a Raspberry Pi 4 based fire fighter rover with camera, IMU, and motor control capabilities.

## 📂 Repository Structure

```
frr_ws/
├── src/
│   ├── frr_bringup/          # Launch files aggregating system nodes
│   ├── frr_control/          # Motor driver, teleop control logic
│   ├── frr_navigation/       # (Planned / WIP) navigation and obstacle logic
│   ├── frr_sensors/          # IMU, camera, lidar and sensor interfaces
│   ├── frr_video/            # Video streaming (Flask MJPEG)
│   ├── ydlidar_ros2_driver/  # Vendor ROS2 driver for YDLidar
│   └── YDLidar-SDK/          # Low-level SDK sources
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
- **MMA8452 Accelerometer** → tilt/fall detection and velocity estimation
- **L298N Motor Driver** → differential drive with PWM speed control
- **Teleoperation** → keyboard control for manual driving

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

## 📦 ROS 2 Packages

### 1️⃣ `frr_sensors`
- `imu_node` - MMA8452 I²C reader with fall detection
- `camera_node` - OpenCV camera with ArUco marker detection

### 2️⃣ `frr_control` 
- `motor_driver_node` - L298N PWM controller with differential drive
- `teleop_node` - Keyboard teleoperation interface

### 3️⃣ `frr_video`
- `streamer_node` - Flask-based MJPEG video streaming server

### 4️⃣ `frr_bringup`
- Launch files to start all nodes at once

## 🔄 ROS 2 Topics

| Topic                  | Type                  | Publisher    | Subscriber     | Purpose                         |
|------------------------|----------------------|--------------|----------------|---------------------------------|
| `/cmd_vel`             | `geometry_msgs/Twist` | teleop       | motor_driver   | Linear/angular velocity         |
| `/imu/data_raw`        | `sensor_msgs/Imu`     | imu_node     | —              | Raw accel + integrated velocity |
| `/rover/fall_detected` | `std_msgs/Bool`       | imu_node     | —              | Fall/tip event                  |
| `/camera/image_raw`    | `sensor_msgs/Image`   | camera_node  | video_streamer | Captured frames                 |
| `/aruco/pose`          | `geometry_msgs/Pose`  | camera_node  | —              | 3D pose of detected ArUco markers (position + orientation) |
| `/motor_status`        | `std_msgs/String`     | motor_driver | —              | Motor diagnostics               |

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

# Launch all nodes (with video streaming)
ros2 launch frr_bringup rover_bringup.launch.py

# Launch without video streaming (lower CPU usage)
ros2 launch frr_bringup rover_bringup.launch.py enable_video_stream:=false
```

### Manual Control
```bash
# In a separate terminal
ros2 launch frr_bringup teleop.launch.py

# Or run directly
ros2 run frr_control teleop_node
```

### Individual Node Testing
```bash
# Test IMU
ros2 run frr_sensors imu_node

# Test camera
ros2 run frr_sensors camera_node

# Test motor driver
ros2 run frr_control motor_driver_node

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
ros2 topic echo /imu/data_raw

# Monitor fall detection
ros2 topic echo /rover/fall_detected

# Monitor ArUco detection
ros2 topic echo /aruco/pose

# Monitor motor status
ros2 topic echo /motor_status
```

### Node Information
```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /imu_node
ros2 node info /camera_node
ros2 node info /motor_driver_node
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

Fire Fighter Rover Development Team
