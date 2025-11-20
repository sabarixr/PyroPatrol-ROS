# Fire Fighter Rover - Launch Guide

## Quick Start

### 1. Launch All Rover Systems (Single Command)
```bash
source install/setup.bash
ros2 launch frr_bringup rover_bringup.launch.py
```

This starts:
- **imu_node** - Front accelerometer (MMA8452)
- **mpu6050_node** - Center IMU (MPU6050 accelerometer + gyroscope)
- **odometry_node** - Fuses both IMUs for velocity/position estimation
- **camera_node** - Camera with optimized 320x240 resolution
- **motor_driver_node** - Motor controller with speed feedback
- **streamer_node** - Video streaming server on port 8080

### 2. Launch Teleop Control (Separate Terminal)
```bash
source install/setup.bash
ros2 launch frr_bringup teleop.launch.py
```

Or run directly:
```bash
ros2 run frr_control teleop_node
```

## Teleop Controls

### Movement
- `w` - Forward
- `x` - Backward  
- `a` - Turn left
- `d` - Turn right
- `q` - Forward + left
- `e` - Forward + right
- `z` - Backward + left
- `c` - Backward + right
- `s` - Stop

### Speed Control
- `t` / `g` - Increase/decrease linear speed limit
- `r` / `f` - Increase/decrease angular speed limit
- `1-9` - Set speed to 10%-90% (1=10%, 2=20%, etc.)
- `0` - Set speed to 100%

### Camera Control
- `i` - Tilt camera up
- `k` - Tilt camera down

### Exit
- `Ctrl+C` - Quit teleop

## View Video Stream

Open in browser:
```
http://<raspberry-pi-ip>:8080/stream.mjpg
```

## Launch Options

### Disable Video Streaming (Save CPU)
```bash
ros2 launch frr_bringup rover_bringup.launch.py enable_video_stream:=false
```

## Individual Nodes

If you need to run nodes separately:

```bash
# Front IMU
ros2 run frr_sensors imu_node

# Center IMU
ros2 run frr_sensors mpu6050_node

# Odometry Fusion
ros2 run frr_sensors odometry_node

# Camera
ros2 run frr_sensors camera_node

# Motor Controller
ros2 run frr_control motor_driver_node

# Teleop
ros2 run frr_control teleop_node

# Video Streamer
ros2 run frr_video streamer_node
```

## Topics

### Published
- `/cmd_vel` - Velocity commands (from teleop)
- `/camera_tilt` - Camera servo angle
- `/imu/data_raw` - Front accelerometer data (MMA8452)
- `/imu/mpu6050` - Center IMU data (MPU6050)
- `/imu/mpu6050_raw` - Raw MPU6050 data with velocity estimates
- `/odom` - Odometry (position, velocity)
- `/velocity_error` - Difference between commanded and actual velocity
- `/speed_correction` - Speed correction factors for motors
- `/camera/image_raw` - Camera images
- `/camera/image_raw/compressed` - Compressed JPEG images (optimized)
- `/motor_status` - Motor status

### Subscribed
- Motor controller subscribes to `/cmd_vel`, `/camera_tilt`, `/speed_correction`
- Odometry subscribes to `/imu/data_raw`, `/imu/mpu6050`, `/cmd_vel`

## New Features

### ✅ Camera Servo Control
- Control camera tilt angle using `i` and `k` keys in teleop

### ✅ Fixed Motor Directions
- Forward now moves forward, backward moves backward

### ✅ Improved Turning
- Motors move in opposite directions for proper turning

### ✅ MPU6050 Integration
- Added center IMU for better motion tracking
- Provides accelerometer + gyroscope data

### ✅ Odometry Fusion
- Combines front (MMA8452) and center (MPU6050) sensors
- Estimates rover position and velocity
- Computes velocity errors for speed feedback

### ✅ Speed Feedback Control
- Motor speeds automatically adjust based on actual vs commanded velocity
- Uses odometry data to maintain target speeds

### ✅ Dynamic Speed Control
- Press number keys (0-9) to instantly set speed percentage
- Much faster than incrementing with t/g keys

### ✅ Optimized Camera
- Reduced resolution to 320x240 for less lag
- JPEG compression for efficient streaming
- Maintains perfect exposure/brightness settings

## Troubleshooting

### Permission Issues
```bash
sudo ./fix_permissions.sh
```

### I2C Not Working
```bash
sudo raspi-config
# Enable I2C in Interface Options
sudo reboot
```

### Camera Not Found
```bash
ls /dev/video*
# Should show /dev/video0
```

### Build Errors
```bash
cd ~/frr_ws
colcon build --symlink-install
source install/setup.bash
```
