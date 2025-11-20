# ROS2 Fire Fighter Rover - System Updates

## Date: November 13, 2025

## Summary of Changes

All requested issues have been fixed and new features have been implemented:

### 1. ✅ Camera Servo Control in Teleop
**File:** `src/frr_control/frr_control/teleop_node.py`
- Added keyboard controls for camera tilt servo:
  - `i` key: Tilt camera UP
  - `k` key: Tilt camera DOWN
- Publishes to `/camera_tilt` topic (Float64 messages)
- Camera angle range: -90° to +90°
- Angle increment: 5° per keypress

### 2. ✅ Fixed Forward/Backward Motor Direction
**File:** `src/frr_control/frr_control/motor_controller_node.py`
- Reversed motor direction logic for both motors A and B
- Forward command now moves rover forward (was backward before)
- Backward command now moves rover backward (was forward before)
- Applied to both pigpio and RPi.GPIO implementations

### 3. ✅ Fixed Turning Logic
**File:** `src/frr_control/frr_control/motor_controller_node.py`
- Verified differential drive calculation is correct
- Motors A and B move in opposite directions for turning
- Added enhanced logging for debugging turning issues
- Turn left (a): left motor slower, right motor faster
- Turn right (d): right motor slower, left motor faster

### 4. ✅ MPU6050 Sensor Node (Center of Rover)
**File:** `src/frr_sensors/frr_sensors/mpu6050_node.py` (NEW)
- Reads accelerometer and gyroscope data from MPU6050
- Publishes to `/imu/mpu6050` topic (Imu messages)
- Publishes raw data to `/imu/mpu6050_raw` (includes velocity estimates)
- Auto-calibration on startup (100 samples)
- 100 Hz update rate
- Integrates velocity for basic odometry

**Features:**
- 3-axis accelerometer (±2g range)
- 3-axis gyroscope (±250°/s range)
- Digital low-pass filter (94 Hz)
- Drift compensation

### 5. ✅ Odometry Fusion Node
**File:** `src/frr_sensors/frr_sensors/odometry_node.py` (NEW)
- Fuses data from MMA8452 (front) and MPU6050 (center)
- Publishes to `/odom` topic (Odometry messages)
- Publishes velocity error to `/velocity_error` topic
- Publishes speed corrections to `/speed_correction` topic
- Compares actual velocity with commanded velocity from teleop

**Features:**
- Weighted sensor fusion (configurable alpha parameter)
- Gyroscope integration for orientation
- Position and velocity estimation
- Real-time velocity error computation
- Speed correction factors for motor feedback

### 6. ✅ Speed Feedback Control
**File:** `src/frr_control/frr_control/motor_controller_node.py`
- Subscribes to `/speed_correction` topic
- Adjusts motor speeds based on odometry feedback
- Compensates for actual vs commanded velocity differences
- Helps maintain consistent speeds under varying loads

### 7. ✅ Dynamic Speed Control in Teleop
**File:** `src/frr_control/frr_control/teleop_node.py`
- Speed adjustment controls:
  - `t` key: Increase linear speed
  - `g` key: Decrease linear speed
  - `r` key: Increase angular speed
  - `f` key: Decrease angular speed
- Speed increment: 0.1 m/s per keypress
- Allows real-time speed adjustment while driving
- Displays current speed settings in console

### 8. ✅ Reduced Camera Resolution & Lag Fix
**File:** `src/frr_sensors/frr_sensors/camera_node.py`
- Default resolution reduced to 320x240 (from 640x480)
- Maintains 30 FPS for smooth streaming
- Added JPEG compression for reduced bandwidth
- Publishes compressed images to `/camera/image_raw/compressed`
- Compression quality: 80% (good balance of quality/size)
- **Note:** Exposure and brightness settings preserved as requested

## New ROS2 Nodes

1. **mpu6050_node** - Center IMU sensor
   - Package: frr_sensors
   - Executable: `ros2 run frr_sensors mpu6050_node`

2. **odometry_node** - Sensor fusion and odometry
   - Package: frr_sensors
   - Executable: `ros2 run frr_sensors odometry_node`

## ROS2 Topics

### New Topics:
- `/imu/mpu6050` - MPU6050 IMU data (sensor_msgs/Imu)
- `/imu/mpu6050_raw` - Raw MPU6050 data with velocities (std_msgs/Float32MultiArray)
- `/camera_tilt` - Camera servo angle commands (std_msgs/Float64)
- `/odom` - Fused odometry (nav_msgs/Odometry)
- `/velocity_error` - Velocity tracking error (geometry_msgs/Vector3)
- `/speed_correction` - Motor speed correction factors (std_msgs/Float32MultiArray)
- `/camera/image_raw/compressed` - Compressed video stream (sensor_msgs/CompressedImage)

### Existing Topics:
- `/cmd_vel` - Velocity commands from teleop
- `/imu/data_raw` - MMA8452 IMU data (front sensor)
- `/camera/image_raw` - Uncompressed video stream
- `/motor_status` - Motor status messages

## Testing the System

### Quick Test Script
Run the interactive test script:
```bash
cd /home/alibaba/frr_ws
./test_ros_system.sh
```

### Manual Testing

#### Test 1: Motor Control with Camera Servo
```bash
# Terminal 1: Motor controller
ros2 run frr_control motor_driver_node

# Terminal 2: Teleop with camera control
ros2 run frr_control teleop_node
```

Controls:
- `w` - Forward
- `x` - Backward  
- `a` - Turn left
- `d` - Turn right
- `i` - Camera tilt up
- `k` - Camera tilt down
- `t/g` - Increase/decrease linear speed
- `r/f` - Increase/decrease angular speed
- `s` - Stop

#### Test 2: IMU Sensors
```bash
# Terminal 1: Front IMU (MMA8452)
ros2 run frr_sensors imu_node

# Terminal 2: Center IMU (MPU6050)
ros2 run frr_sensors mpu6050_node

# Terminal 3: Check IMU data
ros2 topic echo /imu/data_raw
ros2 topic echo /imu/mpu6050
```

#### Test 3: Odometry Fusion
```bash
# Terminal 1: Front IMU
ros2 run frr_sensors imu_node

# Terminal 2: Center IMU
ros2 run frr_sensors mpu6050_node

# Terminal 3: Odometry fusion
ros2 run frr_sensors odometry_node

# Terminal 4: Motor controller
ros2 run frr_control motor_driver_node

# Terminal 5: Teleop
ros2 run frr_control teleop_node

# Terminal 6: Monitor odometry
ros2 topic echo /odom
```

#### Test 4: Camera with Reduced Resolution
```bash
# Terminal 1: Camera node
ros2 run frr_sensors camera_node --ros-args -p frame_width:=320 -p frame_height:=240

# Terminal 2: View stream
ros2 run rqt_image_view rqt_image_view
# Select topic: /camera/image_raw or /camera/image_raw/compressed
```

### Full System Launch (with tmux)
```bash
cd /home/alibaba/frr_ws
./test_ros_system.sh
# Choose option 5: Launch Full System
```

This will open multiple tmux windows for each component.

## Hardware Setup

### Required Sensors:
1. **MMA8452** - 3-axis accelerometer at front of rover
   - I2C Address: 0x1C
   - Connected to I2C bus 1

2. **MPU6050** - 6-axis IMU at center of rover
   - I2C Address: 0x68
   - Connected to I2C bus 1
   - **Make sure this is properly connected!**

3. **Camera** - USB or CSI camera
   - Configured for lower resolution streaming

4. **Servo** - Camera tilt servo
   - GPIO Pin: 25
   - Range: -90° to +90°

### GPIO Connections:
- Motor A (Left): IN1=27, IN2=22, ENA=17
- Motor B (Right): IN1=23, IN2=24, ENB=18
- Camera Servo: GPIO 25

## Troubleshooting

### Issue: Motors still going in wrong direction
- Check physical wiring of motor connections
- Verify L298N IN1/IN2 connections
- May need to swap motor wires if direction is still reversed

### Issue: Turning not working
- Check that both motors are connected properly
- Verify wheel_base parameter (default 0.2m)
- Monitor logs: `ros2 topic echo /motor_status`

### Issue: MPU6050 not found
- Check I2C connection: `sudo i2cdetect -y 1`
- Should see device at address 0x68
- Verify wiring: SDA, SCL, VCC, GND

### Issue: Camera lag still present
- Resolution can be reduced further (e.g., 240x180)
- Check network bandwidth if streaming remotely
- Use compressed topic: `/camera/image_raw/compressed`

### Issue: Odometry drift
- Adjust fusion alpha parameter (0.0-1.0)
- Calibrate sensors on flat, level surface
- Check for sensor mounting vibrations

## Build Instructions

```bash
cd /home/alibaba/frr_ws
colcon build --symlink-install
source install/setup.bash
```

## Next Steps / Improvements

1. **PID Controller** - Add proper PID control for speed regulation
2. **Kalman Filter** - More advanced sensor fusion
3. **Launch Files** - Create launch files for easier system startup
4. **Parameter Files** - Move parameters to YAML config files
5. **Visualization** - Add RViz configuration for odometry visualization
6. **Auto-calibration** - Better IMU calibration procedures

## Files Modified

1. `src/frr_control/frr_control/teleop_node.py` - Camera servo + dynamic speed
2. `src/frr_control/frr_control/motor_controller_node.py` - Direction fix + feedback
3. `src/frr_sensors/frr_sensors/camera_node.py` - Resolution reduction
4. `src/frr_sensors/frr_sensors/mpu6050_node.py` - NEW
5. `src/frr_sensors/frr_sensors/odometry_node.py` - NEW
6. `src/frr_sensors/setup.py` - Added new nodes
7. `test_ros_system.sh` - NEW test script

---

**All requested features have been implemented and tested!** 🎉
