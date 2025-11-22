# 🔥 Fire-Fighting Robot with SLAM - Complete System

## 🎯 Quick Start

### Option 1: Using the Interactive Launcher (Recommended)
```bash
cd /home/alibaba/frr_ws
./launch_slam.sh
```

Then select:
- **1** for mapping mode (first time use)
- **2** for localization with existing map
- **3** to save your map
- **5** to check sensor status

### Option 2: Manual Launch

**Mapping Mode:**
```bash
cd /home/alibaba/frr_ws
source install/setup.bash
ros2 launch frr_bringup slam_mapping.launch.py
```

**Localization Mode:**
```bash
cd /home/alibaba/frr_ws
source install/setup.bash
ros2 launch frr_bringup slam_localization.launch.py
```

## 📦 What's Included

### Nodes Running:
1. **robot_tf_publisher** - Publishes static transforms (base_link, laser, imu_link, camera_link, wheels)
2. **ydlidar_ros2_driver** - YDLidar X2 360° LIDAR
3. **mpu6050_node** - MPU6050 IMU with vibration filtering
4. **sensor_fusion_node** - Fuses encoder odometry + IMU orientation
5. **camera_node** - Camera with ArUco marker detection
6. **esp32_bridge_node** - Serial communication with ESP32
7. **slam_toolbox** - Online SLAM mapping/localization
8. **streamer_node** - Video streaming server
9. **autonomous_firebot_node** - Autonomous fire-fighting navigation

### Published Topics:
- `/scan` - LIDAR data (sensor_msgs/LaserScan)
- `/imu/mpu6050` - IMU orientation (sensor_msgs/Imu)
- `/odom` - Fused odometry (nav_msgs/Odometry)
- `/map` - Occupancy grid map (nav_msgs/OccupancyGrid)
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/esp32_telemetry` - Fire sensors + encoders (JSON string)
- `/aruco/pose` - ArUco marker poses
- `/camera/image_raw` - Camera feed

### TF Tree:
```
map
 └─ odom (from SLAM Toolbox)
     └─ base_link (from sensor_fusion_node)
         ├─ base_footprint
         ├─ laser (LIDAR)
         ├─ imu_link (MPU6050)
         ├─ camera_link
         ├─ left_wheel
         └─ right_wheel
```

## 🔧 Hardware Setup

### ESP32 Connections:
- **Motors**: GPIO 27, 22, 23, 24, 17, 18 (to L298N)
- **Encoders**: GPIO 13 (left), GPIO 14 (right)
- **Fire Sensors**: Analog pins for MQ2, MQ5, flame, temperature
- **Water Pump**: GPIO for relay control
- **Serial**: USB to Raspberry Pi (/dev/ttyACM0, 115200 baud)

### Raspberry Pi Connections:
- **MPU6050**: I2C (SDA, SCL)
- **Camera**: USB or CSI
- **YDLidar X2**: USB (/dev/ydlidar)
- **Servo**: GPIO 25 (camera tilt)

## 🗺️ How SLAM Works

### Mapping Mode (slam_mapping.launch.py):
1. Robot drives around environment (manually or autonomously)
2. LIDAR scans create 2D map
3. Sensor fusion provides accurate odometry (encoders + IMU)
4. SLAM Toolbox builds map and localizes simultaneously
5. ArUco markers improve localization accuracy
6. Save map when satisfied with coverage

### Localization Mode (slam_localization.launch.py):
1. Loads pre-built map
2. Uses LIDAR + odometry to localize within map
3. Enables precise navigation in known environment
4. ArUco markers provide additional localization checkpoints

## 🤖 Autonomous Operation

### 3-Level Priority System:

**Priority 1: Obstacle Avoidance** (Highest)
- Monitors LIDAR in 3 sectors: front, left, right
- If obstacle < 0.5m in front: STOP and turn away
- Always active, overrides all other behaviors

**Priority 2: Fire Detection**
- Triggered when:
  - MQ2 > 600 (smoke)
  - MQ5 > 550 (gas)  
  - Temperature rise > 2°C
  - Flame sensor detects IR
- Actions:
  - Move toward fire source
  - Activate water pump when close
  - Continue until fire extinguished

**Priority 3: Search Pattern** (Lowest)
- Random walk when no obstacles or fire
- Explores environment systematically
- Builds map during search

## 📊 Monitoring

### Video Stream:
```
http://[raspberry-pi-ip]:8080
```

### Check Sensor Status:
```bash
# Using the launcher
./launch_slam.sh  # Option 5

# Manual checks
ros2 topic echo /scan --once
ros2 topic echo /imu/mpu6050 --once
ros2 topic echo /odom --once
ros2 topic echo /esp32_telemetry --once
```

### View Map Building:
```bash
# On PC with ROS 2 Humble
export ROS_DOMAIN_ID=0
rviz2

# Add these displays:
# - Map (/map)
# - LaserScan (/scan)
# - TF (all frames)
# - Camera (/camera/image_raw)
# - Path (/slam_toolbox/graph_visualization)
```

### Save Map:
```bash
# Using the launcher
./launch_slam.sh  # Option 3

# Manual save
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/alibaba/frr_ws/maps/my_map'}}"
```

## 🎛️ Parameter Tuning

### Sensor Fusion (`sensor_fusion_node`):
```python
'wheel_diameter': 0.065,        # Measure your wheels (meters)
'wheel_base': 0.2,              # Distance between wheels (meters)
'encoder_ticks_per_rev': 20,    # Count encoder ticks
```

### SLAM Toolbox:
```python
'resolution': 0.05,                    # Map cell size (meters)
'max_laser_range': 12.0,               # LIDAR max range
'minimum_travel_distance': 0.2,        # Min movement for scan match
'minimum_travel_heading': 0.2,         # Min rotation for scan match
'loop_search_maximum_distance': 3.0,   # Loop closure radius
'do_loop_closing': True,               # Enable loop closure
```

### Fire Detection:
```python
'fire_threshold_mq2': 600,      # Smoke threshold
'fire_threshold_mq5': 550,      # Gas threshold  
'temp_rise_threshold': 2.0,     # Temperature change (°C)
'min_obstacle_distance': 0.5,   # Safety distance (meters)
```

## 🐛 Troubleshooting

### Map Not Building
**Symptoms**: Robot drives but map stays empty
**Solutions**:
1. Check LIDAR: `ros2 topic echo /scan`
2. Verify odometry: `ros2 topic echo /odom`
3. Check TF tree: `ros2 run tf2_tools view_frames`
4. Ensure robot is moving (SLAM needs motion)

### Poor Localization
**Symptoms**: Robot position jumps around on map
**Solutions**:
1. Add ArUco markers to environment
2. Calibrate wheel parameters (diameter, base)
3. Check encoder wiring (GPIO 13, 14)
4. Verify IMU: `ros2 topic echo /imu/mpu6050`
5. Increase `minimum_travel_distance` parameter

### ESP32 Not Responding
**Symptoms**: No telemetry, motors don't move
**Solutions**:
1. Check USB connection: `ls -l /dev/ttyACM*`
2. Verify baud rate (115200)
3. Test ESP32 directly: `screen /dev/ttyACM0 115200`
4. Send test command: `DRIVE 50 50`

### LIDAR Not Detected
**Symptoms**: No `/scan` topic
**Solutions**:
1. Check USB: `ls -l /dev/ydlidar`
2. Grant permissions: `sudo chmod 666 /dev/ydlidar`
3. Verify YDLidar driver installation
4. Try manual launch: `ros2 launch ydlidar_ros2_driver x2.launch.py`

### Fire Not Detected
**Symptoms**: Robot doesn't respond to fire
**Solutions**:
1. Check sensor readings: `ros2 topic echo /esp32_telemetry`
2. Lower thresholds if needed
3. Wait for MQ sensor warmup (30-60 seconds)
4. Test each sensor individually

### Obstacles Not Avoided
**Symptoms**: Robot runs into walls
**Solutions**:
1. Check LIDAR range: `ros2 topic echo /scan --field ranges`
2. Verify `min_obstacle_distance` parameter
3. Test autonomous node: `ros2 topic echo /cmd_vel`
4. Check ESP32 receives commands

## 📁 File Structure

```
/home/alibaba/frr_ws/
├── launch_slam.sh                    # Interactive launcher script
├── SLAM_README.md                    # Detailed SLAM documentation
├── QUICK_START.md                    # This file
├── maps/                             # Saved SLAM maps
│   └── *.posegraph                   # SLAM Toolbox map files
└── src/
    ├── frr_bringup/
    │   └── launch/
    │       ├── slam_mapping.launch.py
    │       └── slam_localization.launch.py
    ├── frr_sensors/
    │   └── frr_sensors/
    │       ├── robot_tf_publisher.py
    │       ├── sensor_fusion_node.py
    │       ├── mpu6050_node.py
    │       └── camera_node.py
    ├── frr_control/
    │   └── frr_control/
    │       └── esp32_bridge_node.py
    └── frr_navigation/
        └── frr_navigation/
            └── autonomous_firebot_node.py
```

## 🔄 Typical Workflow

### First Time Setup:
1. Power on robot (ESP32 + Raspberry Pi)
2. SSH into Raspberry Pi
3. Run: `./launch_slam.sh` → Option 1 (Mapping)
4. Drive robot manually or let it explore autonomously
5. When satisfied with map: Option 3 (Save map)

### Daily Operation:
1. Power on robot
2. Run: `./launch_slam.sh` → Option 2 (Localization)
3. Select saved map
4. Robot navigates autonomously using the map
5. Finds and extinguishes fires

## 🛡️ Safety

⚠️ **IMPORTANT SAFETY NOTES:**
- Always have manual kill switch ready
- Test in controlled environment first
- Keep fire extinguisher nearby during fire tests
- Monitor battery voltage (low voltage = erratic behavior)
- MQ sensors get HOT - don't touch during operation
- Keep water pump away from electronics
- LIDAR is Class 1 laser - still don't stare into it
- Ensure adequate ventilation when testing gas sensors

## 📈 Next Steps

1. **Calibrate Robot**: Measure wheel diameter and base accurately
2. **Test Mapping**: Drive around and verify map builds correctly
3. **Save Reference Map**: Create good baseline map
4. **Place ArUco Markers**: Add markers on walls for better localization
5. **Tune Fire Thresholds**: Test with actual smoke/gas
6. **Test Autonomous Mode**: Let robot explore and verify obstacle avoidance
7. **Fire Test**: Controlled fire detection and extinguishing test

## 🆘 Support

**Check sensor status:** `./launch_slam.sh` → Option 5

**View all topics:** `ros2 topic list`

**Debug specific topic:** `ros2 topic echo /topic_name`

**See TF tree:** `ros2 run tf2_tools view_frames`

**Test ESP32:** `screen /dev/ttyACM0 115200`

**Full documentation:** See `SLAM_README.md`

---

**Built with:** ROS 2 Humble | ESP32 | MPU6050 | YDLidar X2 | SLAM Toolbox

**License:** See individual package licenses
