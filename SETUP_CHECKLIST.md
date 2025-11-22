# 🔥 Fire-Fighting Robot SLAM - Setup Checklist

Use this checklist to ensure everything is properly configured before launching.

## ✅ Hardware Checklist

### ESP32
- [ ] ESP32 powered on
- [ ] USB connected to Raspberry Pi
- [ ] Appears as `/dev/ttyACM0` (check with `ls -l /dev/ttyACM*`)
- [ ] L298N motor driver connected to GPIO 27, 22, 23, 24, 17, 18
- [ ] Left encoder connected to GPIO 13
- [ ] Right encoder connected to GPIO 14
- [ ] MQ2 smoke sensor connected to analog pin 34
- [ ] MQ5 gas sensor connected to analog pin 35
- [ ] Flame sensor connected to analog pin 32
- [ ] Temperature sensor connected to analog pin 33
- [ ] Water pump relay connected to GPIO 25
- [ ] MPU6050 IMU connected via I2C (SDA=21, SCL=22) **on ESP32**

### Raspberry Pi
- [ ] Power supply adequate (5V 3A minimum)
- [ ] SSH access working
- [ ] Camera connected (USB or CSI)
- [ ] Camera permissions set (`sudo chmod 666 /dev/video0`)
- [ ] YDLidar X2 connected via USB
- [ ] YDLidar permissions set (`sudo chmod 666 /dev/ydlidar`)
- [ ] Battery monitoring in place
- [ ] All grounds connected (ESP32, Pi, motors, sensors)

### Physical Setup
- [ ] Wheels properly attached
- [ ] Wheel diameter measured: _______ meters
- [ ] Wheel base measured: _______ meters
- [ ] Encoders aligned with wheel rotation
- [ ] LIDAR mounted at center of robot, 5cm above base
- [ ] Camera mounted 8cm forward, 10cm above base
- [ ] Weight balanced (center of gravity low)
- [ ] Water pump secure and away from electronics
- [ ] Battery secure and accessible for swapping

## ✅ Software Checklist

### ESP32 Firmware
- [ ] Arduino IDE installed
- [ ] ESP32 board support added
- [ ] ESP32_FIRMWARE_TEMPLATE.ino uploaded to ESP32
- [ ] Serial monitor shows "ESP32 Fire-Fighting Robot Ready"
- [ ] Test commands work:
  - [ ] `DRIVE 50 50` → motors spin
  - [ ] `STOP` → motors stop
  - [ ] `SCAN` → sensor readings appear
  - [ ] `STATUS` → JSON telemetry appears
- [ ] MPU6050 data appears in telemetry JSON
- [ ] Encoder counts increment when wheels turn

### ROS 2 Packages
- [ ] ROS 2 Humble installed
- [ ] Workspace built: `colcon build`
- [ ] No build errors
- [ ] Workspace sourced: `source install/setup.bash`
- [ ] YDLidar driver installed: `ros-humble-ydlidar-ros2-driver`
- [ ] SLAM Toolbox installed: `ros-humble-slam-toolbox`
- [ ] Launch script executable: `chmod +x launch_slam.sh`

### Node Tests (Individual)
Test each node individually before full launch:

**1. ESP32 Bridge:**
```bash
source install/setup.bash
ros2 run frr_control esp32_bridge_node
# Should show: "ESP32 bridge node started"
# Check: ros2 topic echo /esp32_telemetry
```
- [ ] Node starts without errors
- [ ] Telemetry appears on `/esp32_telemetry`
- [ ] Encoders increment when wheels turn

**2. YDLidar:**
```bash
ros2 launch ydlidar_ros2_driver x2.launch.py
# Check: ros2 topic echo /scan
```
- [ ] LIDAR spins
- [ ] `/scan` topic publishes LaserScan data
- [ ] Range values reasonable (0.1 - 12.0 meters)

**3. MPU6050 Node:**
```bash
ros2 run frr_sensors mpu6050_node
# Check: ros2 topic echo /imu/mpu6050
```
- [ ] Node starts without I2C errors
- [ ] IMU data publishes on `/imu/mpu6050`
- [ ] Orientation quaternion updates

**4. Sensor Fusion:**
```bash
ros2 run frr_sensors sensor_fusion_node
# Check: ros2 topic echo /odom
```
- [ ] Node starts
- [ ] Odometry publishes on `/odom`
- [ ] Position updates when robot moves

**5. Camera:**
```bash
ros2 run frr_sensors camera_node
# Check: http://[pi-ip]:8080
```
- [ ] Camera opens successfully
- [ ] Video stream accessible
- [ ] ArUco detection works (if markers present)

**6. Robot TF Publisher:**
```bash
ros2 run frr_sensors robot_tf_publisher
# Check: ros2 run tf2_tools view_frames
```
- [ ] Static transforms published
- [ ] TF tree complete (base_link → laser, imu_link, etc.)

## ✅ Calibration Checklist

### Wheel Parameters
1. [ ] Measure wheel diameter with calipers: _______ mm
2. [ ] Convert to meters: _______ m
3. [ ] Measure distance between wheel centers: _______ mm
4. [ ] Convert to meters: _______ m
5. [ ] Count encoder ticks per revolution:
   - [ ] Mark wheel position
   - [ ] Rotate exactly one revolution
   - [ ] Count ticks: _______ ticks
6. [ ] Update in launch files:
   ```python
   'wheel_diameter': _______,
   'wheel_base': _______,
   'encoder_ticks_per_rev': _______,
   ```

### Fire Sensor Thresholds
1. [ ] Test MQ2 in clean air: _______ (baseline)
2. [ ] Test MQ2 near smoke: _______ (fire threshold)
3. [ ] Test MQ5 in clean air: _______ (baseline)
4. [ ] Test MQ5 near gas: _______ (fire threshold)
5. [ ] Test flame sensor normal: _______ (baseline)
6. [ ] Test flame sensor with IR: _______ (fire threshold)
7. [ ] Update in launch files:
   ```python
   'fire_threshold_mq2': _______,
   'fire_threshold_mq5': _______,
   ```

### LIDAR Alignment
- [ ] LIDAR front (0°) points forward
- [ ] LIDAR mounted level (not tilted)
- [ ] LIDAR at known height above ground
- [ ] LIDAR center aligned with robot center

## ✅ Pre-Launch Checklist

### Environment Preparation
- [ ] Clear floor space (minimum 3m x 3m for testing)
- [ ] Remove trip hazards
- [ ] Good lighting for camera
- [ ] Place ArUco markers on walls (optional but recommended)
- [ ] Fire extinguisher nearby for fire tests
- [ ] Ventilation adequate for gas sensor tests

### System Verification
```bash
# Run the launcher
./launch_slam.sh

# Select Option 5: Check sensor status
```
- [ ] ✓ LIDAR is publishing
- [ ] ✓ IMU is publishing
- [ ] ✓ Odometry is publishing
- [ ] ✓ ESP32 is connected
- [ ] ✓ Camera is publishing

### TF Tree Verification
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf
```
- [ ] `map` → `odom` exists
- [ ] `odom` → `base_link` exists
- [ ] `base_link` → `laser` exists
- [ ] `base_link` → `imu_link` exists
- [ ] `base_link` → `camera_link` exists
- [ ] No broken transforms

## ✅ First Mapping Run

### Initial Test
1. [ ] Start mapping mode: `./launch_slam.sh` → Option 1
2. [ ] All nodes start successfully
3. [ ] No error messages in console
4. [ ] Video stream accessible: http://[pi-ip]:8080
5. [ ] LIDAR visible in RViz (if connected)

### Manual Driving Test
1. [ ] Open teleop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
2. [ ] Test forward/backward (I/K keys)
3. [ ] Test rotation (J/L keys)
4. [ ] Watch map build in RViz
5. [ ] Odometry position updates correctly
6. [ ] No jumps or glitches in position

### Autonomous Test
1. [ ] Start autonomous mode (already running in mapping)
2. [ ] Robot avoids obstacles
3. [ ] Robot searches area
4. [ ] Map builds as robot explores
5. [ ] No crashes or stuck situations

### Map Saving
1. [ ] Stop robot
2. [ ] Save map: `./launch_slam.sh` → Option 3
3. [ ] Enter map name: `test_map`
4. [ ] Verify files created: `ls -l maps/`
5. [ ] Should see: `test_map.posegraph`, `test_map.yaml`, `test_map.data`

## ✅ Fire Detection Test

### Sensor Warmup
- [ ] MQ2 warmed up (30-60 seconds, gets hot)
- [ ] MQ5 warmed up (30-60 seconds, gets hot)
- [ ] Baseline readings stable

### Detection Test
1. [ ] Monitor telemetry: `ros2 topic echo /esp32_telemetry`
2. [ ] Introduce smoke near MQ2
3. [ ] Value rises above threshold
4. [ ] Robot responds (moves toward source)
5. [ ] Water pump activates when close

### Safety Test
1. [ ] Place obstacle in front of robot
2. [ ] Robot should stop and avoid
3. [ ] Obstacle avoidance overrides fire detection
4. [ ] Robot finds alternate path if possible

## ✅ Troubleshooting Reference

### Quick Fixes

**"No /scan topic"**
```bash
sudo chmod 666 /dev/ydlidar
ros2 launch ydlidar_ros2_driver x2.launch.py
```

**"ESP32 not responding"**
```bash
ls -l /dev/ttyACM*  # Verify exists
sudo chmod 666 /dev/ttyACM0
screen /dev/ttyACM0 115200  # Test directly
```

**"IMU I2C error"**
- Check I2C connections (SDA, SCL)
- Verify MPU6050 address: `i2cdetect -y 1`
- Should show 0x68

**"Map not building"**
- Robot must be moving
- Check odometry: `ros2 topic echo /odom`
- Verify TF tree complete
- Increase `minimum_travel_distance` if too noisy

**"Poor localization"**
- Add ArUco markers to environment
- Calibrate wheel parameters accurately
- Check encoder wiring and counts
- Reduce speed for better accuracy

## ✅ Ready to Launch!

Once all items checked:

**Mapping Mode:**
```bash
./launch_slam.sh
# Select: 1 (Mapping)
```

**Localization Mode:**
```bash
./launch_slam.sh
# Select: 2 (Localization)
# Select saved map
```

## 📊 Performance Targets

Good performance indicators:
- [ ] Map update rate: 1-2 Hz
- [ ] Odometry rate: 10 Hz
- [ ] LIDAR rate: 5-10 Hz
- [ ] IMU rate: 10 Hz
- [ ] Position drift: < 5% of distance traveled
- [ ] Loop closure: Successfully closes loops
- [ ] Obstacle detection: Stops before collision
- [ ] Fire detection: Responds within 2 seconds

---

**Notes:**
- Keep this checklist handy for future deployments
- Update calibration values as needed
- Document any custom modifications
- Test in safe environment first!

**Ready?** Run: `./launch_slam.sh` and select your mode! 🚀
