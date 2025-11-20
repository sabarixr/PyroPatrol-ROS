# 🚀 QUICK FIX GUIDE - Your Robot Issues

## 3 Problems You Reported:

1. ❌ **MPU6050 still showing I2C errors**
2. ❌ **Obstacle avoidance not working** (robot hit box in front)
3. ❌ **Need to see if LiDAR is actually working**

---

## ✅ SOLUTION - Follow These Steps:

### Step 1: Stop Current Robot (In the terminal running it)
```bash
# Press Ctrl+C in terminal with ros2 launch
```

### Step 2: Start Fixed Robot
```bash
cd ~/frr_ws
./start_robot.sh
```

This will verify ALL fixes are applied and start the robot.

### Step 3: In NEW Terminal - Check LiDAR Is Working
```bash
cd ~/frr_ws
source install/setup.bash
python3 check_lidar.py
```

**What you should see:**
- 🟢 GREEN = All clear (>0.5m away)
- 🟡 YELLOW = Warning (0.35-0.5m)
- 🔴 RED = DANGER (< 0.35m) - Robot should STOP

**Put box in front of robot:**
- Front distance should show **RED** when < 35cm
- If it shows GREEN while box is close = LiDAR not working properly

### Step 4: In ANOTHER Terminal - Start Teleop
```bash
cd ~/frr_ws
./start_teleop.sh
# Press 'c' to continue if detection fails
# Choose option 1 (SAFE mode)
```

### Step 5: Test Obstacle Avoidance
1. **With check_lidar.py running**, put your hand 30cm in front
2. You should see **RED warning** in LiDAR viewer
3. Try driving forward with teleop → Robot should STOP or SLOW DOWN
4. If robot still moves → Obstacle avoidance not receiving LiDAR data

---

## 🔧 What Got Fixed:

### 1. Speed Increased ✓
- **Teleop:** 1.5 → **2.5 m/s**
- **Motor Driver:** 1.5 → **2.5 m/s**
- Both files updated to match your request

### 2. MPU6050 Vibration Filtering ✓
- Already applied (vibration-filtered code is active)
- But I2C errors still show → **Hardware/wiring issue likely**

### 3. Obstacle Detection Made More Sensitive ✓
- Safety margin: 15cm → **25cm** (67% larger)
- Stop distance: 30cm → **35cm**
- Should now detect obstacles farther away

### 4. LiDAR Visualization Tool Created ✓
- **check_lidar.py** shows real-time distances with colors
- Helps diagnose if LiDAR actually sees obstacles

---

## 🐛 If Obstacle Avoidance STILL Doesn't Work:

### Debug Step 1: Is obstacle_avoidance_node running?
```bash
ros2 node list | grep obstacle
```
Should show: `/obstacle_avoidance_node`

### Debug Step 2: Is it receiving LiDAR data?
```bash
ros2 topic hz /scan
```
Should show: `average rate: ~10.000`

### Debug Step 3: Is it receiving teleop commands?
```bash
ros2 topic hz /cmd_vel_teleop
```
Should show rate when you press keys

### Debug Step 4: Is it publishing filtered commands?
```bash
ros2 topic hz /cmd_vel
```
Should show rate (obstacle node publishes filtered commands here)

### Debug Step 5: Check topic flow
```bash
# Start teleop in safe mode, then:
ros2 topic echo /cmd_vel_teleop --once
# Press 'w' key, you should see velocity

ros2 topic echo /cmd_vel --once
# Should show filtered velocity (may be zero if obstacle detected)
```

---

## 🔍 Diagnosing MPU6050 I2C Errors

The MPU6050 errors might be **hardware/electrical** issues:

### Possible Causes:
1. **Loose wiring** - Check connections:
   - SDA → GPIO 2 (Pin 3)
   - SCL → GPIO 3 (Pin 5)  
   - VCC → 3.3V
   - GND → GND

2. **Electrical interference** - LiDAR motor creates noise:
   - Try running WITHOUT LiDAR: `ros2 launch frr_bringup rover_bringup.launch.py enable_lidar:=false`
   - If errors stop → Need hardware filtering (capacitors)

3. **I2C speed too fast** - Slow down I2C bus:
```bash
sudo nano /boot/firmware/config.txt
# Add or change:
dtparam=i2c_arm_baudrate=50000
# Save, then reboot
```

4. **Power supply issues** - Check:
```bash
# Measure voltage at MPU6050 VCC pin
# Should be 3.2-3.4V steady
```

---

## 📊 Expected Behavior After Fixes:

### ✅ WORKING System:
- **Speed:** Robot drives up to 2.5 m/s when full throttle
- **LiDAR Viewer:** Shows RED when object < 35cm in front
- **Obstacle Avoidance:** Robot stops/slows when LiDAR viewer shows RED front
- **No Safety Timeouts:** Teleop publishes continuously

### ❌ If Still Broken:

**Symptom:** Robot hits obstacles even when LiDAR shows RED
- **Cause:** Obstacle avoidance node not running OR not in data path
- **Fix:** Check launch file has obstacle_avoidance_node enabled
- **Verify:** `ros2 node list` should show obstacle_avoidance_node

**Symptom:** LiDAR always shows GREEN even with box in front
- **Cause:** LiDAR not working OR USB cable disconnected
- **Fix:** 
  ```bash
  ls /dev/ttyUSB*  # Should show /dev/ttyUSB0
  sudo chmod 666 /dev/ttyUSB0
  ```

**Symptom:** MPU6050 errors never stop
- **Cause:** Hardware/electrical issue (not software)
- **Fix:** Check wiring, add capacitors, or disable MPU6050:
  ```bash
  # Edit launch file to comment out MPU6050 node
  ```

---

## 🎯 Test Checklist:

Run these tests IN ORDER:

### Test 1: LiDAR Sees Obstacles ✓
```bash
python3 check_lidar.py
# Put box 30cm in front → Should show RED
```

### Test 2: Obstacle Avoidance Gets LiDAR Data ✓
```bash
ros2 topic hz /scan
# Should show ~10 Hz
```

### Test 3: Teleop Publishes Commands ✓
```bash
# Start teleop, press 'w'
ros2 topic echo /cmd_vel_teleop --once
# Should show linear.x > 0
```

### Test 4: Obstacle Node Filters Commands ✓
```bash
# With box in front, press 'w'
ros2 topic echo /cmd_vel --once
# Should show linear.x = 0 (stopped) if obstacle detected
```

### Test 5: Motors Respond ✓
```bash
# Remove obstacles, press 'w'
# Robot should move forward up to 2.5 m/s
```

---

## 🚨 Emergency: If Nothing Works

### Nuclear Option - Clean Restart:
```bash
# Stop everything
pkill -f ros2

# Rebuild from scratch
cd ~/frr_ws
rm -rf build/ install/ log/
colcon build --symlink-install

# Source
source install/setup.bash

# Start fresh
ros2 launch frr_bringup rover_bringup.launch.py
```

### Last Resort - Disable Problematic Nodes:
```bash
# If MPU6050 won't stop erroring, disable it:
ros2 launch frr_bringup rover_bringup.launch.py enable_imu:=false

# If obstacle avoidance causes issues, bypass it:
# Use teleop in DIRECT mode (option 2)
```

---

## 📝 Quick Command Reference:

```bash
# Start robot
cd ~/frr_ws && ./start_robot.sh

# Check LiDAR (in new terminal)
cd ~/frr_ws && source install/setup.bash && python3 check_lidar.py

# Start teleop (in another new terminal)
cd ~/frr_ws && ./start_teleop.sh

# Check what nodes are running
ros2 node list

# Check topic rates
ros2 topic hz /scan
ros2 topic hz /cmd_vel_teleop
ros2 topic hz /cmd_vel

# View topic data
ros2 topic echo /scan --once
ros2 topic echo /cmd_vel --once
```

---

**TL;DR:**
1. Run `./start_robot.sh` 
2. Run `python3 check_lidar.py` to see if LiDAR works
3. Run `./start_teleop.sh` to control robot
4. Put box in front while driving → Robot should stop if working

If robot still hits box while LiDAR shows RED = obstacle avoidance not in the data path!
