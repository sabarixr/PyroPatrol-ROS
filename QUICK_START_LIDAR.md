# Quick Start Guide - LiDAR Navigation System

## What Was Done

### ✅ Removed Non-Working Components
- **MMA8452 accelerometer** - Marked as deprecated (sensor burned)
- **Sensor fusion code** - Removed from odometry node

### ✅ Created New LiDAR System
1. **lidar_node.py** - Publishes LiDAR scan data
2. **lidar_odometry_node.py** - Position tracking with triangulation
3. **obstacle_avoidance_node.py** - Robust 4-corner obstacle detection

### ✅ Key Features
- Starts from origin (0, 0) coordinates
- Uses triangulation to reduce position drift
- Accounts for LiDAR offset (16cm back, 8cm left, 10.5cm right, 10cm front)
- Detects obstacles in all 4 directions
- Prevents collisions during forward, reverse, and turning motions
- Emergency stop when trapped

## Files Modified

```
src/frr_sensors/frr_sensors/
├── imu_node.py                      [MODIFIED] - Deprecated, use mpu6050_node
├── odometry_node.py                 [MODIFIED] - Removed MMA8452 fusion
├── lidar_node.py                    [NEW] - LiDAR ROS2 node
├── lidar_odometry_node.py          [NEW] - Triangulation odometry
└── obstacle_avoidance_node.py      [NEW] - 4-corner obstacle avoidance

src/frr_sensors/
└── setup.py                         [MODIFIED] - Added new node entries

src/frr_bringup/launch/
└── lidar_navigation.launch.py      [NEW] - Launch all nodes together
```

## How to Use

### 1. Build (Already Done)
```bash
cd /home/alibaba/frr_ws
colcon build --packages-select frr_sensors frr_bringup
source install/setup.bash
```

### 2. Launch Complete System
```bash
# Terminal 1: Launch rover
ros2 launch frr_bringup rover_bringup.launch.py

# Terminal 2: Launch teleop with obstacle avoidance
ros2 run frr_control teleop_node --ros-args -r /cmd_vel:=/cmd_vel_teleop
```

This starts:
- MPU6050 IMU node (orientation)
- LiDAR node (scanning)
- LiDAR odometry node (position tracking)
- Obstacle avoidance node (safety)
- Camera & video (if enabled)
- Motor control

### 3. Test Individual Components

**Test LiDAR standalone:**
```bash
cd /home/alibaba/frr_ws
python3 lidar.py
```

**Test system diagnostics:**
```bash
./test_lidar_navigation.sh
```

**Tune obstacle avoidance:**
```bash
./tune_obstacle_avoidance.sh
```

### 4. Monitor Operation

**Check odometry (position):**
```bash
ros2 topic echo /lidar_odom
```

**Check obstacles:**
```bash
ros2 topic echo /obstacle_detected
```

**Check LiDAR scan:**
```bash
ros2 topic echo /scan --once
```

## Important Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/scan` | LaserScan | LiDAR measurements |
| `/imu/mpu6050` | Imu | IMU sensor data |
| `/lidar_odom` | Odometry | Position from (0,0) |
| `/cmd_vel_raw` | Twist | Raw commands (teleop) |
| `/cmd_vel` | Twist | Safe commands (to motors) |
| `/obstacle_detected` | Bool | Obstacle warning |

## Safety System

### Obstacle Detection Zones
```
        FRONT (-45° to +45°)
              ↑
              │
    LEFT ←────┼────→ RIGHT
   (-135°     │     (45° to 135°)
    to -45°)  │
              │
              ↓
        BACK (135° to 225°)
```

### Motion Safety
- **Forward**: Stops if front obstacle
- **Reverse**: Stops if back obstacle  
- **Turn Left**: Stops if left obstacle
- **Turn Right**: Stops if right obstacle
- **Emergency**: Stops if trapped (obstacles on opposite sides)

## Tuning Parameters

### Conservative (Safe)
```bash
ros2 param set /obstacle_avoidance_node safety_margin 0.20
ros2 param set /obstacle_avoidance_node stop_distance 0.40
```

### Balanced (Default)
```bash
ros2 param set /obstacle_avoidance_node safety_margin 0.15
ros2 param set /obstacle_avoidance_node stop_distance 0.30
```

### Aggressive (Tight Spaces)
```bash
ros2 param set /obstacle_avoidance_node safety_margin 0.10
ros2 param set /obstacle_avoidance_node stop_distance 0.20
```

## Troubleshooting

### LiDAR Not Working
```bash
# Check connection
ls /dev/ttyUSB0

# Fix permissions
sudo chmod 666 /dev/ttyUSB0
```

### No Odometry Data
```bash
# Check if nodes are running
ros2 node list

# Check if topics are publishing
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /imu/mpu6050
```

### Obstacle Detection Too Sensitive
```bash
# Increase safety margin
ros2 param set /obstacle_avoidance_node safety_margin 0.20
```

### Position Drift
- Triangulation should auto-correct
- Ensure environment has features (walls, objects)
- Check that reference points were initialized (check logs)

## Testing Checklist

Before deploying, verify:

- [ ] LiDAR publishes at ~50 Hz: `ros2 topic hz /scan`
- [ ] IMU publishes orientation: `ros2 topic echo /imu/mpu6050 --once`
- [ ] Odometry starts at (0, 0): `ros2 topic echo /lidar_odom --once`
- [ ] Position updates when rover moves
- [ ] Forward motion stops at obstacles
- [ ] Reverse motion stops at obstacles
- [ ] Turns are blocked by side obstacles
- [ ] Emergency stop works when trapped

## System Architecture

```
┌─────────────┐
│   LiDAR     │──→ /scan
└─────────────┘
       │
       ├──→ Odometry Node ──→ /lidar_odom
       │         ↑
       │    ┌────┴────┐
       │    │   IMU   │ (orientation)
       │    └─────────┘
       │
       └──→ Obstacle Avoidance ──→ /cmd_vel (safe)
                 ↑
           /cmd_vel_raw (from teleop)
```

## Next Steps

1. **Test in open area** - Verify odometry accuracy
2. **Test obstacle avoidance** - Place objects in all directions
3. **Test combined motion** - Forward + turn, reverse + turn
4. **Tune parameters** - Adjust for your environment
5. **Monitor position drift** - Should be minimal with triangulation

## Support

📖 Full documentation: `LIDAR_NAVIGATION_README.md`
🔧 Test script: `./test_lidar_navigation.sh`
⚙️ Tuning tool: `./tune_obstacle_avoidance.sh`

For detailed algorithm explanations and advanced configuration, see the full README.

---

**Created:** November 14, 2025  
**Status:** ✅ Built and Ready to Deploy
