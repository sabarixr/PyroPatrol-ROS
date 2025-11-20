# LiDAR-Based Navigation System

## Overview

This update removes the non-working MMA8452 accelerometer and implements a complete LiDAR-based navigation system with:

1. **LiDAR Odometry with Triangulation** - Accurate position tracking starting from (0,0)
2. **Robust Obstacle Avoidance** - 4-corner detection for safe navigation in all directions
3. **MPU6050 IMU Integration** - Orientation and motion sensing

## Changes Made

### 1. Removed MMA8452 Accelerometer

#### `imu_node.py`
- Converted to placeholder node since sensor is burned/not working
- Recommends using `mpu6050_node` instead

#### `odometry_node.py`
- Removed MMA8452 subscription and sensor fusion
- Now uses only MPU6050 for basic odometry
- Simplified to single-sensor operation

### 2. New LiDAR System Components

#### `lidar_node.py`
- ROS2 wrapper for the existing `lidar.py` code structure
- Publishes `/scan` (LaserScan messages) for navigation
- Publishes `/lidar/raw` (raw distance data) for debugging
- 50 Hz update rate for responsive navigation

#### `lidar_odometry_node.py`
- **Position Tracking with Triangulation**
  - Starts from origin (0, 0)
  - Uses scan matching between consecutive frames
  - Implements triangulation with reference landmarks
  - Corrects drift using feature matching
  
- **LiDAR Offset Compensation**
  - Front: 10 cm from center
  - Back: 16 cm from center
  - Left: 8 cm from center
  - Right: 10.5 cm from center
  
- **IMU Fusion**
  - Gyroscope integration for accurate orientation
  - Accelerometer for velocity estimation
  - Complements LiDAR-based motion estimation
  
- **Publishes**: `/lidar_odom` (Odometry message)

#### `obstacle_avoidance_node.py`
- **4-Corner Detection System**
  - Front zone: -45° to +45°
  - Back zone: 135° to 225°
  - Left zone: -135° to -45°
  - Right zone: 45° to 135°
  
- **Smart Avoidance Logic**
  - Forward motion: Stops if obstacle ahead
  - Reverse motion: Stops if obstacle behind
  - Left turn: Stops if obstacle on left, suggests right turn
  - Right turn: Stops if obstacle on right, suggests left turn
  - Combined motions: Checks corners during turning
  
- **Safety Features**
  - Configurable safety margin (default: 15 cm)
  - Emergency stop distance (default: 30 cm)
  - Accounts for rover dimensions and LiDAR offset
  - Emergency stop if trapped (obstacles on multiple sides)
  
- **Topics**
  - Subscribes to: `/cmd_vel_raw` (raw commands from teleop)
  - Subscribes to: `/scan` (LiDAR data)
  - Publishes to: `/cmd_vel` (safe commands to motors)
  - Publishes to: `/obstacle_detected` (Boolean flag)

## Rover Dimensions

```
Rover Dimensions:
- Total Length: ~30 cm
- Total Width: ~20 cm

LiDAR Position (from center):
- Forward: 10 cm
- Backward: 16 cm
- Left: 8 cm
- Right: 10.5 cm

         Front (10cm)
              ▲
              │
    ┌─────────┼─────────┐
    │         │         │
Left│    🎯 LiDAR       │Right
8cm │         │         │10.5cm
    │         │         │
    │       Center      │
    │         │         │
    └─────────┼─────────┘
              │
              ▼
         Back (16cm)
```

## Usage

### Build the Workspace

```bash
cd /home/alibaba/frr_ws
colcon build --packages-select frr_sensors frr_bringup
source install/setup.bash
```

### Launch Complete Navigation System

```bash
ros2 launch frr_bringup lidar_navigation.launch.py
```

This will start:
1. MPU6050 IMU node
2. LiDAR node
3. LiDAR odometry node (with triangulation)
4. Obstacle avoidance node

### Launch Individual Nodes

#### LiDAR Only
```bash
ros2 run frr_sensors lidar_node
```

#### LiDAR Odometry
```bash
ros2 run frr_sensors lidar_odometry_node
```

#### Obstacle Avoidance
```bash
ros2 run frr_sensors obstacle_avoidance_node
```

### Test Standalone LiDAR

```bash
cd /home/alibaba/frr_ws
python3 lidar.py
```

## Topic Structure

```
/scan                    # LaserScan - LiDAR measurements
/lidar/raw              # Float32MultiArray - Raw LiDAR data
/imu/mpu6050           # Imu - MPU6050 sensor data
/lidar_odom            # Odometry - LiDAR-based position
/cmd_vel_raw           # Twist - Raw commands from teleop
/cmd_vel               # Twist - Safe commands (after obstacle avoidance)
/obstacle_detected     # Bool - Obstacle detection flag
```

## Configuration Parameters

### LiDAR Node
- `port`: Serial port (default: `/dev/ttyUSB0`)
- `baud`: Baud rate (default: `115200`)
- `scan_frequency`: Update rate in Hz (default: `50.0`)

### LiDAR Odometry Node
- `lidar_offset_x`: Forward offset in meters (default: `0.10`)
- `lidar_offset_y`: Right offset in meters (default: `-0.0125`)

### Obstacle Avoidance Node
- `rover_width`: Width in meters (default: `0.20`)
- `rover_length`: Length in meters (default: `0.30`)
- `safety_margin`: Safety distance in meters (default: `0.15`)
- `stop_distance`: Emergency stop distance in meters (default: `0.30`)

## Triangulation Algorithm

The odometry system uses a sophisticated triangulation approach:

1. **Feature Extraction**
   - Detects edges and corners from LiDAR scans
   - Filters noise and invalid measurements
   
2. **Reference Landmarks**
   - First scan establishes reference points in global frame
   - These act as "landmarks" for position correction
   
3. **Scan Matching**
   - ICP-like algorithm compares consecutive scans
   - Estimates frame-to-frame motion
   
4. **Triangulation**
   - Matches current features to reference landmarks
   - Calculates position from multiple landmark distances
   - Uses at least 3 landmarks for accurate positioning
   
5. **Sensor Fusion**
   - Combines scan matching with triangulation
   - IMU provides orientation (gyro integration)
   - Low-pass filter prevents abrupt corrections

## Obstacle Avoidance Algorithm

### Zone Detection
Each corner is monitored independently with proper offset compensation:

```python
# Front zone example
if -45° <= angle <= 45°:
    if distance < (lidar_front_offset + rover_length/2 + safety_margin):
        obstacle_front = True
```

### Motion Safety Rules

| Motion | Check Zones | Action |
|--------|-------------|--------|
| Forward | Front | Stop if blocked |
| Backward | Back | Stop if blocked |
| Turn Left | Left | Stop turn, suggest right |
| Turn Right | Right | Stop turn, suggest left |
| Forward + Left | Front, Left | Stop if either blocked |
| Forward + Right | Front, Right | Stop if either blocked |
| Backward + Left | Back, Right* | Stop if either blocked |
| Backward + Right | Back, Left* | Stop if either blocked |

*Note: In reverse, turn directions are inverted

### Emergency Stop
- Triggered when obstacles on opposite sides (trapped)
- Triggered when within stop_distance threshold
- All motion halted immediately

## Advantages of This System

1. **No Dead Sensors**: Removed non-working MMA8452
2. **Accurate Positioning**: Triangulation reduces drift
3. **Starts from Origin**: Known starting position (0, 0)
4. **Complete Coverage**: 360° obstacle detection
5. **Dimension Aware**: Uses actual rover and LiDAR geometry
6. **Robust Avoidance**: Handles all movement combinations
7. **Optimized**: Efficient algorithms for real-time performance

## Troubleshooting

### LiDAR Not Detected
```bash
# Check USB connection
ls /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyUSB0
```

### No Odometry Data
- Ensure LiDAR is publishing `/scan`
- Verify IMU is publishing `/imu/mpu6050`
- Check logs for initialization messages

### Obstacle Avoidance Too Sensitive
```bash
# Adjust safety margin (increase for less sensitive)
ros2 param set /obstacle_avoidance_node safety_margin 0.20
```

### Obstacle Avoidance Not Responsive Enough
```bash
# Reduce safety margin (decrease for more responsive)
ros2 param set /obstacle_avoidance_node safety_margin 0.10
```

### Position Drift
- Triangulation should reduce drift automatically
- Ensure environment has distinct features (not empty room)
- Check that reference landmarks were initialized

## Testing Checklist

- [ ] LiDAR publishes scan data at 50 Hz
- [ ] IMU publishes orientation data
- [ ] Odometry starts at (0, 0)
- [ ] Position updates as rover moves
- [ ] Forward motion stops at front obstacles
- [ ] Reverse motion stops at back obstacles
- [ ] Left turns stop at left obstacles
- [ ] Right turns stop at right obstacles
- [ ] Emergency stop when trapped
- [ ] Triangulation reduces position drift

## Future Improvements

1. **SLAM Integration**: Add mapping capability
2. **Path Planning**: Autonomous navigation to goal
3. **Dynamic Obstacles**: Track moving objects
4. **Multi-LiDAR**: Fuse multiple LiDAR sensors
5. **Machine Learning**: Learn optimal avoidance strategies

## Support

For issues or questions, check:
- `/scan` topic for LiDAR data
- `/lidar_odom` topic for position
- `/obstacle_detected` topic for avoidance status
- Node logs for detailed debugging

Created: November 14, 2025
