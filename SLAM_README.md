# Fire-Fighting Robot SLAM System

## Overview
This SLAM system integrates multiple sensors for autonomous fire-fighting navigation:
- **YDLidar X2**: 360° LIDAR for obstacle detection and mapping
- **MPU6050**: IMU for orientation and angular velocity
- **Wheel Encoders**: On ESP32 for odometry
- **Camera**: ArUco marker detection for localization
- **Fire Sensors**: MQ2 (smoke), MQ5 (gas), flame sensor, temperature

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Raspberry Pi (Brain)                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │   YDLidar    │  │   MPU6050    │  │   Camera     │         │
│  │     X2       │  │     IMU      │  │  (ArUco)     │         │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘         │
│         │                  │                  │                 │
│         └──────────┬───────┴──────────────────┘                 │
│                    │                                            │
│         ┌──────────▼────────────┐                               │
│         │   Sensor Fusion Node   │                              │
│         │  (LIDAR + IMU + Odom)  │                              │
│         └──────────┬─────────────┘                              │
│                    │                                            │
│         ┌──────────▼────────────┐                               │
│         │   SLAM Toolbox Node    │                              │
│         │    (Map Building)      │                              │
│         └──────────┬─────────────┘                              │
│                    │                                            │
│         ┌──────────▼────────────┐                               │
│         │ Autonomous Firebot     │                              │
│         │  (Fire Detection +     │                              │
│         │   Obstacle Avoidance)  │                              │
│         └──────────┬─────────────┘                              │
│                    │                                            │
│         ┌──────────▼────────────┐                               │
│         │   ESP32 Bridge Node    │                              │
│         └──────────┬─────────────┘                              │
└────────────────────┼─────────────────────────────────────────────┘
                     │ Serial (115200 baud)
┌────────────────────▼─────────────────────────────────────────────┐
│                      ESP32 (Muscle)                              │
├──────────────────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │  L298N   │  │ Encoders │  │   Fire   │  │  Water   │        │
│  │  Motor   │  │  (IR)    │  │ Sensors  │  │  Pump    │        │
│  │  Driver  │  │          │  │          │  │          │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
└──────────────────────────────────────────────────────────────────┘
```

## Launch Files

### 1. Mapping Mode (First Time)
Creates a new map while exploring:

```bash
cd /home/alibaba/frr_ws
source install/setup.bash
ros2 launch frr_bringup slam_mapping.launch.py
```

**What it does:**
- Starts all sensors (LIDAR, IMU, Camera, ESP32)
- Runs sensor fusion to combine odometry sources
- Creates map in real-time using SLAM Toolbox
- Enables autonomous fire-fighting navigation
- Streams video to http://[raspberry-pi-ip]:8080

**Usage:**
1. Let the robot explore the environment
2. When satisfied with the map, save it:
   ```bash
   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/alibaba/frr_ws/maps/firebot_map'}}"
   ```
3. The map will be saved to `/home/alibaba/frr_ws/maps/`

### 2. Localization Mode (Use Existing Map)
Navigate using a pre-built map:

```bash
cd /home/alibaba/frr_ws
source install/setup.bash
ros2 launch frr_bringup slam_localization.launch.py
```

Or specify a custom map:
```bash
ros2 launch frr_bringup slam_localization.launch.py map_file:=/path/to/your/map.posegraph
```

**What it does:**
- Loads existing map
- Localizes robot position within the map
- Uses all sensors for accurate localization
- Enables autonomous navigation with known map

## ROS Topics

### Published Topics:
- `/scan` - LaserScan from YDLidar X2
- `/imu/mpu6050` - IMU data from MPU6050
- `/odom` - Fused odometry (encoders + IMU + LIDAR)
- `/map` - Occupancy grid map from SLAM Toolbox
- `/cmd_vel` - Velocity commands to ESP32
- `/esp32_telemetry` - Sensor readings (MQ2, MQ5, flame, temp, encoders)
- `/aruco/pose` - ArUco marker poses for localization
- `/camera/image_raw` - Camera feed

### Subscribed Topics:
- `/scan` - Used by SLAM Toolbox and autonomous navigation
- `/imu/mpu6050` - Used by sensor fusion
- `/esp32_telemetry` - Used by sensor fusion and autonomous navigation

## Sensor Fusion Parameters

Edit in launch file or override at runtime:

```yaml
wheel_diameter: 0.065        # meters (adjust to your wheels)
wheel_base: 0.2              # meters (distance between wheels)
encoder_ticks_per_rev: 20    # ticks per wheel revolution
```

**How fusion works:**
1. Converts encoder ticks to linear/angular velocity
2. Integrates to get position estimate
3. Fuses with IMU orientation (complementary filter)
   - 70% encoder-based rotation
   - 30% IMU gyroscope
4. Publishes `/odom` topic and `odom→base_link` TF transform

## SLAM Toolbox Parameters

Key parameters (in launch files):

```yaml
resolution: 0.05                    # Map resolution in meters
max_laser_range: 12.0               # Maximum LIDAR range
minimum_travel_distance: 0.2        # Min distance before scan matching
minimum_travel_heading: 0.2         # Min rotation before scan matching
loop_search_maximum_distance: 3.0   # Loop closure search radius
do_loop_closing: true               # Enable loop closure
```

## Autonomous Navigation

The autonomous firebot operates with a 3-level priority system:

### Priority 1: Obstacle Avoidance
- Monitors LIDAR scan in 3 sectors (front, left, right)
- If obstacle < 0.5m: emergency stop and turn away
- Always takes priority over fire detection

### Priority 2: Fire Detection
Activated when:
- MQ2 sensor > 600 (smoke)
- MQ5 sensor > 550 (gas)
- Temperature rise > 2.0°C
- Flame sensor detects IR

Actions:
- Move toward fire source
- Activate water pump when close
- Continue until fire extinguished

### Priority 3: Search Pattern
When no obstacles or fire:
- Random walk pattern
- Covers area systematically

## Calibration

### 1. Wheel Parameters
Measure your robot and update in launch files:
```bash
# Measure wheel diameter
wheel_diameter = measured_diameter_in_meters

# Measure distance between wheel centers
wheel_base = measured_distance_in_meters

# Count encoder ticks per revolution
encoder_ticks_per_rev = counted_ticks
```

### 2. IMU Calibration
The MPU6050 node has built-in vibration filtering, but you can adjust:
```yaml
use_vibration_filter: true
filter_window_size: 5    # Larger = smoother but slower response
```

### 3. Fire Sensor Thresholds
Test in clean air, then near smoke/gas:
```yaml
fire_threshold_mq2: 600      # Adjust based on testing
fire_threshold_mq5: 550      # Adjust based on testing
temp_rise_threshold: 2.0     # Temperature change in °C
```

## Visualization

### RViz2 (on PC with ROS 2)
```bash
# On PC, set ROS_DOMAIN_ID to match robot
export ROS_DOMAIN_ID=0

# Launch RViz2
rviz2
```

Add displays:
- **Map** → Topic: `/map`
- **LaserScan** → Topic: `/scan`
- **TF** → Show all frames
- **Camera** → Topic: `/camera/image_raw`
- **Path** → Topic: `/slam_toolbox/graph_visualization`

### Video Stream
Open browser to: `http://[raspberry-pi-ip]:8080`

## Troubleshooting

### Map not building
1. Check LIDAR is spinning: `ros2 topic echo /scan`
2. Verify odometry: `ros2 topic echo /odom`
3. Check TF tree: `ros2 run tf2_tools view_frames`
4. Ensure robot is moving (can't map while stationary)

### Poor localization
1. Add more ArUco markers in environment
2. Increase `minimum_travel_distance` if too noisy
3. Check encoder connections to ESP32 GPIO 13, 14
4. Verify IMU data: `ros2 topic echo /imu/mpu6050`

### Robot not avoiding obstacles
1. Check LIDAR range: `ros2 topic echo /scan --field ranges`
2. Verify obstacle threshold in autonomous_firebot_node parameters
3. Ensure ESP32 is responding: `ros2 topic echo /esp32_telemetry`

### Fire not detected
1. Test sensors in Terminal: `ros2 topic echo /esp32_telemetry`
2. Lower threshold values if sensors not triggering
3. Check MQ sensor heating time (30-60 seconds warmup)

## ESP32 Commands

The ESP32 accepts these commands over serial:

```
DRIVE <left> <right>    # -100 to 100 for each wheel
STOP                    # Stop both motors
SCAN                    # Read fire sensors (MQ2, MQ5, flame, temp)
PUMP_ON                 # Activate water pump
PUMP_OFF                # Deactivate water pump
STATUS                  # Get full telemetry JSON
DISABLE                 # Disable motors
```

## Files

### Launch Files:
- `slam_mapping.launch.py` - Full SLAM mapping mode
- `slam_localization.launch.py` - Localization with existing map

### Nodes:
- `sensor_fusion_node` - Fuses encoders + IMU for odometry
- `mpu6050_node` - Reads MPU6050 IMU with vibration filtering
- `camera_node` - Camera with ArUco detection
- `esp32_bridge_node` - Communicates with ESP32
- `autonomous_firebot_node` - Autonomous navigation and fire-fighting
- `streamer_node` - Video streaming server

### Maps:
- Saved to `/home/alibaba/frr_ws/maps/`
- Format: `.posegraph` (SLAM Toolbox format)

## Next Steps

1. **Test mapping**: Drive robot around manually and watch map build
2. **Calibrate sensors**: Adjust thresholds for your environment
3. **Add ArUco markers**: Place markers on walls for better localization
4. **Test autonomous mode**: Let robot explore and find fire sources
5. **Save maps**: Save good maps for localization mode

## Safety Notes

⚠️ **IMPORTANT:**
- Always have manual override ready (kill switch on ESP32)
- Test in controlled environment first
- Monitor battery voltage during operation
- Keep water pump away from electronics
- MQ sensors get HOT during operation - don't touch!
- LIDAR laser is Class 1 but still avoid looking directly at it
