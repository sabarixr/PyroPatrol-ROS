# Fire-Fighting Robot SLAM System Architecture

## Complete System Diagram

```
╔═══════════════════════════════════════════════════════════════════════════════╗
║                         RASPBERRY PI 4 (Brain / ROS 2 Humble)                 ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║                                                                               ║
║  ┌────────────────────────────────────────────────────────────────────────┐  ║
║  │                        SLAM & NAVIGATION STACK                         │  ║
║  ├────────────────────────────────────────────────────────────────────────┤  ║
║  │                                                                        │  ║
║  │   ┌──────────────────┐         ┌─────────────────────┐               │  ║
║  │   │  SLAM Toolbox    │────────>│    Occupancy Map    │               │  ║
║  │   │   (Mapping/      │         │     /map topic      │               │  ║
║  │   │  Localization)   │         └─────────────────────┘               │  ║
║  │   └────────┬─────────┘                                                │  ║
║  │            │                                                           │  ║
║  │            │ map ──> odom transform                                   │  ║
║  │            │                                                           │  ║
║  │   ┌────────▼─────────┐                                                │  ║
║  │   │ Sensor Fusion    │                                                │  ║
║  │   │  - Encoders      │                                                │  ║
║  │   │  - IMU (MPU6050) │                                                │  ║
║  │   │  - LIDAR assist  │                                                │  ║
║  │   └────────┬─────────┘                                                │  ║
║  │            │                                                           │  ║
║  │            │ /odom topic + odom ──> base_link TF                     │  ║
║  │            │                                                           │  ║
║  │   ┌────────▼─────────┐                                                │  ║
║  │   │ Autonomous       │                                                │  ║
║  │   │ Firebot Node     │                                                │  ║
║  │   │                  │                                                │  ║
║  │   │ Priority 1:      │  Inputs:                                      │  ║
║  │   │  Obstacle        │  • /scan (LIDAR)                              │  ║
║  │   │  Avoidance       │  • /esp32_telemetry (fire sensors)            │  ║
║  │   │                  │  • /aruco/pose (markers)                      │  ║
║  │   │ Priority 2:      │                                                │  ║
║  │   │  Fire Detection  │  Output:                                      │  ║
║  │   │  & Extinguish    │  • /cmd_vel (velocity commands)               │  ║
║  │   │                  │                                                │  ║
║  │   │ Priority 3:      │                                                │  ║
║  │   │  Search Pattern  │                                                │  ║
║  │   └────────┬─────────┘                                                │  ║
║  │            │                                                           │  ║
║  └────────────┼───────────────────────────────────────────────────────────┘  ║
║               │                                                               ║
║               │ /cmd_vel (Twist messages)                                    ║
║               │                                                               ║
║  ┌────────────▼───────────────────────────────────────────────────────────┐  ║
║  │                          SENSOR NODES                                  │  ║
║  ├────────────────────────────────────────────────────────────────────────┤  ║
║  │                                                                        │  ║
║  │  ┌───────────┐  ┌───────────┐  ┌───────────┐  ┌──────────────────┐  │  ║
║  │  │  YDLidar  │  │  MPU6050  │  │  Camera   │  │  Robot TF Pub    │  │  ║
║  │  │    X2     │  │   Node    │  │   Node    │  │  (Static TFs)    │  │  ║
║  │  │           │  │           │  │           │  │                  │  │  ║
║  │  │ /scan     │  │ /imu/     │  │ /camera/  │  │  base_link ──>   │  │  ║
║  │  │ topic     │  │ mpu6050   │  │ image_raw │  │  - laser         │  │  ║
║  │  │           │  │           │  │           │  │  - imu_link      │  │  ║
║  │  │ 360°      │  │ 6-axis    │  │ ArUco     │  │  - camera_link   │  │  ║
║  │  │ LIDAR     │  │ IMU       │  │ detection │  │  - wheels        │  │  ║
║  │  └─────┬─────┘  └─────┬─────┘  └─────┬─────┘  └──────────────────┘  │  ║
║  │        │              │              │                                │  ║
║  └────────┼──────────────┼──────────────┼────────────────────────────────┘  ║
║           │              │              │                                    ║
║  ┌────────▼──────────────▼──────────────▼────────────────────────────────┐  ║
║  │                      ESP32 Bridge Node                                │  ║
║  │                                                                        │  ║
║  │  • Converts /cmd_vel to ESP32 DRIVE commands                          │  ║
║  │  • Parses ESP32 telemetry JSON                                        │  ║
║  │  • Publishes /esp32_telemetry                                         │  ║
║  │  • Serial communication (115200 baud)                                 │  ║
║  └────────────────────────────────┬───────────────────────────────────────┘  ║
║                                   │                                          ║
║                                   │ Serial USB (/dev/ttyACM0)                ║
║                                   │ Text Commands + JSON                     ║
╚═══════════════════════════════════╪══════════════════════════════════════════╝
                                    │
╔═══════════════════════════════════╪══════════════════════════════════════════╗
║                                   │                ESP32 (Muscle)            ║
╠═══════════════════════════════════╪══════════════════════════════════════════╣
║                                   │                                          ║
║  ┌────────────────────────────────▼──────────────────────────────────────┐  ║
║  │                      ESP32 MAIN CONTROLLER                            │  ║
║  │                                                                        │  ║
║  │  Commands Received:                   Telemetry Sent:                 │  ║
║  │  • DRIVE <L> <R>                      • Fire sensors (MQ2, MQ5, ...)  │  ║
║  │  • STOP                               • Encoder counts (L, R)         │  ║
║  │  • PUMP_ON / PUMP_OFF                 • IMU data (ax,ay,az,gx,gy,gz) │  ║
║  │  • SCAN                               • Temperature                   │  ║
║  │  • STATUS                             • JSON format @ 10 Hz           │  ║
║  │  • DISABLE                                                            │  ║
║  └────────┬──────────────┬──────────────┬──────────────┬─────────────────┘  ║
║           │              │              │              │                     ║
║  ┌────────▼──────┐  ┌────▼──────┐  ┌───▼──────┐  ┌────▼─────────────────┐  ║
║  │   L298N       │  │ Encoders  │  │  MPU6050 │  │  Fire Sensors        │  ║
║  │   Motor       │  │ (IR)      │  │  IMU     │  │                      │  ║
║  │   Driver      │  │           │  │  I2C     │  │  • MQ2 (smoke)       │  ║
║  │               │  │ GPIO 13   │  │          │  │  • MQ5 (gas)         │  ║
║  │ GPIO:         │  │ GPIO 14   │  │ SDA: 21  │  │  • Flame (IR)        │  ║
║  │  27,22,23     │  │           │  │ SCL: 22  │  │  • Temperature       │  ║
║  │  24,17,18     │  │ Interrupt │  │          │  │                      │  ║
║  └───────┬───────┘  └───────┬───┘  └──────────┘  └──────────────────────┘  ║
║          │                  │                                                ║
║  ┌───────▼──────────────────▼───────┐         ┌──────────────────────────┐  ║
║  │      DC Motors (Left/Right)      │         │   Water Pump Relay       │  ║
║  │                                   │         │   GPIO 25                │  ║
║  │  • 6V DC Geared Motors            │         │                          │  ║
║  │  • Hardware PWM from ESP32        │         │  Activate on fire detect │  ║
║  │  • Smooth speed control           │         │                          │  ║
║  └───────────────────────────────────┘         └──────────────────────────┘  ║
║                                                                               ║
╚═══════════════════════════════════════════════════════════════════════════════╝
```

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         PERCEPTION LAYER                                │
└────┬──────────┬──────────┬──────────┬─────────────────────────────┬─────┘
     │          │          │          │                             │
     │          │          │          │                             │
  YDLidar    MPU6050    Camera    Encoders                      Fire Sensors
  /scan      /imu/      /camera/  (via ESP32)                   (via ESP32)
             mpu6050    image_raw                                /esp32_tele
     │          │          │          │                             │
     │          │          │          │                             │
     └──────────┴──────────┴──────────┴─────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────────────────┐
│                        FUSION LAYER                                     │
│                                                                         │
│  ┌────────────────────────────────────────────────────────────┐        │
│  │           Sensor Fusion Node                               │        │
│  │  • Combines encoder odometry with IMU orientation          │        │
│  │  • Complementary filter (70% encoders + 30% gyro)          │        │
│  │  • Publishes /odom at 10 Hz                                │        │
│  │  • Broadcasts odom ──> base_link TF                        │        │
│  └──────────────────────────────┬─────────────────────────────┘        │
│                                 │                                       │
└─────────────────────────────────┼───────────────────────────────────────┘
                                  │
┌─────────────────────────────────▼───────────────────────────────────────┐
│                         MAPPING LAYER                                   │
│                                                                         │
│  ┌────────────────────────────────────────────────────────────┐        │
│  │                    SLAM Toolbox                            │        │
│  │  • Consumes /scan (LIDAR) + /odom (fused)                  │        │
│  │  • Builds occupancy grid map (/map)                        │        │
│  │  • Publishes map ──> odom transform                        │        │
│  │  • Loop closure for drift correction                       │        │
│  └──────────────────────────────┬─────────────────────────────┘        │
│                                 │                                       │
└─────────────────────────────────┼───────────────────────────────────────┘
                                  │
┌─────────────────────────────────▼───────────────────────────────────────┐
│                      NAVIGATION LAYER                                   │
│                                                                         │
│  ┌────────────────────────────────────────────────────────────┐        │
│  │           Autonomous Firebot Node                          │        │
│  │                                                             │        │
│  │  Priority 1: Obstacle Avoidance                            │        │
│  │  ├─ Monitors /scan for obstacles < 0.5m                    │        │
│  │  └─ Emergency stop & turn if obstacle detected             │        │
│  │                                                             │        │
│  │  Priority 2: Fire Detection & Extinguishing                │        │
│  │  ├─ Monitors /esp32_telemetry for fire signatures          │        │
│  │  ├─ Moves toward fire source                               │        │
│  │  └─ Activates pump when close                              │        │
│  │                                                             │        │
│  │  Priority 3: Search Pattern                                │        │
│  │  └─ Random walk to explore and map area                    │        │
│  │                                                             │        │
│  │  Output: /cmd_vel (Twist)                                  │        │
│  └──────────────────────────────┬─────────────────────────────┘        │
│                                 │                                       │
└─────────────────────────────────┼───────────────────────────────────────┘
                                  │
┌─────────────────────────────────▼───────────────────────────────────────┐
│                       EXECUTION LAYER                                   │
│                                                                         │
│  ┌────────────────────────────────────────────────────────────┐        │
│  │              ESP32 Bridge Node                             │        │
│  │  • Subscribes to /cmd_vel                                  │        │
│  │  • Converts Twist to differential drive                    │        │
│  │  • Sends "DRIVE <left> <right>" via serial                 │        │
│  │  • ESP32 executes with hardware PWM                        │        │
│  └──────────────────────────────┬─────────────────────────────┘        │
│                                 │                                       │
└─────────────────────────────────┼───────────────────────────────────────┘
                                  │
                            Motors Move!
```

## TF Transform Tree

```
map (SLAM Toolbox)
 │
 └── odom (SLAM Toolbox publishes map→odom)
      │
      └── base_link (Sensor Fusion publishes odom→base_link)
           │
           ├── base_footprint (Static, -5cm Z)
           │
           ├── laser (Static, +5cm Z, center)
           │    └── [YDLidar X2 measures from here]
           │
           ├── imu_link (Static, +3cm Z, center)
           │    └── [MPU6050 orientation from here]
           │
           ├── camera_link (Static, +8cm X, +10cm Z)
           │    └── [Camera captures from here]
           │         └── [ArUco poses relative to this]
           │
           ├── left_wheel (Static, +10cm Y, -3.25cm Z)
           │    └── [Left encoder counts rotations]
           │
           └── right_wheel (Static, -10cm Y, -3.25cm Z)
                └── [Right encoder counts rotations]
```

## Communication Protocol

### ROS 2 Topics (Raspberry Pi)
```
Published:
  /scan               - sensor_msgs/LaserScan     @ 10 Hz
  /imu/mpu6050        - sensor_msgs/Imu           @ 10 Hz
  /odom               - nav_msgs/Odometry         @ 10 Hz
  /map                - nav_msgs/OccupancyGrid    @ 1 Hz
  /camera/image_raw   - sensor_msgs/Image         @ 30 Hz
  /aruco/pose         - geometry_msgs/PoseStamped (when detected)
  /esp32_telemetry    - std_msgs/String (JSON)    @ 10 Hz
  /cmd_vel            - geometry_msgs/Twist       (as needed)

Subscribed:
  /scan               - By SLAM Toolbox, Autonomous Firebot
  /imu/mpu6050        - By Sensor Fusion
  /odom               - By SLAM Toolbox
  /cmd_vel            - By ESP32 Bridge
  /esp32_telemetry    - By Sensor Fusion, Autonomous Firebot
```

### Serial Protocol (ESP32 ←→ Raspberry Pi)
```
Direction: Raspberry Pi → ESP32
Format: Text commands, newline terminated
Commands:
  DRIVE <left> <right>    # -100 to 100 for each wheel
  STOP                    # Emergency stop
  PUMP_ON                 # Activate water pump
  PUMP_OFF                # Deactivate water pump
  SCAN                    # Request sensor reading
  STATUS                  # Request full telemetry
  DISABLE                 # Disable motors

Direction: ESP32 → Raspberry Pi
Format: JSON string, newline terminated
Telemetry (10 Hz):
  {
    "mq2": 450,                    # Smoke sensor (0-4095)
    "mq5": 380,                    # Gas sensor (0-4095)
    "flame": 120,                  # Flame sensor (0-4095)
    "temp": 2500,                  # Temperature (ADC value)
    "encoders": {
      "left": 12345,               # Left encoder ticks
      "right": 12340               # Right encoder ticks
    },
    "imu": {
      "ax": 0.012,                 # Accel X (g)
      "ay": -0.003,                # Accel Y (g)
      "az": 1.002,                 # Accel Z (g)
      "gx": 0.15,                  # Gyro X (deg/s)
      "gy": -0.08,                 # Gyro Y (deg/s)
      "gz": 0.02,                  # Gyro Z (deg/s)
      "temp_c": 28.5               # IMU temp (°C)
    }
  }
```

## File Locations

```
/home/alibaba/frr_ws/
├── launch_slam.sh                          # Interactive launcher
├── QUICK_START.md                          # Quick reference
├── SLAM_README.md                          # Detailed documentation
├── SETUP_CHECKLIST.md                      # Pre-flight checklist
├── ARCHITECTURE.md                         # This file
├── ESP32_FIRMWARE_TEMPLATE.ino             # ESP32 Arduino code
│
├── maps/                                   # Saved SLAM maps
│   ├── map1.posegraph
│   ├── map1.yaml
│   └── map1.data
│
└── src/
    ├── frr_bringup/
    │   └── launch/
    │       ├── slam_mapping.launch.py      # Mapping mode
    │       └── slam_localization.launch.py # Localization mode
    │
    ├── frr_sensors/
    │   └── frr_sensors/
    │       ├── robot_tf_publisher.py       # Static TF frames
    │       ├── sensor_fusion_node.py       # Odometry fusion
    │       ├── mpu6050_node.py             # IMU driver
    │       └── camera_node.py              # Camera + ArUco
    │
    ├── frr_control/
    │   └── frr_control/
    │       └── esp32_bridge_node.py        # ESP32 communication
    │
    └── frr_navigation/
        └── frr_navigation/
            └── autonomous_firebot_node.py  # Autonomous brain
```

---

**System Philosophy:**
- **Raspberry Pi = Brain**: High-level decision making, sensor fusion, SLAM
- **ESP32 = Muscle**: Real-time motor control, sensor reading, hardware PWM
- **ROS 2 = Nervous System**: Inter-node communication, coordinate transforms
- **SLAM = Memory**: Builds and maintains spatial awareness
- **Sensors = Senses**: Perceive environment (vision, touch, smell, orientation)

**Design Goals:**
✓ Modular architecture (easy to add/remove sensors)
✓ Real-time obstacle avoidance (safety first)
✓ Accurate localization (sensor fusion + SLAM)
✓ Fire detection and extinguishing (primary mission)
✓ Autonomous exploration (search unknown areas)
✓ Robust communication (ESP32 handles motor glitches)
