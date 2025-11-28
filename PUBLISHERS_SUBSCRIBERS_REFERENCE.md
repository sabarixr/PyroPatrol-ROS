# ROS2 Publishers & Subscribers Reference - PyroPatrol

Complete reference of all ROS2 publishers, subscribers, and their callbacks in the PyroPatrol codebase.

---

## 1. Sensor Fusion Node (`frr_sensors/sensor_fusion_node.py`)

### Subscribers
```python
# ESP32 Telemetry (wheel encoders)
self.telemetry_sub = self.create_subscription(
    String, '/esp32_telemetry', self.telemetry_callback, 10
)

# IMU Data
self.imu_sub = self.create_subscription(
    Imu, '/imu/mpu6050', self.imu_callback, 10
)
```

### Publishers
```python
# Odometry (fused wheel encoders + IMU)
self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

# Fused Pose for higher-level navigation
self.fusion_pose_pub = self.create_publisher(PoseStamped, '/sensor_fusion/pose', 10)
```

### Callbacks
```python
def telemetry_callback(self, msg):
    """Receive encoder data from ESP32 and compute odometry"""
    try:
        data = json.loads(msg.data)
        encoders = data.get('encoders', {})
        left_ticks = encoders.get('left', 0)
        right_ticks = encoders.get('right', 0)
        self.update_odometry(left_ticks, right_ticks)
    except (json.JSONDecodeError, KeyError):
        pass

def imu_callback(self, msg):
    """Store IMU data for sensor fusion"""
    self.imu_orientation = msg.orientation
    self.imu_angular_velocity_z = msg.angular_velocity.z
```

---

## 2. Autonomous Firebot Node (`frr_navigation/autonomous_firebot_node.py`)

### Subscribers
```python
# ESP32 Telemetry (fire sensors)
self.telemetry_sub = self.create_subscription(
    String, '/esp32_telemetry', self.telemetry_callback, 10
)

# LIDAR Scan
self.lidar_sub = self.create_subscription(
    LaserScan, '/scan', self.lidar_callback, 10
)

# ArUco Marker Pose
self.aruco_sub = self.create_subscription(
    Pose, '/aruco/pose', self.aruco_callback, 10
)

# Autonomous Mode Enable/Disable
self.mode_sub = self.create_subscription(
    Bool, '/autonomous_mode', self.mode_callback, 10
)
```

### Publishers
```python
# Velocity Commands
self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

# Water Pump Control
self.pump_pub = self.create_publisher(Bool, '/water_pump', 10)

# Status Messages
self.status_pub = self.create_publisher(String, '/autonomous_status', 10)
```

### Callbacks
```python
def telemetry_callback(self, msg):
    """Receive sensor data from ESP32"""
    try:
        data = json.loads(msg.data)
        self.sensor_data = data
        
        # Check for fire indicators
        mq2_high = data.get('mq2', 0) > self.fire_threshold_mq2
        mq5_high = data.get('mq5', 0) > self.fire_threshold_mq5
        temp_rise = (data.get('temp', 25) - self.baseline_temp) > self.temp_rise_threshold
        flame = data.get('flame', 0) == 0  # Flame sensor is active LOW
        
        self.fire_detected = mq2_high or mq5_high or temp_rise or flame
    except json.JSONDecodeError:
        pass

def lidar_callback(self, msg):
    """Process LIDAR data for obstacle avoidance"""
    self.lidar_ranges = msg.ranges
    num_readings = len(msg.ranges)
    if num_readings == 0:
        return
    
    # Front sector: -30° to +30°
    front_start = int(num_readings * (330/360))
    front_end = int(num_readings * (30/360))
    front_ranges = list(msg.ranges[front_start:]) + list(msg.ranges[:front_end])
    front_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]
    self.min_front_distance = min(front_ranges) if front_ranges else float('inf')
    
    # Left sector: 30° to 90°
    left_start = int(num_readings * (30/360))
    left_end = int(num_readings * (90/360))
    left_ranges = [r for r in msg.ranges[left_start:left_end] if msg.range_min < r < msg.range_max]
    self.min_left_distance = min(left_ranges) if left_ranges else float('inf')
    
    # Right sector: 270° to 330°
    right_start = int(num_readings * (270/360))
    right_end = int(num_readings * (330/360))
    right_ranges = [r for r in msg.ranges[right_start:right_end] if msg.range_min < r < msg.range_max]
    self.min_right_distance = min(right_ranges) if right_ranges else float('inf')
    
    # Determine obstacle direction
    if self.min_front_distance < self.min_obstacle_distance:
        self.obstacle_detected = True
        if self.min_left_distance > self.min_right_distance:
            self.obstacle_direction = 'turn_left'
        else:
            self.obstacle_direction = 'turn_right'
    else:
        self.obstacle_detected = False
        self.obstacle_direction = None

def aruco_callback(self, msg):
    """Receive ArUco marker pose and store for navigation use"""
    try:
        self.aruco_pose = msg
        self.get_logger().debug(f'Received ArUco pose: x={msg.position.x:.2f}, y={msg.position.y:.2f}, z={msg.position.z:.2f}')
    except Exception:
        self.aruco_pose = None

def mode_callback(self, msg):
    """Enable/disable autonomous mode"""
    self.autonomous_enabled = msg.data
    if self.autonomous_enabled:
        self.get_logger().info('🤖 Autonomous mode ENABLED')
    else:
        self.get_logger().info('🛑 Autonomous mode DISABLED')
        self.stop_robot()
```

---

## 3. Camera Node (`frr_sensors/camera_node.py`)

### Publishers
```python
# Raw Camera Image
self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

# Compressed Camera Image (for bandwidth efficiency)
self.compressed_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)

# ArUco Marker Pose
self.aruco_pose_pub = self.create_publisher(Pose, '/aruco/pose', 10)
```

### No Subscribers (Camera publishes only)

---

## 4. ESP32 Bridge Node (`frr_control/esp32_bridge_node.py`)

### Subscribers
```python
# Velocity Commands
self.cmd_vel_sub = self.create_subscription(
    Twist, '/cmd_vel', self.cmd_vel_callback, 10
)

# Camera Tilt Control
self.servo_sub = self.create_subscription(
    Float64, '/camera_tilt', self.servo_callback, 10
)

# Water Pump Control
self.pump_sub = self.create_subscription(
    Bool, '/water_pump', self.pump_callback, 10
)

# Fire Scanning Mode
self.scan_sub = self.create_subscription(
    Bool, '/fire_scan', self.scan_callback, 10
)
```

### Publishers
```python
# Motor Status
self.status_pub = self.create_publisher(String, '/motor_status', 10)

# ESP32 Telemetry (all sensors)
self.telemetry_pub = self.create_publisher(String, '/esp32_telemetry', 10)

# Smoke Sensor (MQ2)
self.smoke_pub = self.create_publisher(Range, '/sensors/smoke', 10)

# Gas Sensor (MQ5)
self.gas_pub = self.create_publisher(Range, '/sensors/gas', 10)

# Flame Sensor
self.flame_pub = self.create_publisher(Bool, '/sensors/flame', 10)

# Temperature Sensor
self.temp_pub = self.create_publisher(Temperature, '/sensors/temperature', 10)
```

### Callbacks
```python
def cmd_vel_callback(self, msg):
    """Handle incoming velocity commands - convert to ESP32 motor commands"""
    linear_vel = msg.linear.x  # -1.0 to 1.0
    angular_vel = msg.angular.z  # -1.0 to 1.0
    
    # Convert to differential drive
    left_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
    right_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)
    
    # Normalize to -1.0 to 1.0
    max_vel = max(abs(left_vel), abs(right_vel))
    if max_vel > 1.0:
        left_vel /= max_vel
        right_vel /= max_vel
    
    # Convert to PWM (0-100)
    left_pwm = int(left_vel * 100)
    right_pwm = int(right_vel * 100)
    
    if left_pwm == 0 and right_pwm == 0:
        self.send_command("STOP")
    else:
        self.send_command(f"DRIVE {left_pwm} {right_pwm}")
    
    self.current_left_speed = left_vel
    self.current_right_speed = right_vel

def servo_callback(self, msg):
    """Handle camera tilt servo commands"""
    angle = int(msg.data)
    # Map -90 to 90 degrees to servo range 30-150
    servo_angle = int((angle + 90) * (150 - 30) / 180 + 30)
    servo_angle = max(30, min(150, servo_angle))
    self.send_command(f"SERVO {servo_angle}")

def pump_callback(self, msg):
    """Handle water pump control"""
    if msg.data:
        self.send_command("PUMP_ON")
    else:
        self.send_command("PUMP_OFF")

def scan_callback(self, msg):
    """Handle fire scanning mode"""
    if msg.data:
        self.send_command("SCAN")
        self.scanning_enabled = True
    else:
        self.send_command("DISABLE")
        self.scanning_enabled = False
```

---

## 5. Teleop Node (`frr_control/teleop_node_clean.py`)

### Subscribers
```python
# Obstacle Detection (optional, only if with_avoidance=True)
self.obstacle_sub = self.create_subscription(
    Bool, '/obstacle_detected', self.obstacle_callback, 10
)
```

### Publishers
```python
# Velocity Commands (varies based on mode)
if self.with_avoidance:
    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 1)
else:
    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

# Camera Tilt Control
self.camera_tilt_pub = self.create_publisher(Float64, '/camera_tilt', 1)
```

### Callbacks
```python
def obstacle_callback(self, msg):
    """Update obstacle status"""
    self.obstacle_detected = msg.data
```

---

## 6. Simple Autonomous Node (`frr_navigation/simple_autonomous_node.py`)

### Subscribers
```python
# LIDAR Scan
self.scan_sub = self.create_subscription(
    LaserScan, '/scan', self.scan_callback, 10
)

# ArUco Marker
self.aruco_sub = self.create_subscription(
    Pose, '/aruco/pose', self.aruco_callback, 10
)

# ESP32 Telemetry
self.telemetry_sub = self.create_subscription(
    String, '/esp32_telemetry', self.telemetry_callback, 10
)

# Sensor Fusion Pose
self.fusion_sub = self.create_subscription(
    PoseStamped, '/sensor_fusion/pose', self.fusion_callback, 10
)
```

### Publishers
```python
# Velocity Commands
self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
```

### Callbacks
```python
def scan_callback(self, msg):
    """Process LIDAR scan data"""
    self.last_scan = msg

def aruco_callback(self, msg):
    """Process ArUco marker detection"""
    self.aruco_detected = True
    self.aruco_pose = msg
    
    self.get_logger().info(
        f'ArUco marker detected! Position: '
        f'x={msg.position.x:.2f}m, y={msg.position.y:.2f}m, z={msg.position.z:.2f}m'
    )
    
    if not self.approaching_marker:
        self.approaching_marker = True
        self.approach_start_time = time.time()
        self.get_logger().info('Starting ArUco marker approach!')

def telemetry_callback(self, msg):
    """Process ESP32 telemetry for fire detection"""
    try:
        data = json.loads(msg.data)
        mq2 = data.get('mq2', 0)
        mq5 = data.get('mq5', 0)
        flame = data.get('flame', 0)
        
        # Simple fire detection
        if mq2 > 600 or mq5 > 550 or flame < 100:
            if not self.fire_detected:
                self.get_logger().warn('FIRE DETECTED!')
            self.fire_detected = True
        else:
            self.fire_detected = False
    except Exception:
        pass

def fusion_callback(self, msg):
    """Receive fused pose from sensor fusion node"""
    self.fusion_pose = msg.pose
    self.get_logger().debug(
        f'Received fused pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
    )
```

---

## 7. MPU6050 IMU Node (`frr_sensors/mpu6050_node_robust.py`)

### Publishers
```python
# IMU Data
self.imu_pub = self.create_publisher(Imu, '/imu/mpu6050', 10)

# Raw IMU Data
self.raw_pub = self.create_publisher(Float32MultiArray, '/imu/mpu6050_raw', 10)
```

---

## 8. LIDAR Node (`frr_sensors/lidar_node.py`)

### Publishers
```python
# LIDAR Scan
self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

# Raw LIDAR Data
self.raw_pub = self.create_publisher(Float32MultiArray, '/lidar/raw', 10)
```

---

## 9. Video Streamer Node (`frr_video/streamer_node.py`)

### Subscribers
```python
# Camera Image
self.image_sub = self.create_subscription(
    Image, '/camera/image_raw', self.image_callback, 10
)
```

### Callback
```python
def image_callback(self, msg):
    """Receive camera images and stream via web server"""
    # Convert ROS image to OpenCV
    # Encode to JPEG and stream
```

---

## Quick Reference Table

| Topic | Message Type | Publishers | Subscribers |
|-------|--------------|------------|-------------|
| `/cmd_vel` | Twist | teleop, autonomous_firebot, simple_autonomous | esp32_bridge |
| `/scan` | LaserScan | lidar_node | autonomous_firebot, simple_autonomous |
| `/aruco/pose` | Pose | camera_node | autonomous_firebot, simple_autonomous |
| `/camera/image_raw` | Image | camera_node | streamer_node |
| `/camera/image_raw/compressed` | CompressedImage | camera_node | - |
| `/esp32_telemetry` | String | esp32_bridge | sensor_fusion, autonomous_firebot, simple_autonomous |
| `/imu/mpu6050` | Imu | mpu6050_node | sensor_fusion |
| `/odom` | Odometry | sensor_fusion | - |
| `/sensor_fusion/pose` | PoseStamped | sensor_fusion | simple_autonomous |
| `/camera_tilt` | Float64 | teleop | esp32_bridge |
| `/water_pump` | Bool | autonomous_firebot | esp32_bridge |
| `/autonomous_mode` | Bool | - | autonomous_firebot |
| `/autonomous_status` | String | autonomous_firebot | - |
| `/obstacle_detected` | Bool | - | teleop (if avoidance enabled) |
| `/sensors/smoke` | Range | esp32_bridge | - |
| `/sensors/gas` | Range | esp32_bridge | - |
| `/sensors/flame` | Bool | esp32_bridge | - |
| `/sensors/temperature` | Temperature | esp32_bridge | - |

---

## Example Usage Patterns

### Pattern 1: Obstacle Avoidance with Sensor Fusion
```python
# Subscribe to LIDAR scan
self.scan_sub = self.create_subscription(
    LaserScan, '/scan', self.scan_callback, 10
)

def scan_callback(self, msg):
    # Process ranges
    front_ranges = msg.ranges[front_start:front_end]
    min_distance = min(front_ranges)
    
    if min_distance < obstacle_threshold:
        # Obstacle detected - take action
        self.avoid_obstacle()
```

### Pattern 2: Fire Detection and Response
```python
# Subscribe to ESP32 telemetry
self.telemetry_sub = self.create_subscription(
    String, '/esp32_telemetry', self.telemetry_callback, 10
)

def telemetry_callback(self, msg):
    data = json.loads(msg.data)
    mq2 = data.get('mq2', 0)
    flame = data.get('flame', 0)
    
    if mq2 > 600 or flame == 0:
        # Fire detected
        self.activate_water_pump()
```

### Pattern 3: Camera-Based Navigation
```python
# Subscribe to ArUco pose
self.aruco_sub = self.create_subscription(
    Pose, '/aruco/pose', self.aruco_callback, 10
)

def aruco_callback(self, msg):
    # Get marker position
    x = msg.position.x
    z = msg.position.z  # Distance
    
    # Calculate steering
    angular_velocity = -0.5 * x  # Proportional control
    
    # Publish movement command
    twist = Twist()
    twist.linear.x = 0.3
    twist.angular.z = angular_velocity
    self.cmd_vel_pub.publish(twist)
```

---

## Notes

- All nodes use queue size of 10 for subscribers/publishers (reliable communication)
- Teleop uses queue size of 1 (low latency, discard old commands)
- Callbacks should be fast and non-blocking
- Heavy processing should be done in separate threads or timers
- Use `self.get_logger()` for debugging inside callbacks
