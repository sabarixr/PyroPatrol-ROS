#!/usr/bin/env python3
"""
Mission Controller Node

Lightweight state machine to coordinate modes for the robot:
 - IDLE, SLAM_MAPPING, FOLLOW_ARUCO, WAIT_USER_TAKEOVER, USER_TAKEOVER,
   AUTONOMOUS_FIRE, FIRE_HANDLING, ESTOP

NEW ARCHITECTURE (Nov 2025):
 - ESP32 handles: MQ2, MQ5, TMP117, flame sensor, scan servo, turret servo, water pump, AND MOTORS
 - Raspberry Pi: sends motor commands to ESP32, reads LIDAR for T-junction detection, runs SLAM, ArUco, camera
 - Pi passes motor commands (LEFT/RIGHT speed or turn commands) to ESP32 via /esp32_command
 - ESP32 continuously runs SCAN_FIRE unless directional SCAN is requested
 - Mission controller reads ESP32 telemetry and sends SCAN commands when LIDAR detects T-junction
 - Autonomous fire mode: Pi commands forward movement via ESP32, LIDAR detects T-junctions, SCAN at junctions, approach fire

Interfaces (topics/services):
 - subscribe: /aruco/pose (Pose), /scan (LaserScan), /esp32_telemetry (String), /mission/command (String)
 - publish:  /cmd_vel (Twist), /mission/status (String), /mission/telemetry (String), /mission/obstacle_warning (String), /esp32_command (String)
 - services: /mission/takeover (Trigger), /mission/estop (Trigger)

This file is intentionally minimal and uses JSON on /mission/command for easy Flutter integration.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import json
import math
import time


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # State
        self.state = 'IDLE'

        # Telemetry
        self.last_aruco_pose = None
        self.last_scan = None
        self.obstacles = {'front': float('inf'), 'left': float('inf'), 'right': float('inf')}

        # Ultrasonic sensor (backup when LIDAR unavailable)
        self.ultrasonic_distance = float('inf')
        self.lidar_available = False
        self.last_lidar_time = 0

        # ESP32 fire sensor telemetry
        self.esp32_data = {
            'mq2': 0,
            'mq5': 0,
            'temp': 25.0,
            'flame': False,
            'scan_angle': 90,
            'pump_active': False,
            'scan_result': None,
            'scan_direction': None
        }
        self.last_flame_detection = 0
        self.fire_detected = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.telemetry_pub = self.create_publisher(String, '/mission/telemetry', 10)
        self.obstacle_pub = self.create_publisher(String, '/mission/obstacle_warning', 10)
        self.esp32_cmd_pub = self.create_publisher(String, '/esp32_command', 10)

        # Subscribers
        self.create_subscription(Pose, '/aruco/pose', self.aruco_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(String, '/mission/command', self.command_callback, 10)
        self.create_subscription(String, '/esp32_telemetry', self.esp32_callback, 10)
        self.create_subscription(String, '/mission/fire_perception', self.fire_perception_callback, 10)
        self.create_subscription(Range, '/ultrasonic/front', self.ultrasonic_callback, 10)

        # Services
        self.takeover_srv = self.create_service(Trigger, '/mission/takeover', self.handle_takeover)
        self.estop_srv = self.create_service(Trigger, '/mission/estop', self.handle_estop)

        # Timer for telemetry & safety checks
        self.create_timer(0.5, self.publish_telemetry)
        self.create_timer(0.1, self.autonomous_fire_update)

        # Controller gains & limits for ArUco follower
        self.aruco_goal_distance = 0.5  # meters
        self.max_linear = 0.5
        self.max_angular = 1.0
        self.kp_linear = 0.8
        self.kp_angular = 1.5

        # ArUco marker task tracking
        self.detected_marker_id = None
        self.aruco_task_executed = False
        self.aruco_marker_tasks = {
            1: 'TURN_LEFT',
            2: 'TURN_RIGHT',
            3: 'MOVE_FORWARD',
            4: 'ACTIVATE_PUMP',
            5: 'STOP'
        }

        # Autonomous fire mode state
        self.waiting_for_scan = False
        self.scan_request_time = 0
        self.last_tjunction_check = 0
        self.last_direction_inversion_warned = 0
        self.reversing_before_scan = False  # NEW: Track if currently reversing to reach 50cm
        self.reverse_start_time = 0
        self.autonomous_speed_multiplier = 0.5  # Default 50% speed (range 0.0-1.0)
        # Ready handshake tracking
        self.declare_parameter('send_ready_on_start', True)
        self.ready_sent = False

        # Optional: send READY handshake once shortly after start so ESP32 unlocks motors/pump
        try:
            send_ready = self.get_parameter('send_ready_on_start').get_parameter_value().bool_value
        except Exception:
            send_ready = True
        if send_ready:
            # create a periodic timer that will perform a one-time send
            self.create_timer(1.0, self._send_ready_handshake)

        self.get_logger().info('Mission controller started (NEW ARCHITECTURE - ESP32 fire sensors + ArUco tasks)')

    # -------------------- Callbacks --------------------
    def esp32_callback(self, msg: String):
        """Receive ESP32 fire sensor telemetry and scan results."""
        try:
            data = json.loads(msg.data)
            msg_type = data.get('type')

            if msg_type == 'scan_sample':
                # Continuous sensor readings during scan
                self.esp32_data['mq2'] = data.get('mq2', 0)
                self.esp32_data['mq5'] = data.get('mq5', 0)
                self.esp32_data['temp'] = data.get('temp', 25.0)
                self.esp32_data['scan_angle'] = data.get('angle', 90)

            elif msg_type == 'scan_result':
                # Result from endpoint sampling
                self.esp32_data['scan_result'] = data
                self.get_logger().info(f"Scan result: angle={data.get('angle')}, score={data.get('score'):.3f}, dominant={data.get('dominant')}")

            elif msg_type == 'scan_complete':
                # Handled by AI Perception Node now
                pass

            elif msg_type == 'alert':
                # Flame detected by ESP32
                self.fire_detected = True
                self.last_flame_detection = time.time()
                self.get_logger().warn(f"FIRE ALERT from ESP32: {data.get('msg')}")

                # If in autonomous fire mode, stop and let ESP32 handle pump
                if self.state == 'AUTONOMOUS_FIRE':
                    self.stop_robot()
                    self.publish_status('Fire detected - ESP32 activating pump')

            elif msg_type == 'pump':
                self.esp32_data['pump_active'] = (data.get('status') == 'on')

        except Exception as e:
            self.get_logger().warn(f'Failed to parse ESP32 telemetry: {e}')

    def fire_perception_callback(self, msg: String):
        """Receive fused AI prediction (Direction & Severity) from fire perception node."""
        try:
            data = json.loads(msg.data)
            direction = data.get('direction', 'none')
            severity = data.get('severity', 0.0)

            self.waiting_for_scan = False
            self.get_logger().info(f"AI Perception: {direction.upper()} (Severity: {severity:.2f})")

            if self.state == 'AUTONOMOUS_FIRE':
                if direction != 'none':
                    self.get_logger().info(f"AI says '{direction}' → Turning robot {direction}")
                    self.execute_turn(direction)

                    # Fire Severity Pump Control
                    if direction == 'front' and severity > 0.6:
                        self.get_logger().info(f"Fire is FRONT and Severity is high ({severity:.2f}) -> ACTIVATING PUMP!")
                        self.send_esp32_command('PUMP_ON')

                        # Auto-stop pump after 5 seconds to re-evaluate
                        import threading
                        threading.Timer(5.0, lambda: self.send_esp32_command('PUMP_OFF')).start()
                else:
                    self.execute_turn('none')

        except Exception as e:
            self.get_logger().warn(f'Failed to parse AI fire perception: {e}')

    def aruco_callback(self, msg: Pose):
        """Receive pose of detected ArUco marker (camera frame)."""
        self.last_aruco_pose = msg

        if self.state == 'FOLLOW_ARUCO':
            # Check if marker has an ID attached (assuming it's in orientation.w or custom field)
            # For now, we'll simulate marker ID detection
            # In real implementation, you'd get this from aruco_detector node

            # Execute ArUco marker task if close enough
            if hasattr(msg.position, 'z') and msg.position.z is not None:
                distance = msg.position.z

                # If very close to marker (< 0.4m), execute its task
                if distance < 0.4 and not self.aruco_task_executed:
                    # Simulate marker ID (in real system, this comes from aruco detector)
                    # For testing, cycle through IDs based on time
                    import random
                    if self.detected_marker_id is None:
                        self.detected_marker_id = random.randint(1, 5)

                    self.execute_aruco_task(self.detected_marker_id)
                    self.aruco_task_executed = True

                    # After task, wait for user takeover
                    self.set_state('WAIT_USER_TAKEOVER')
                    self.publish_status(f'ArUco marker {self.detected_marker_id} task complete - awaiting takeover')

                elif distance < 0.6:
                    # Getting close - follow the marker
                    self.follow_aruco(msg)
                else:
                    # Far away - follow the marker
                    self.follow_aruco(msg)

                    # Reset task execution flag when far from marker
                    if distance > 1.0:
                        self.aruco_task_executed = False
                        self.detected_marker_id = None

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg
        self.lidar_available = True
        self.last_lidar_time = time.time()

        # simple sector checks (front: -15..15 deg, left: 60..120, right: -120..-60)
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        n = len(ranges)

        def sector_min(start_deg, end_deg):
            start = int((math.radians(start_deg) - angle_min) / angle_increment)
            end = int((math.radians(end_deg) - angle_min) / angle_increment)
            start = max(0, min(n - 1, start))
            end = max(0, min(n - 1, end))
            if end < start:
                start, end = end, start
            vals = [r for r in ranges[start:end + 1] if r > 0]
            return min(vals) if vals else float('inf')

        try:
            front = sector_min(-15, 15)
            left = sector_min(45, 135)
            right = sector_min(-135, -45)
        except Exception:
            front = left = right = float('inf')

        self.obstacles = {'front': float(front), 'left': float(left), 'right': float(right)}

    def ultrasonic_callback(self, msg: Range):
        """Receive ultrasonic sensor data (backup when LIDAR unavailable)"""
        if msg.range < msg.max_range and msg.range > msg.min_range:
            self.ultrasonic_distance = msg.range
        else:
            self.ultrasonic_distance = float('inf')

        # Publish a simple warning message. Use ultrasonic when LIDAR not available.
        warn = None
        thresh = 0.35

        # Prefer LIDAR obstacles if available, otherwise use ultrasonic for front
        front_val = self.obstacles.get('front', float('inf')) if self.lidar_available else self.ultrasonic_distance
        left_val = self.obstacles.get('left', float('inf'))
        right_val = self.obstacles.get('right', float('inf'))

        if front_val < thresh:
            warn = 'front'
        elif left_val < thresh:
            warn = 'left'
        elif right_val < thresh:
            warn = 'right'

        warning_msg = {'front': front_val, 'left': left_val, 'right': right_val, 'warning': warn}
        self.obstacle_pub.publish(String(data=json.dumps(warning_msg)))

        # If in autonomous or follow mode and obstacle in front, stop
        if warn and self.state in ['AUTONOMOUS_NAV', 'FOLLOW_ARUCO']:
            self.get_logger().warn(f'Obstacle detected in {warn} — stopping')
            self.stop_robot()

    def command_callback(self, msg: String):
        """Receive JSON commands from app. Example: {"cmd":"set_mode","mode":"FOLLOW_ARUCO"}"""
        try:
            data = json.loads(msg.data)
            cmd = data.get('cmd')

            if cmd == 'set_mode':
                mode = data.get('mode', 'IDLE')
                self.set_mode_from_string(mode)
            elif cmd == 'start_slam':
                self.set_state('SLAM_MAPPING')
                self.publish_status('SLAM mapping started (teleop to map)')
            elif cmd == 'stop_slam':
                self.set_state('IDLE')
                self.publish_status('SLAM mapping stopped')
            elif cmd == 'start_fire_mode':
                # Start autonomous firefighting mode
                self.set_state('AUTONOMOUS_FIRE')
                # Send SCAN_FIRE command to ESP32
                self.send_esp32_command('SCAN_FIRE')
                self.publish_status('Autonomous fire mode started')
            elif cmd == 'stop_fire_mode':
                self.set_state('IDLE')
                self.send_esp32_command('DISABLE')
                self.stop_robot()
                self.publish_status('Autonomous fire mode stopped')
            elif cmd == 'scan':
                # Manual scan request from app
                self.send_esp32_command('SCAN')
                self.publish_status('Directional scan requested')
            elif cmd == 'set_speed':
                # Autonomous speed control from app
                speed = data.get('speed', 0.5)
                self.autonomous_speed_multiplier = max(0.2, min(1.0, speed))  # Clamp 0.2-1.0
                self.get_logger().info(f'Autonomous speed set to {self.autonomous_speed_multiplier*100:.0f}%')
                self.publish_status(f'Speed: {self.autonomous_speed_multiplier*100:.0f}%')
            elif cmd == 'pump':
                # Manual pump control (forward to ESP32)
                desired = data.get('on', False)
                self.send_esp32_command('PUMP_ON' if desired else 'PUMP_OFF')
                self.publish_status(f'Pump command sent: {"ON" if desired else "OFF"}')
            elif cmd == 'takeover':
                # Allow takeover via mission command (from WS bridge) — reuse service-style safety check
                front = self.obstacles.get('front', float('inf'))
                if front < 0.3:
                    self.publish_status(f'Unsafe to takeover - obstacle {front:.2f}m in front')
                else:
                    self.set_state('USER_TAKEOVER')
                    self.send_esp32_command('DISABLE')  # Stop ESP32 scanning
                    self.publish_status('User takeover granted')
            else:
                self.get_logger().warn(f'Unknown mission command: {cmd}')

        except Exception as e:
            self.get_logger().warn(f'Invalid mission command payload: {e}')

    # -------------------- Actions --------------------
    def send_esp32_command(self, cmd: str):
        """Send command to ESP32 via /esp32_command topic."""
        msg = String()
        msg.data = cmd
        self.esp32_cmd_pub.publish(msg)
        self.get_logger().info(f'Sent to ESP32: {cmd}')

    def invert_direction(self, esp_direction: str):
        """Invert ESP32 scan direction to robot's actual direction.
        ESP32's left sensor = Robot's right side.
        ESP32's right sensor = Robot's left side.
        """
        if esp_direction == 'left':
            return 'right'  # ESP32 left = robot right
        elif esp_direction == 'right':
            return 'left'   # ESP32 right = robot left
        else:
            return esp_direction  # center or none

    def execute_turn(self, robot_direction: str):
        """Execute turn based on AI direction - send command to ESP32."""
        if robot_direction == 'left':
            self.send_esp32_command('DRIVE -100 100')  # Turn robot left at FULL speed (don't scale turns)
            self.get_logger().info('Commanding ESP32 to turn robot LEFT toward fire')
            # Resume forward movement after turn
            import threading
            threading.Timer(1.5, lambda: self.resume_forward_after_turn()).start()
        elif robot_direction == 'right':
            self.send_esp32_command('DRIVE 100 -100')  # Turn robot right at FULL speed (don't scale turns)
            self.get_logger().info('Commanding ESP32 to turn robot RIGHT toward fire')
            # Resume forward movement after turn
            import threading
            threading.Timer(1.5, lambda: self.resume_forward_after_turn()).start()
        else:
            forward_speed = int(-40 * self.autonomous_speed_multiplier)
            self.get_logger().info('➡️  No significant fire direction, continuing forward')
            self.send_esp32_command(f'DRIVE {forward_speed} {forward_speed}')  # Continue forward with speed control

    def resume_forward_after_turn(self):
        """Resume forward movement after completing a turn."""
        if self.state == 'AUTONOMOUS_FIRE':
            forward_speed = int(-40 * self.autonomous_speed_multiplier)
            self.get_logger().info('[OK] Turn complete, resuming forward movement')
            self.send_esp32_command(f'DRIVE {forward_speed} {forward_speed}')  # Forward with speed control

    def execute_aruco_task(self, marker_id: int):
        """Execute predefined task for ArUco marker ID."""
        task = self.aruco_marker_tasks.get(marker_id, 'UNKNOWN')

        self.get_logger().info(f'📍 Executing ArUco Marker {marker_id} task: {task}')

        if task == 'TURN_LEFT':
            self.send_esp32_command('DRIVE -80 80')  # Turn left
            self.publish_status(f'Marker {marker_id}: Turning Left')
        elif task == 'TURN_RIGHT':
            self.send_esp32_command('DRIVE 80 -80')  # Turn right
            self.publish_status(f'Marker {marker_id}: Turning Right')
        elif task == 'MOVE_FORWARD':
            self.send_esp32_command('DRIVE -60 -60')  # Forward (NEGATIVE = FORWARD)
            self.publish_status(f'Marker {marker_id}: Moving Forward')
            # Continue forward for 2 seconds
            import threading
            threading.Timer(2.0, lambda: self.send_esp32_command('STOP')).start()
        elif task == 'ACTIVATE_PUMP':
            self.send_esp32_command('PUMP_ON')
            self.publish_status(f'Marker {marker_id}: Activating Pump')
            # Turn off pump after 5 seconds
            import threading
            threading.Timer(5.0, lambda: self.send_esp32_command('PUMP_OFF')).start()
        elif task == 'STOP':
            self.send_esp32_command('STOP')
            self.publish_status(f'Marker {marker_id}: Stopping Robot')
        else:
            self.get_logger().warn(f'Unknown marker ID: {marker_id}')

    def detect_tjunction(self):
        """Detect if robot is at a T-junction using LIDAR."""
        front = self.obstacles.get('front', float('inf'))
        left = self.obstacles.get('left', float('inf'))
        right = self.obstacles.get('right', float('inf'))

        # T-junction: front blocked, but left or right open
        if front < 0.5 and (left > 1.0 or right > 1.0):
            return True
        return False

    def autonomous_fire_update(self):
        """Autonomous fire mode behavior loop."""
        if self.state != 'AUTONOMOUS_FIRE':
            return

        now = time.time()

        # Check if LIDAR is available (received data in last 2 seconds)
        lidar_active = (now - self.last_lidar_time) < 2.0

        # Choose obstacle detection source
        if lidar_active:
            front = self.obstacles.get('front', float('inf'))
            left = self.obstacles.get('left', float('inf'))
            right = self.obstacles.get('right', float('inf'))
            sensor_type = 'LIDAR'
        else:
            # Fallback to ultrasonic (front only)
            front = self.ultrasonic_distance
            left = float('inf')  # Ultrasonic can't detect sides
            right = float('inf')
            sensor_type = 'ULTRASONIC'

            # Log once when switching to ultrasonic
            if not hasattr(self, '_ultrasonic_mode_logged'):
                self.get_logger().warn('[!] LIDAR unavailable - using ULTRASONIC sensor for front obstacle detection')
                self._ultrasonic_mode_logged = True

        # If currently reversing to reach desired scan distance (50cm)
        if self.reversing_before_scan:
            # Check if we've reached the target distance or timeout
            if front >= 0.45 and front <= 0.55:
                # Good distance - stop and scan
                self.get_logger().info(f'[OK] Reached scan distance ({front:.2f}m), requesting SCAN')
                self.send_esp32_command('STOP')
                self.send_esp32_command('SCAN')
                self.reversing_before_scan = False
                self.waiting_for_scan = True
                self.scan_request_time = now
                return
            elif now - self.reverse_start_time > 3.0:
                # Timeout - scan anyway
                self.get_logger().warn(f'Reverse timeout, scanning at current distance ({front:.2f}m)')
                self.send_esp32_command('STOP')
                self.send_esp32_command('SCAN')
                self.reversing_before_scan = False
                self.waiting_for_scan = True
                self.scan_request_time = now
                return
            else:
                # Continue reversing with speed multiplier
                reverse_speed = int(40 * self.autonomous_speed_multiplier)
                self.send_esp32_command(f'DRIVE {reverse_speed} {reverse_speed}')
                return

        # If waiting for scan result, don't move
        if self.waiting_for_scan:
            if now - self.scan_request_time > 10.0:
                # Timeout - resume movement
                self.waiting_for_scan = False
                self.get_logger().warn('Scan timeout, resuming forward movement')
            else:
                return

        # LIDAR MODE: Check for T-junction every 2 seconds
        if lidar_active and now - self.last_tjunction_check > 2.0:
            self.last_tjunction_check = now

            if self.detect_tjunction():
                # Check distance and reverse if too close
                if front < 0.4:
                    self.get_logger().info(f'🔍 T-junction detected at {front:.2f}m (too close), reversing to 50cm before SCAN')
                    reverse_speed = int(40 * self.autonomous_speed_multiplier)
                    self.send_esp32_command(f'DRIVE {reverse_speed} {reverse_speed}')  # Reverse
                    self.reversing_before_scan = True
                    self.reverse_start_time = now
                else:
                    self.get_logger().info('🔍 T-junction detected by LIDAR, requesting directional scan from ESP32')
                    self.send_esp32_command('STOP')
                    self.send_esp32_command('SCAN')
                    self.waiting_for_scan = True
                    self.scan_request_time = now
                return

        # Check for front obstacle that requires scanning
        obstacle_detected = front < 0.5  # 50cm threshold

        if obstacle_detected:
            # Check if we're too close (< 40cm) - reverse first
            if front < 0.4:
                self.get_logger().info(f'🔍 Obstacle at {front:.2f}m ({sensor_type}), reversing to 50cm before SCAN')
                reverse_speed = int(40 * self.autonomous_speed_multiplier)
                self.send_esp32_command(f'DRIVE {reverse_speed} {reverse_speed}')  # Reverse
                self.reversing_before_scan = True
                self.reverse_start_time = now
                return
            else:
                # Good distance - stop and scan
                self.get_logger().info(f'🔍 Obstacle at {front:.2f}m ({sensor_type}), requesting directional scan from ESP32')
                self.send_esp32_command('STOP')
                self.send_esp32_command('SCAN')
                self.waiting_for_scan = True
                self.scan_request_time = now
                return

        # Move forward if path is clear
        # Apply speed multiplier to forward movement
        if front > 0.6:
            forward_speed = int(-40 * self.autonomous_speed_multiplier)
            self.send_esp32_command(f'DRIVE {forward_speed} {forward_speed}')  # Forward at controlled speed
        elif front > 0.5:
            # Approaching obstacle - slow down even more
            slow_speed = int(-30 * self.autonomous_speed_multiplier)
            self.send_esp32_command(f'DRIVE {slow_speed} {slow_speed}')  # Forward slower
        else:
            # Within scan range - this should trigger obstacle_detected above
            self.send_esp32_command('STOP')

    def follow_aruco(self, pose: Pose):
        """Simple proportional follower using ArUco pose in camera frame.
        Expects pose.position.x (right), y (down), z (forward).
       """
        try:
            z = float(pose.position.z) if pose.position.z is not None else None
            x = float(pose.position.x) if pose.position.x is not None else 0.0
        except Exception:
            return

        if z is None or math.isinf(z) or z <= 0:
            return

        # Error in distance (positive if too far)
        dist_err = z - self.aruco_goal_distance
        linear = self.kp_linear * dist_err
        angular = -self.kp_angular * x / max(z, 0.001)

        # clamp
        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Stop robot by sending STOP command to ESP32."""
        self.send_esp32_command('STOP')

    # -------------------- Services --------------------
    def handle_takeover(self, request, response):
        """Give control to user if it's safe."""
        # safety check: no close obstacle in front
        front = self.obstacles.get('front', float('inf'))
        if front < 0.3:
            response.success = False
            response.message = f'Unsafe to takeover - obstacle {front:.2f}m in front'
            return response

        self.set_state('USER_TAKEOVER')
        self.publish_status('User takeover granted')
        response.success = True
        response.message = 'Takeover granted'
        return response

    def handle_estop(self, request, response):
        self.set_state('ESTOP')
        self.stop_robot()
        response.success = True
        response.message = 'E-STOP engaged'
        self.publish_status('E-STOP engaged')
        return response

    # -------------------- Helpers --------------------
    def set_mode_from_string(self, mode_str: str):
        mode = mode_str.strip().upper()
        # Map Flutter app modes to mission controller states
        mode_mapping = {
            'MANUAL': 'USER_TAKEOVER',
            'FIRE_SEEKING': 'AUTONOMOUS_FIRE',
            'ARUCO': 'FOLLOW_ARUCO',
            'SLAM_MAPPING': 'SLAM_MAPPING',
            'FOLLOW_ARUCO': 'FOLLOW_ARUCO',
            'AUTONOMOUS_NAV': 'AUTONOMOUS_NAV',
            'FIRE_HANDLING': 'AUTONOMOUS_FIRE',
            'AUTONOMOUS_FIRE': 'AUTONOMOUS_FIRE',
            'IDLE': 'IDLE',
            'USER_TAKEOVER': 'USER_TAKEOVER'
        }

        mapped_mode = mode_mapping.get(mode, mode)
        allowed = ['IDLE', 'SLAM_MAPPING', 'FOLLOW_ARUCO', 'USER_TAKEOVER', 'AUTONOMOUS_FIRE', 'AUTONOMOUS_NAV']

        if mapped_mode not in allowed:
            self.get_logger().warn(f'Attempt to set unknown mode: {mode} (mapped to {mapped_mode})')
            return

        # Enable/disable ESP32 scanning based on mode
        if mapped_mode == 'AUTONOMOUS_FIRE':
            self.send_esp32_command('SCAN_FIRE')
            self.get_logger().info('Fire Seeking Mode: SCAN_FIRE enabled')
        elif mapped_mode == 'FOLLOW_ARUCO':
            self.send_esp32_command('DISABLE')
            self.aruco_task_executed = False
            self.detected_marker_id = None
            self.get_logger().info('📍 ArUco Mode: Ready to detect markers')
        elif mapped_mode in ['IDLE', 'USER_TAKEOVER']:
            self.send_esp32_command('DISABLE')
            self.get_logger().info('🎮 Manual Mode: Scanning disabled')

        self.set_state(mapped_mode)
        self.publish_status(f'Mode set to {mapped_mode} (from {mode})')

    def set_state(self, new_state: str):
        self.get_logger().info(f'State {self.state} -> {new_state}')
        self.state = new_state

    def _send_ready_handshake(self):
        """Internal timer callback: send READY once to ESP32 to handshake/unlock motors.
        This is intentionally idempotent: it will only send once (ready_sent flag).
        """
        try:
            if self.ready_sent:
                return
            # Send READY command
            self.send_esp32_command('READY')
            self.get_logger().info('Sent READY handshake to ESP32 (mission_controller)')
            self.ready_sent = True
        except Exception as e:
            self.get_logger().warn(f'Failed to send READY handshake: {e}')

    def publish_status(self, text: str):
        msg = String()
        msg.data = json.dumps({'state': self.state, 'message': text})
        self.status_pub.publish(msg)

    def publish_telemetry(self):
        # Choose obstacle source: prefer LIDAR if available; otherwise use ultrasonic for front
        source_obstacles = dict(self.obstacles)
        if not self.lidar_available:
            # Replace front with ultrasonic reading when LIDAR not available
            source_obstacles['front'] = self.ultrasonic_distance

        # Convert infinity to None for JSON serialization
        obstacles_safe = {}
        for key, value in source_obstacles.items():
            if math.isinf(value) or math.isnan(value):
                obstacles_safe[key] = None
            else:
                obstacles_safe[key] = value

        data = {
            'state': self.state,
            'aruco_distance': self.last_aruco_pose.position.z if self.last_aruco_pose is not None else None,
            'aruco_marker_id': self.detected_marker_id,
            'obstacles': obstacles_safe,
            'esp32': {
                'mq2': self.esp32_data.get('mq2', 0),
                'mq5': self.esp32_data.get('mq5', 0),
                'temperature': self.esp32_data.get('temp', 25.0),
                'flame': self.fire_detected,
                'pump_active': self.esp32_data.get('pump_active', False),
                'scan_angle': self.esp32_data.get('scan_angle', 90),
                'scan_direction': self.esp32_data.get('scan_direction', None)
            }
        }
        try:
            self.telemetry_pub.publish(String(data=json.dumps(data)))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
