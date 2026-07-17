#!/usr/bin/env python3
"""
WebSocket Bridge Node

Lightweight JSON WebSocket server that bridges between your Flutter app and ROS2 topics.

Protocol (simple):
- Client -> Server: send JSON messages (string) which will be re-published to `/mission/command` as a String.
- Server -> Client: forwards messages received on `/mission/telemetry` and `/mission/status` as-is (expects stringified JSON).

Notes:
- This is intentionally minimal. For video, use the existing streamer (HTTP/MJPEG) and load the URL in Flutter.
- Requires `websockets` Python package on the robot: pip install websockets
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import asyncio
import threading
import json
import time
import websockets
from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError


class WebSocketBridgeNode(Node):
    def __init__(self):
        super().__init__('ws_bridge_node')

        # Parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('failsafe_timeout', 2.0)  # seconds

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.failsafe_timeout = self.get_parameter('failsafe_timeout').get_parameter_value().double_value

        # ROS publishers/subscribers
        self.command_pub = self.create_publisher(String, '/mission/command', 10)
        self.esp32_cmd_pub = self.create_publisher(String, '/esp32_command', 10)
        self.tilt_pub = self.create_publisher(Float64, '/camera_tilt', 10)
        
        self.telemetry_sub = self.create_subscription(String, '/mission/telemetry', self.telemetry_cb, 10)
        self.status_sub = self.create_subscription(String, '/mission/status', self.status_cb, 10)
        self.esp32_sub = self.create_subscription(String, '/esp32_telemetry', self.esp32_cb, 10)
        self.obstacle_sub = self.create_subscription(String, '/mission/obstacle_warning', self.obstacle_cb, 10)

        # Connected websocket clients
        self._ws_clients = set()

        # Failsafe: track last command time
        self.last_cmd_time = time.time()
        self.failsafe_active = False
        
        # Create timer to check for failsafe
        self.failsafe_timer = self.create_timer(0.1, self.check_failsafe)  # 10Hz check

        # Asyncio loop in separate thread
        self.loop = asyncio.new_event_loop()
        self.server_thread = threading.Thread(target=self._start_loop, daemon=True)
        self.server_thread.start()

        # Start websocket server coroutine
        fut = asyncio.run_coroutine_threadsafe(self._start_server(), self.loop)
        # don't block here, but keep reference
        self.server_future = fut

        self.get_logger().info(f'WebSocket bridge listening on ws://{self.host}:{self.port}')

    def _start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def _start_server(self):
        async def handler(ws):
            await self._client_handler(ws)

        return await websockets.serve(handler, self.host, self.port)

    async def _client_handler(self, websocket):
        # Register
        self._ws_clients.add(websocket)
        client_info = f'{websocket.remote_address}'

        try:
            await websocket.send(json.dumps({'welcome': 'connected to ws bridge'}))

            async for message in websocket:
                # Update last command timestamp
                self.last_cmd_time = time.time()
                if self.failsafe_active:
                    self.failsafe_active = False
                    # Logging from async context causes crashes - skipped
                
                # Expect JSON string from client; route to topics or /mission/command
                try:
                    parsed = json.loads(message)
                except Exception:
                    parsed = message

                # If parsed is a dict and contains routing info, handle specially
                if isinstance(parsed, dict):
                    # Direct topic routing (teleop) - convert to ESP32 motor commands
                    topic = parsed.get('topic')
                    if topic == '/cmd_vel' and 'twist' in parsed:
                        try:
                            lin = parsed['twist'].get('linear', {})
                            ang = parsed['twist'].get('angular', {})
                            linear_x = float(lin.get('x', 0.0))
                            angular_z = float(ang.get('z', 0.0))
                            
                            # Convert twist to differential drive (ESP32 expects -100 to 100)
                            # Simple differential drive model
                            wheel_base = 0.2  # meters
                            max_speed = 100  # PWM value
                            
                            left_vel = linear_x - (angular_z * wheel_base / 2.0)
                            right_vel = linear_x + (angular_z * wheel_base / 2.0)
                            
                            # Normalize and scale to PWM
                            max_vel = max(abs(left_vel), abs(right_vel), 0.01)
                            if max_vel > 1.0:
                                left_vel /= max_vel
                                right_vel /= max_vel
                            
                            left_pwm = int(left_vel * max_speed)
                            right_pwm = int(right_vel * max_speed)
                            
                            # Send DRIVE command to ESP32
                            esp32_cmd = String()
                            if left_pwm == 0 and right_pwm == 0:
                                esp32_cmd.data = 'STOP'
                            else:
                                esp32_cmd.data = f'DRIVE {left_pwm} {right_pwm}'
                            self.esp32_cmd_pub.publish(esp32_cmd)
                            
                        except Exception:
                            pass  # Logging from async causes crashes
                        continue
                    
                    if topic == '/camera_tilt' and 'data' in parsed:
                        try:
                            v = Float64()
                            v.data = float(parsed.get('data', 0.0))
                            self.tilt_pub.publish(v)
                        except Exception:
                            pass
                        continue

                    if topic == '/esp32_command' and 'data' in parsed:
                        try:
                            s = String()
                            s.data = str(parsed.get('data'))
                            self.esp32_cmd_pub.publish(s)
                        except Exception:
                            pass
                        continue
                    
                    # Handle direct tilt command (new format from Flutter app)
                    if 'tilt' in parsed:
                        try:
                            tilt_msg = Float64()
                            tilt_msg.data = float(parsed['tilt'])
                            self.tilt_pub.publish(tilt_msg)
                        except Exception:
                            pass
                        continue
                    
                    # Otherwise forward as mission command
                    ros_msg = String()
                    ros_msg.data = json.dumps(parsed)
                    self.command_pub.publish(ros_msg)
                    continue

                # Fallback: publish raw string to /mission/command
                ros_msg = String()
                ros_msg.data = str(parsed)
                self.command_pub.publish(ros_msg)

        except (ConnectionClosedOK, ConnectionClosedError):
            pass  # Normal disconnect
        except Exception:
            pass  # Error but can't log from async
        finally:
            self._ws_clients.discard(websocket)

    def check_failsafe(self):
        """Stop robot if no commands received for failsafe_timeout seconds."""
        time_since_last_cmd = time.time() - self.last_cmd_time
        
        if time_since_last_cmd > self.failsafe_timeout and not self.failsafe_active:
            self.failsafe_active = True
            self.get_logger().warn(f'FAILSAFE ACTIVATED: No commands for {time_since_last_cmd:.1f}s - stopping robot')
            
            # Send STOP command to ESP32
            stop_cmd = String()
            stop_cmd.data = 'STOP'
            self.esp32_cmd_pub.publish(stop_cmd)
            
            # Send ESTOP to mission controller
            mission_cmd = String()
            mission_cmd.data = json.dumps({'cmd': 'estop', 'reason': 'app_timeout'})
            self.command_pub.publish(mission_cmd)

    def telemetry_cb(self, msg: String):
        # Forward telemetry to all connected clients
        payload = msg.data
        try:
            # Schedule broadcast on asyncio loop
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), self.loop)
        except Exception as e:
            self.get_logger().warn(f'Failed to schedule telemetry broadcast: {e}')

    def status_cb(self, msg: String):
        payload = msg.data
        try:
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), self.loop)
        except Exception as e:
            self.get_logger().warn(f'Failed to schedule status broadcast: {e}')
    
    def esp32_cb(self, msg: String):
        """Forward ESP32 telemetry directly to Flutter app."""
        payload = msg.data
        try:
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), self.loop)
        except Exception as e:
            self.get_logger().warn(f'Failed to schedule ESP32 broadcast: {e}')
    
    def obstacle_cb(self, msg: String):
        """Forward obstacle warnings to Flutter app."""
        payload = msg.data
        try:
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), self.loop)
        except Exception as e:
            self.get_logger().warn(f'Failed to schedule obstacle broadcast: {e}')

    async def _broadcast(self, payload: str):
        if not self._ws_clients:
            return
        dead = []
        for ws in list(self._ws_clients):
            try:
                await ws.send(payload)
            except Exception:
                dead.append(ws)

        for d in dead:
            self._ws_clients.discard(d)

    def destroy_node(self):
        # Close websockets server
        try:
            # Cancel server future if exists
            if hasattr(self, 'server_future'):
                self.server_future.cancel()
        except Exception:
            pass

        # Stop loop
        try:
            self.loop.call_soon_threadsafe(self.loop.stop)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
