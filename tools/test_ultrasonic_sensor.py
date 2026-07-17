#!/usr/bin/env python3
"""
Simple ROS2 runtime test for the HC-SR04 ultrasonic sensor.

Usage:
  - Ensure ROS2 and your workspace are sourced and the ultrasonic node is running:
      . install/setup.bash
      ros2 run frr_sensors ultrasonic_node

  - Run this script while ROS2 is running:
      python3 tools/test_ultrasonic_sensor.py --timeout 10

Exits:
  0 = success (valid Range message received within timeout)
  2 = timeout / no valid message
  3 = runtime error

The script subscribes to `/ultrasonic/front` (sensor_msgs/Range) and considers a
reading valid when it's numeric, not infinite/NaN and inside the message's min/max.
"""
import sys
import math
import argparse

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Range
except Exception as e:
    print("ERROR: this script requires ROS2 python packages (rclpy, sensor_msgs).\nMake sure you run it on the Pi with ROS2 sourced.", file=sys.stderr)
    print(f"Details: {e}", file=sys.stderr)
    sys.exit(3)


class UltrasonicTest(Node):
    def __init__(self, timeout=10.0):
        super().__init__('ultrasonic_test')
        self.timeout = float(timeout)
        self.received = False
        self.valid = False
        self.msg = None
        self.sub = self.create_subscription(Range, '/ultrasonic/front', self._cb, 10)
        self.get_logger().info(f'Ultrasonic test node started, waiting up to {self.timeout:.1f}s for /ultrasonic/front')

    def _cb(self, msg: Range):
        self.get_logger().info(f'Received Range: {msg.range} (min {msg.min_range} max {msg.max_range})')
        self.received = True
        # Validate numeric and within min/max
        if (not math.isinf(msg.range)) and (not math.isnan(msg.range)):
            # Some implementations publish invalid min/max as 0 - treat that as permissive
            minr = msg.min_range if (msg.min_range and msg.min_range > 0) else 0.02
            maxr = msg.max_range if (msg.max_range and msg.max_range > 0) else 4.0
            if (msg.range >= minr) and (msg.range <= maxr):
                self.valid = True
                self.msg = msg


def main(argv=None):
    parser = argparse.ArgumentParser(description='Verify ultrasonic sensor publishes /ultrasonic/front')
    parser.add_argument('--timeout', type=float, default=10.0, help='Seconds to wait for a valid reading')
    args = parser.parse_args(argv)

    try:
        rclpy.init()
        node = UltrasonicTest(timeout=args.timeout)
        start = node.get_clock().now()
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.5)
                if node.valid:
                    print(f'OK: valid ultrasonic reading: {node.msg.range:.3f} m')
                    return 0
                # timeout
                now = node.get_clock().now()
                elapsed = (now - start).nanoseconds / 1e9
                if elapsed > args.timeout:
                    if node.received:
                        print('FAIL: messages received but no valid reading within sensor min/max', file=sys.stderr)
                    else:
                        print('FAIL: no /ultrasonic/front messages received (timeout)', file=sys.stderr)
                    return 2
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f'ERROR: test failed to run: {e}', file=sys.stderr)
        return 3


if __name__ == '__main__':
    sys.exit(main())
