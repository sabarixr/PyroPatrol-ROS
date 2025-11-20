#!/bin/bash
# Motor diagnostic script - helps identify motor direction issues

source ~/frr_ws/install/setup.bash

echo "========================================="
echo "  MOTOR DIAGNOSTICS & FIX HELPER"
echo "========================================="
echo
echo "This script will help you identify and fix motor direction issues."
echo

# Check if motor driver is running
if ! ros2 node list 2>/dev/null | grep -q "motor_driver_node"; then
    echo "❌ motor_driver_node is NOT running!"
    echo
    echo "Start it first with:"
    echo "  ./start_robot.sh"
    echo
    exit 1
fi

echo "✓ motor_driver_node is running"
echo

# Show current motor parameters
echo "Current Motor Parameters:"
echo "─────────────────────────────────────────"
MAX_SPEED=$(ros2 param get /motor_driver_node max_speed 2>/dev/null | grep "Double value" | awk '{print $4}')
MAX_ANGULAR=$(ros2 param get /motor_driver_node max_angular_speed 2>/dev/null | grep "Double value" | awk '{print $4}')
LEFT_INV=$(ros2 param get /motor_driver_node left_inverted 2>/dev/null | grep "Boolean value" | awk '{print $4}')
RIGHT_INV=$(ros2 param get /motor_driver_node right_inverted 2>/dev/null | grep "Boolean value" | awk '{print $4}')

echo "  max_speed:          ${MAX_SPEED} m/s"
echo "  max_angular_speed:  ${MAX_ANGULAR} rad/s"
echo "  left_inverted:      ${LEFT_INV}"
echo "  right_inverted:     ${RIGHT_INV}"
echo

# Monitor cmd_vel commands
echo "Monitoring /cmd_vel for 5 seconds..."
echo "Press W in teleop to test forward motion"
echo "─────────────────────────────────────────"
echo

timeout 5 ros2 topic echo /cmd_vel 2>/dev/null | head -20

echo
echo "========================================="
echo "  COMMON FIXES"
echo "========================================="
echo
echo "Problem: Left motor spins backward when pressing W"
echo "Fix:"
echo "  ros2 param set /motor_driver_node left_inverted true"
echo
echo "Problem: Right motor spins backward when pressing W"
echo "Fix:"
echo "  ros2 param set /motor_driver_node right_inverted true"
echo
echo "Problem: One motor doesn't spin at all"
echo "Fix: Check GPIO wiring and power. Enable debug logging:"
echo "  ros2 param set /motor_driver_node log_level DEBUG"
echo
echo "Problem: Motors are jerky or stuttering"
echo "Fix: Already applied PWM smoothing in latest build"
echo
echo "To revert inversions:"
echo "  ros2 param set /motor_driver_node left_inverted false"
echo "  ros2 param set /motor_driver_node right_inverted false"
echo
echo "========================================="
echo "  TEST COMMANDS"
echo "========================================="
echo
echo "Send a test forward command (both motors should go forward):"
echo "  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 1.0}, angular: {z: 0.0}}\""
echo
echo "Send a test rotation command (motors should spin opposite):"
echo "  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.5}}\""
echo
echo "Stop motors:"
echo "  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.0}}\""
echo
