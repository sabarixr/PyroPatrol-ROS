#!/bin/bash
# Quick Test Script for ESP32-ROS Integration

echo "🚒 Testing ESP32-ROS Integration"
echo "================================"

# Source workspace
cd /home/alibaba/frr_ws
source install/setup.bash

echo ""
echo "1️⃣  Testing ESP32 Bridge Node..."
echo "Starting bridge in background..."

# Start ESP32 bridge in background
ros2 run frr_control esp32_bridge_node --ros-args -p serial_port:=/dev/ttyACM0 &
BRIDGE_PID=$!

sleep 3

echo ""
echo "2️⃣  Testing motor commands..."

# Test forward
echo "  ▶️  Forward..."
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
sleep 2

# Test turn right
echo "  ↪️  Turn right..."
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}"
sleep 2

# Test turn left
echo "  ↩️  Turn left..."
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
sleep 2

# Stop
echo "  ⏹️  Stop..."
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

echo ""
echo "3️⃣  Testing fire scanning..."
ros2 topic pub --once /fire_scan std_msgs/Bool "{data: true}"
sleep 5
ros2 topic pub --once /fire_scan std_msgs/Bool "{data: false}"

echo ""
echo "✅ Test complete!"
echo "Check if motors responded correctly."

# Cleanup
kill $BRIDGE_PID
