#!/bin/bash
# Quick Start Guide for ESP32-Based Fire Fighter Rover

echo "🚒 Fire Fighter Rover - ESP32 System Quick Start"
echo "=================================================="
echo ""

# Step 1: Check ESP32 connection
echo "📌 Step 1: Checking ESP32 connection..."
if ls /dev/ttyUSB* 2>/dev/null || ls /dev/ttyACM* 2>/dev/null; then
    echo "✅ ESP32 device found"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
else
    echo "❌ No ESP32 found! Please connect ESP32 via USB"
    exit 1
fi

echo ""

# Step 2: Check permissions
echo "📌 Step 2: Checking permissions..."
if groups | grep -q dialout; then
    echo "✅ User is in dialout group"
else
    echo "⚠️  Adding user to dialout group..."
    sudo usermod -a -G dialout $USER
    echo "⚠️  Please log out and log back in for changes to take effect"
fi

echo ""

# Step 3: Build workspace
echo "📌 Step 3: Building ROS 2 workspace..."
cd /home/alibaba/frr_ws
colcon build --packages-select frr_control frr_sensors frr_navigation frr_bringup

echo ""

# Step 4: Source workspace
echo "📌 Step 4: Sourcing workspace..."
source install/setup.bash

echo ""

# Step 5: Test ESP32 communication
echo "📌 Step 5: Testing ESP32 communication..."
echo "Would you like to test ESP32 communication? (y/n)"
read -r response

if [[ "$response" == "y" ]]; then
    python3 test_esp32.py
fi

echo ""
echo "✅ Setup complete!"
echo ""
echo "🚀 To launch the rover system:"
echo "   ros2 launch frr_bringup esp32_rover_bringup.launch.py"
echo ""
echo "🎮 To control the rover (in new terminal):"
echo "   source install/setup.bash"
echo "   ros2 run frr_navigation esp32_teleop_node"
echo ""
echo "📹 Camera stream available at:"
echo "   http://192.168.1.6:8080/stream.mjpg"
echo ""