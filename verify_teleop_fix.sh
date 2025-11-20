#!/bin/bash
# Quick test script for teleop and LiDAR auto-detection

source ~/frr_ws/install/setup.bash

echo "========================================="
echo "  TELEOP & LIDAR FIX - VERIFICATION"
echo "========================================="
echo

# Check if nodes are running
echo "1. Checking running nodes..."
if ros2 node list 2>/dev/null | grep -q "motor_driver_node"; then
    echo "   ✓ motor_driver_node is running"
else
    echo "   ✗ motor_driver_node NOT running"
    echo "     Start it with: ./start_robot.sh"
fi

if ros2 node list 2>/dev/null | grep -q "obstacle_avoidance_node"; then
    echo "   ✓ obstacle_avoidance_node is running"
else
    echo "   ⚠ obstacle_avoidance_node not running (normal if not started yet)"
fi

if ros2 node list 2>/dev/null | grep -q "teleop_node"; then
    echo "   ✓ teleop_node is running"
else
    echo "   ⚠ teleop_node not running (normal if not started yet)"
fi

echo

# Check LiDAR
echo "2. Checking LiDAR status..."
if ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
    echo "   ✓ /scan topic exists"
    
    # Try to get scan frequency (timeout after 3 seconds)
    SCAN_HZ=$(timeout 3 ros2 topic hz /scan 2>/dev/null | grep "average rate" | awk '{print $3}')
    if [ ! -z "$SCAN_HZ" ]; then
        echo "   ✓ LiDAR publishing at ${SCAN_HZ} Hz"
        echo "   → Obstacle avoidance will be ACTIVE"
    else
        echo "   ⚠ /scan topic exists but no data"
        echo "   → Obstacle avoidance will switch to PASSTHROUGH mode"
    fi
else
    echo "   ✗ /scan topic not found"
    echo "   → Obstacle avoidance will run in PASSTHROUGH mode"
fi

echo

# Check motor driver parameters
echo "3. Checking motor driver parameters..."
if ros2 node list 2>/dev/null | grep -q "motor_driver_node"; then
    MAX_SPEED=$(ros2 param get /motor_driver_node max_speed 2>/dev/null | grep "Double value" | awk '{print $4}')
    MAX_ANGULAR=$(ros2 param get /motor_driver_node max_angular_speed 2>/dev/null | grep "Double value" | awk '{print $4}')
    
    if [ ! -z "$MAX_SPEED" ]; then
        echo "   ✓ max_speed: ${MAX_SPEED} m/s"
        if (( $(echo "$MAX_SPEED >= 2.5" | bc -l) )); then
            echo "     → Dynamic speed increase will work properly"
        else
            echo "     ⚠ Speed limited to ${MAX_SPEED} m/s (recommend 2.5)"
        fi
    fi
    
    if [ ! -z "$MAX_ANGULAR" ]; then
        echo "   ✓ max_angular_speed: ${MAX_ANGULAR} rad/s"
    fi
fi

echo

# Check topics
echo "4. Checking teleop topics..."
if ros2 topic list 2>/dev/null | grep -q "/cmd_vel_teleop"; then
    echo "   ✓ /cmd_vel_teleop exists (teleop → obstacle avoidance)"
else
    echo "   ⚠ /cmd_vel_teleop not found (teleop not running?)"
fi

if ros2 topic list 2>/dev/null | grep -q "^/cmd_vel$"; then
    echo "   ✓ /cmd_vel exists (obstacle avoidance → motor driver)"
else
    echo "   ⚠ /cmd_vel not found"
fi

echo
echo "========================================="
echo "  QUICK START COMMANDS"
echo "========================================="
echo
echo "If robot NOT running:"
echo "  cd ~/frr_ws && ./start_robot.sh"
echo
echo "To start teleop:"
echo "  cd ~/frr_ws && ./start_teleop.sh"
echo
echo "Direct mode (no obstacle avoidance):"
echo "  ros2 run frr_control teleop_node_clean --ros-args -p with_avoidance:=false"
echo
echo "Check this documentation:"
echo "  cat ~/frr_ws/TELEOP_LIDAR_FIX.md"
echo
