#!/bin/bash
# Start Rover Script
# Quick launcher for the Fire Rover Robot System
# Updated for ESP32-S3 N16R8 with LIDAR support

echo "Starting Fire Rover Robot System..."

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/frr_ws/install/setup.bash

# Detect ESP32-S3 port (prioritize ttyACM0 for ESP32)
ESP_PORT=""
if [ -e "/dev/ttyACM0" ]; then
    ESP_PORT="/dev/ttyACM0"
    echo "[OK] Found ESP32-S3 on /dev/ttyACM0"
elif [ -e "/dev/ttyACM1" ]; then
    ESP_PORT="/dev/ttyACM1"
    echo "[OK] Found ESP32-S3 on /dev/ttyACM1"
else
    echo "[!] ESP32-S3 not found! Please check connection."
    echo "  Expected: /dev/ttyACM0 or /dev/ttyACM1"
    exit 1
fi

# Check ESP32 permissions
if [ ! -r "$ESP_PORT" ] || [ ! -w "$ESP_PORT" ]; then
    echo "[!] No permission for $ESP_PORT"
    echo "  Run: sudo chmod 666 $ESP_PORT"
    exit 1
fi

# Detect LIDAR port (usually ttyUSB0)
LIDAR_PORT=""
if [ -e "/dev/ttyUSB0" ]; then
    LIDAR_PORT="/dev/ttyUSB0"
    echo "[OK] Found LIDAR on /dev/ttyUSB0"
elif [ -e "/dev/ttyUSB1" ]; then
    LIDAR_PORT="/dev/ttyUSB1"
    echo "[OK] Found LIDAR on /dev/ttyUSB1"
elif [ -e "/dev/ydlidar" ]; then
    LIDAR_PORT="/dev/ydlidar"
    echo "[OK] Found LIDAR on /dev/ydlidar"
fi

echo "WebSocket Server: ws://192.168.1.13:8765"
echo "Video Stream: http://192.168.1.13:8080"
echo "ESP32-S3 Port: $ESP_PORT"
if [ -n "$LIDAR_PORT" ]; then
    echo "LIDAR Port: $LIDAR_PORT"
fi
echo ""

# Parse command line arguments
# LIDAR is now ENABLED BY DEFAULT for autonomous fire-seeking mode
ENABLE_LIDAR="true"
ENABLE_VIDEO="true"

while [[ $# -gt 0 ]]; do
    case $1 in
        --no-lidar)
            ENABLE_LIDAR="false"
            echo "[!] LIDAR disabled"
            shift
            ;;
        --no-video)
            ENABLE_VIDEO="false"
            echo "[!] Video disabled"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--no-lidar] [--no-video]"
            exit 1
            ;;
    esac
done

# Display LIDAR status
if [ "$ENABLE_LIDAR" = "true" ]; then
    echo "[OK] LIDAR enabled (required for autonomous fire-seeking with directional scanning)"
fi

# Check if LIDAR is enabled but not detected
if [ "$ENABLE_LIDAR" = "true" ] && [ -z "$LIDAR_PORT" ]; then
    echo ""
    echo "[!] WARNING: LIDAR not detected!"
    echo "   Autonomous fire-seeking mode requires LIDAR for:"
    echo "   1. T-junction detection"
    echo "   2. Obstacle avoidance"
    echo "   3. Directional scanning decisions"
    echo ""
    echo "   Please check LIDAR connection on /dev/ttyUSB0"
    echo "   Or start without LIDAR: ./start_rover.sh --no-lidar"
    echo ""
    read -p "Continue without LIDAR? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
    ENABLE_LIDAR="false"
fi

# Grant LIDAR permissions if enabled
if [ "$ENABLE_LIDAR" = "true" ] && [ -n "$LIDAR_PORT" ]; then
    if [ ! -r "$LIDAR_PORT" ] || [ ! -w "$LIDAR_PORT" ]; then
        echo "Setting LIDAR permissions..."
        sudo chmod 666 $LIDAR_PORT
        if [ $? -ne 0 ]; then
            echo "[!] Failed to set permissions for $LIDAR_PORT"
            echo "  Please run: sudo chmod 666 $LIDAR_PORT"
            exit 1
        fi
        echo "[OK] LIDAR permissions set"
    fi
    
    # Power cycle recommendation
    echo ""
    echo "[TIP] Note: If LIDAR fails to start, try:"
    echo "   1. Unplug LIDAR USB cable"
    echo "   2. Wait 5 seconds"
    echo "   3. Plug it back in"
    echo "   4. Run this script again"
    echo ""
    sleep 2
fi

echo "Launching ROS2 nodes..."
echo ""

# Display autonomous fire-seeking mode info
if [ "$ENABLE_LIDAR" = "true" ]; then
        echo "AUTONOMOUS FIRE-SEEKING MODE READY"
        echo ""
    echo "How it works:"
    echo "  1. Robot moves forward autonomously"
    echo "  2. LIDAR detects T-junctions (obstacle ahead, paths left/right)"
    echo "  3. ESP32 performs directional SCAN with fire sensors"
    echo "  4. Robot turns toward detected fire direction"
    echo "  5. Repeats until fire is found and extinguished"
    echo ""
    echo "To activate: Send 'start_fire_mode' command from app"
        echo ""
fi

# Launch the rover
LAUNCH_ARGS="serial_port:=$ESP_PORT enable_video_stream:=$ENABLE_VIDEO enable_lidar:=$ENABLE_LIDAR ws_port:=8765"

# Only add lidar_port if LIDAR is enabled and port is detected
if [ "$ENABLE_LIDAR" = "true" ] && [ -n "$LIDAR_PORT" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS lidar_port:=$LIDAR_PORT"
fi

ros2 launch frr_bringup esp32_rover_bringup.launch.py $LAUNCH_ARGS
