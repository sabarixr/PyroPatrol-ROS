#!/bin/bash

# Fire Fighter Rover - System Test Script

echo "🚒 Fire Fighter Rover - System Test"
echo "===================================="

# Check if workspace is sourced
echo "📋 Checking ROS 2 workspace..."
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS 2 not sourced. Run: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo "✅ ROS 2 $ROS_DISTRO detected"
fi

if [ -z "$COLCON_PREFIX_PATH" ] || [[ "$COLCON_PREFIX_PATH" != *"frr_ws"* ]]; then
    echo "⚠️  Workspace not sourced. Run: source install/setup.bash"
else
    echo "✅ Workspace sourced"
fi

# Check available packages
echo ""
echo "📦 Checking ROS 2 packages..."
PACKAGES=("frr_sensors" "frr_control" "frr_video" "frr_bringup")
for pkg in "${PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "^$pkg$"; then
        echo "✅ $pkg package found"
    else
        echo "❌ $pkg package not found"
    fi
done

# Check executables
echo ""
echo "🔧 Checking node executables..."
NODES=(
    "frr_sensors:imu_node"
    "frr_sensors:camera_node"
    "frr_control:motor_driver_node"
    "frr_control:teleop_node"
    "frr_video:streamer_node"
)

for node in "${NODES[@]}"; do
    pkg=$(echo $node | cut -d: -f1)
    exe=$(echo $node | cut -d: -f2)
    if ros2 pkg executables $pkg 2>/dev/null | grep -q "$exe"; then
        echo "✅ $pkg:$exe executable found"
    else
        echo "❌ $pkg:$exe executable not found"
    fi
done

# Check launch files
echo ""
echo "🚀 Checking launch files..."
LAUNCH_FILES=(
    "frr_bringup:rover_bringup.launch.py"
    "frr_bringup:teleop.launch.py"
)

for launch in "${LAUNCH_FILES[@]}"; do
    pkg=$(echo $launch | cut -d: -f1)
    file=$(echo $launch | cut -d: -f2)
    if find /home/alibaba/frr_ws/install -name "$file" | grep -q "$file"; then
        echo "✅ $pkg:$file launch file found"
    else
        echo "❌ $pkg:$file launch file not found"
    fi
done

# Hardware checks
echo ""
echo "🔌 Hardware availability checks..."

# Check I2C
if [ -e /dev/i2c-1 ]; then
    echo "✅ I2C interface available (/dev/i2c-1)"
    echo "🔍 I2C device scan:"
    i2cdetect -y 1 2>/dev/null | head -n 10
else
    echo "❌ I2C interface not available"
    echo "   Enable with: sudo raspi-config → Interface Options → I2C"
fi

# Check camera
echo ""
if [ -e /dev/video0 ]; then
    echo "✅ Camera device available (/dev/video0)"
else
    echo "⚠️  No camera device found at /dev/video0"
    echo "   Check camera connection and enable with: sudo raspi-config → Interface Options → Camera"
fi

# Check GPIO permissions
echo ""
if groups $USER | grep -q gpio; then
    echo "✅ User $USER in gpio group"
else
    echo "❌ User $USER not in gpio group"
    echo "   Add with: sudo usermod -a -G gpio $USER (then logout/login)"
fi

# Python dependencies check
echo ""
echo "🐍 Checking Python dependencies..."
PYTHON_DEPS=("cv2" "smbus" "flask" "numpy")
for dep in "${PYTHON_DEPS[@]}"; do
    if python3 -c "import $dep" 2>/dev/null; then
        echo "✅ Python module '$dep' available"
    else
        echo "❌ Python module '$dep' not available"
        case $dep in
            "cv2") echo "   Install with: sudo apt install python3-opencv" ;;
            "smbus") echo "   Install with: sudo apt install python3-smbus" ;;
            "flask") echo "   Install with: pip3 install flask" ;;
            "numpy") echo "   Install with: pip3 install numpy" ;;
        esac
    fi
done

# Check GPIO libraries
echo ""
echo "🔌 Checking GPIO libraries..."
if python3 -c "import pigpio" 2>/dev/null; then
    echo "✅ pigpio library available (recommended)"
    # Check pigpiod daemon
    if systemctl is-active --quiet pigpiod; then
        echo "✅ pigpiod daemon is running"
    else
        echo "⚠️  pigpiod daemon not running. Start with: sudo systemctl start pigpiod"
    fi
elif python3 -c "import RPi.GPIO" 2>/dev/null; then
    echo "✅ RPi.GPIO library available (fallback)"
    echo "💡 For better performance, install pigpio: sudo apt install pigpio python3-pigpio"
else
    echo "❌ No GPIO library found"
    echo "   Install with: sudo apt install python3-rpi.gpio"
fi

echo ""
echo "🏁 Test completed!"
echo ""
echo "💡 Quick start:"
echo "1. cd /home/alibaba/frr_ws"
echo "2. source install/setup.bash"
echo "3. ros2 launch frr_bringup rover_bringup.launch.py"
echo ""
echo "🎮 For manual control (separate terminal):"
echo "ros2 run frr_control teleop_node"
