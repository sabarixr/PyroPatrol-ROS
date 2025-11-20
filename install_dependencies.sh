#!/bin/bash

# Fire Fighter Rover - Dependency Installation Script
# Run this script on Raspberry Pi OS to install all required dependencies

echo "🚒 Fire Fighter Rover - Installing Dependencies"
echo "=============================================="

# Update package list
echo "📦 Updating package list..."
sudo apt update

# Install system packages
echo "📦 Installing system packages..."
sudo apt install -y \
    python3-opencv \
    python3-smbus \
    python3-rpi.gpio \
    i2c-tools \
    ros-humble-rclpy \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs

# Install Python packages
echo "🐍 Installing Python packages..."
pip3 install --user flask numpy

# Enable I2C interface
echo "🔧 Configuring I2C interface..."
if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
    echo "⚠️  I2C enabled in config.txt - reboot required!"
    REBOOT_REQUIRED=true
fi

# Add user to gpio group
echo "👤 Adding user to gpio group..."
sudo usermod -a -G gpio $USER

# Test I2C (if available)
echo "🔍 Testing I2C interface..."
if [ -e /dev/i2c-1 ]; then
    echo "✅ I2C interface available at /dev/i2c-1"
    echo "🔍 Scanning for I2C devices..."
    i2cdetect -y 1
else
    echo "⚠️  I2C interface not found. Enable it with: sudo raspi-config"
fi

echo ""
echo "✅ Installation complete!"
echo ""
echo "📋 Next steps:"
echo "1. Build the workspace: cd /home/alibaba/frr_ws && colcon build --symlink-install"
echo "2. Source the workspace: source install/setup.bash"

if [ "$REBOOT_REQUIRED" = true ]; then
    echo "3. 🔄 REBOOT REQUIRED for I2C changes to take effect"
fi

echo "4. Launch the rover: ros2 launch frr_bringup rover_bringup.launch.py"
echo ""
echo "🔧 Hardware checklist:"
echo "- Connect MMA8452 to I2C (SDA=GPIO2, SCL=GPIO3, VCC=3.3V, GND=GND, ADDR=GND → 0x1C)"
echo "- Connect L298N motor driver (see README.md for GPIO mapping)"
echo "- Connect CSI camera to camera port"
echo "- Connect motors to L298N outputs"
echo "- Connect 12V power supply to L298N VCC and motors"
echo ""
echo "🌐 Access video stream at: http://$(hostname -I | awk '{print $1}'):8080"
