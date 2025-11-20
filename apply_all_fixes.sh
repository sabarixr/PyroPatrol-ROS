#!/bin/bash
# Apply all fixes at once

set -e

cd /home/alibaba/frr_ws

echo "================================================================"
echo "  APPLYING ALL FIXES"
echo "================================================================"
echo

# 1. Copy vibration-filtered MPU6050
echo "1. Installing vibration-filtered MPU6050..."
cd src/frr_sensors/frr_sensors
if [ -f mpu6050_node.py.backup ]; then
    echo "   Backup already exists"
else
    cp mpu6050_node.py mpu6050_node.py.backup
fi
cp mpu6050_node_vibration_filtered.py mpu6050_node.py
echo "   ✓ MPU6050 updated"

# 2. Rebuild packages
cd /home/alibaba/frr_ws
echo
echo "2. Rebuilding packages..."
source /opt/ros/humble/setup.bash
colcon build --packages-select frr_sensors frr_control --symlink-install 2>&1 | grep -E "Starting|Finished|Summary"
echo "   ✓ Packages rebuilt"

echo
echo "================================================================"
echo "  ALL FIXES APPLIED!"
echo "================================================================"
echo
echo "Changes made:"
echo "  ✓ MPU6050 now uses vibration filtering"
echo "  ✓ Max speed increased to 1.5 m/s"
echo "  ✓ Colorful terminal output"
echo "  ✓ Less log spam"
echo "  ✓ Continuous teleop publishing (no safety timeout)"
echo "  ✓ Obstacle avoidance logs are cleaner"
echo
echo "Restart your robot to apply changes:"
echo "  pkill -f ros2"
echo "  ros2 launch frr_bringup rover_bringup.launch.py"
echo
