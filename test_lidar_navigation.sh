#!/bin/bash

# Test script for LiDAR navigation system
# Tests each component individually and provides diagnostics

echo "=================================="
echo "LiDAR Navigation System Test"
echo "=================================="
echo ""

# Source the workspace
source /home/alibaba/frr_ws/install/setup.bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}1. Checking LiDAR connection...${NC}"
if [ -e /dev/ttyUSB0 ]; then
    echo -e "${GREEN}✓ LiDAR found at /dev/ttyUSB0${NC}"
    ls -l /dev/ttyUSB0
else
    echo -e "${RED}✗ LiDAR not found at /dev/ttyUSB0${NC}"
    echo "Available USB devices:"
    ls /dev/ttyUSB* 2>/dev/null || echo "No USB devices found"
fi
echo ""

echo -e "${YELLOW}2. Checking I2C for IMU...${NC}"
if command -v i2cdetect &> /dev/null; then
    echo "MPU6050 should appear at address 0x68:"
    sudo i2cdetect -y 1 | grep -E "68|UU"
else
    echo "i2c-tools not installed. Install with: sudo apt-get install i2c-tools"
fi
echo ""

echo -e "${YELLOW}3. Listing available ROS2 nodes...${NC}"
echo "After launching, you should see:"
echo "  - /mpu6050_node"
echo "  - /lidar_node"
echo "  - /lidar_odometry_node"
echo "  - /obstacle_avoidance_node"
echo ""

echo -e "${YELLOW}4. Key topics to monitor:${NC}"
echo "  /scan                    - LiDAR scan data"
echo "  /imu/mpu6050            - IMU data"
echo "  /lidar_odom             - LiDAR-based odometry"
echo "  /cmd_vel                - Safe motor commands"
echo "  /obstacle_detected      - Obstacle status"
echo ""

echo -e "${YELLOW}5. Quick diagnostic commands:${NC}"
echo ""
echo "# Check LiDAR data:"
echo "ros2 topic echo /scan --once"
echo ""
echo "# Check IMU data:"
echo "ros2 topic echo /imu/mpu6050 --once"
echo ""
echo "# Check odometry:"
echo "ros2 topic echo /lidar_odom"
echo ""
echo "# Check obstacle detection:"
echo "ros2 topic echo /obstacle_detected"
echo ""
echo "# Monitor all topics:"
echo "ros2 topic list"
echo ""

echo -e "${YELLOW}6. Launch commands:${NC}"
echo ""
echo "# Launch complete system:"
echo "ros2 launch frr_bringup lidar_navigation.launch.py"
echo ""
echo "# Or launch nodes individually:"
echo "ros2 run frr_sensors mpu6050_node"
echo "ros2 run frr_sensors lidar_node"
echo "ros2 run frr_sensors lidar_odometry_node"
echo "ros2 run frr_sensors obstacle_avoidance_node"
echo ""

echo -e "${YELLOW}7. Test standalone LiDAR:${NC}"
echo "cd /home/alibaba/frr_ws && python3 lidar.py"
echo ""

echo -e "${GREEN}=================================="
echo "Test script complete!"
echo "==================================${NC}"
echo ""
echo "To start the system:"
echo "  ros2 launch frr_bringup lidar_navigation.launch.py"
echo ""
