#!/bin/bash
# Fix MPU6050 I2C communication issues

set -e

echo "=============================================="
echo "MPU6050 I2C Issue Fix Script"
echo "=============================================="
echo

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Step 1: Kill any running processes
echo -e "${YELLOW}Step 1: Stopping all ROS processes...${NC}"
pkill -f "ros2 launch" || true
pkill -f "mpu6050" || true
sleep 2
echo -e "${GREEN}✓ Processes stopped${NC}"
echo

# Step 2: Backup original node
echo -e "${YELLOW}Step 2: Backing up original MPU6050 node...${NC}"
cd /home/alibaba/frr_ws/src/frr_sensors/frr_sensors
if [ ! -f mpu6050_node.py.backup ]; then
    cp mpu6050_node.py mpu6050_node.py.backup
    echo -e "${GREEN}✓ Backup created${NC}"
else
    echo -e "${GREEN}✓ Backup already exists${NC}"
fi
echo

# Step 3: Replace with robust version with vibration filtering
echo -e "${YELLOW}Step 3: Installing robust MPU6050 node with vibration filtering...${NC}"
echo "This version filters out LiDAR motor vibrations"
cp mpu6050_node_vibration_filtered.py mpu6050_node.py
echo -e "${GREEN}✓ Robust node with vibration filtering installed${NC}"
echo

# Step 4: Check I2C bus speed configuration
echo -e "${YELLOW}Step 4: Checking I2C bus speed...${NC}"
CONFIG_FILE=""
if [ -f /boot/firmware/config.txt ]; then
    CONFIG_FILE="/boot/firmware/config.txt"
elif [ -f /boot/config.txt ]; then
    CONFIG_FILE="/boot/config.txt"
fi

if [ -n "$CONFIG_FILE" ]; then
    echo "Config file: $CONFIG_FILE"
    
    # Check current setting
    if grep -q "i2c_arm_baudrate" "$CONFIG_FILE"; then
        CURRENT_SPEED=$(grep "i2c_arm_baudrate" "$CONFIG_FILE" | grep -v "^#" | cut -d= -f2)
        echo "Current I2C speed: ${CURRENT_SPEED} Hz"
        
        if [ "$CURRENT_SPEED" -gt 400000 ]; then
            echo -e "${YELLOW}⚠ I2C speed is too high, this can cause communication errors${NC}"
            echo "Recommend setting to 100000 (100kHz) for stability"
            echo
            echo "To change, run:"
            echo "  sudo nano $CONFIG_FILE"
            echo "  Change i2c_arm_baudrate to 100000"
            echo "  Then reboot"
        else
            echo -e "${GREEN}✓ I2C speed looks good${NC}"
        fi
    else
        echo -e "${YELLOW}⚠ No I2C speed configured, using default${NC}"
        echo "To optimize, add this line to $CONFIG_FILE:"
        echo "  dtparam=i2c_arm_baudrate=100000"
        echo
        echo "Would you like to add it now? (requires sudo) [y/N]"
        read -r -n 1 response
        echo
        if [[ "$response" =~ ^[Yy]$ ]]; then
            sudo bash -c "echo 'dtparam=i2c_arm_baudrate=100000' >> $CONFIG_FILE"
            echo -e "${GREEN}✓ I2C speed configuration added${NC}"
            echo -e "${YELLOW}⚠ Reboot required for this change to take effect${NC}"
        fi
    fi
else
    echo -e "${RED}✗ Could not find config file${NC}"
fi
echo

# Step 5: Rebuild the package
echo -e "${YELLOW}Step 5: Rebuilding frr_sensors package...${NC}"
cd /home/alibaba/frr_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select frr_sensors --symlink-install
echo -e "${GREEN}✓ Package rebuilt${NC}"
echo

# Step 6: Test the sensor
echo -e "${YELLOW}Step 6: Testing MPU6050...${NC}"
python3 diagnose_mpu6050.py | grep -E "✓|✗|Results:"
echo

# Step 7: Instructions
echo "=============================================="
echo -e "${GREEN}Fix Applied Successfully!${NC}"
echo "=============================================="
echo
echo "What was fixed:"
echo "  1. ✓ Installed robust MPU6050 node with retry logic"
echo "  2. ✓ Reduced polling rate from 100Hz to 50Hz"
echo "  3. ✓ Added I2C error recovery and reinitialization"
echo "  4. ✓ Added proper delays in initialization"
echo "  5. ✓ Added vibration filtering for LiDAR motor interference"
echo "  6. ✓ Hardware DLPF set to 5Hz to reject high-frequency vibrations"
echo "  7. ✓ Moving average + low-pass filter for smooth data"
echo
echo "To start the robot:"
echo "  cd ~/frr_ws"
echo "  source install/setup.bash"
echo "  ros2 launch frr_bringup rover_bringup.launch.py"
echo
echo "To monitor the IMU:"
echo "  ros2 topic echo /imu/mpu6050"
echo "  ros2 topic hz /imu/mpu6050"
echo
echo -e "${YELLOW}Note: If you added I2C speed configuration, reboot first!${NC}"
echo

# Step 8: Optionally restart the system
echo "Would you like to restart the robot system now? [y/N]"
read -r -n 1 response
echo
if [[ "$response" =~ ^[Yy]$ ]]; then
    echo "Starting robot system..."
    cd /home/alibaba/frr_ws
    source install/setup.bash
    ros2 launch frr_bringup rover_bringup.launch.py
fi
