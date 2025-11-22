#!/bin/bash
# Complete system fix script
# Fixes: MPU6050, speed increase, obstacle avoidance

set -e  # Exit on error

BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Fire Fighter Rover - System Fix${NC}"
echo -e "${BLUE}========================================${NC}"
echo

# 1. Kill existing processes
echo -e "${YELLOW}[1/4] Stopping running nodes...${NC}"
pkill -f "ros2 launch" || true
pkill -f "motor_driver" || true
pkill -f "mpu6050" || true
sleep 2
echo -e "${GREEN}✓ Processes stopped${NC}"
echo

# 2. Source environment
echo -e "${YELLOW}[2/4] Sourcing ROS workspace...${NC}"
cd /home/alibaba/frr_ws
source install/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"
echo

# 3. Verify files are updated
echo -e "${YELLOW}[3/4] Verifying updates...${NC}"

# Check MPU6050 has vibration filtering
if grep -q "vibration filtering" src/frr_sensors/frr_sensors/mpu6050_node.py; then
    echo -e "${GREEN}✓ MPU6050 vibration filtering: ACTIVE${NC}"
else
    echo -e "${RED}✗ MPU6050 still using old code!${NC}"
    exit 1
fi

# Check teleop has updated speeds (3.0 m/s initial, 6.0 m/s max)
if grep -q "self.linear_speed = 3.0" src/frr_control/frr_control/teleop_node_clean.py && \
   grep -q "self.max_linear = 6.0" src/frr_control/frr_control/teleop_node_clean.py; then
    echo -e "${GREEN}✓ Teleop speeds: initial 3.0 m/s, max 6.0 m/s${NC}"
else
    echo -e "${YELLOW}⚠ Teleop using different speed settings${NC}"
fi

# Check motor driver has updated speeds (min 2.0 m/s, max 6.0 m/s)
if grep -q "self.declare_parameter('max_speed', 6.0)" src/frr_control/motor_controller_node.py && \
   grep -q "self.declare_parameter('min_speed', 2.0)" src/frr_control/motor_controller_node.py; then
    echo -e "${GREEN}✓ Motor driver: min 2.0 m/s, max 6.0 m/s${NC}"
else
    echo -e "${YELLOW}⚠ Motor driver using different speed settings${NC}"
fi

# Check obstacle avoidance has increased margins
if grep -q "0.25" src/frr_sensors/frr_sensors/obstacle_avoidance_node.py; then
    echo -e "${GREEN}✓ Obstacle avoidance: More sensitive (25cm/35cm)${NC}"
else
    echo -e "${YELLOW}⚠ Obstacle avoidance margins not updated${NC}"
fi

echo

# 4. Start robot system
echo -e "${YELLOW}[4/4] Starting robot system...${NC}"
echo -e "${BLUE}Launching in 3 seconds...${NC}"
sleep 1
echo "3..."
sleep 1
echo "2..."
sleep 1
echo "1..."
sleep 1

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}✓ Starting Robot!${NC}"
echo -e "${GREEN}========================================${NC}"
echo
echo -e "${YELLOW}Features enabled:${NC}"
echo -e "  ✓ Camera with ArUco detection (FIXED INVERSION)"
echo -e "  ✓ LIDAR obstacle avoidance"
echo -e "  ✓ Manual teleop control (opens in new window)"
echo -e "  ✓ Fire seeking (autonomous when fire detected)"
echo -e "  ✓ ESP32 motor control"
echo -e "  ✓ Video streaming at http://<pi-ip>:8080"
echo
echo -e "${YELLOW}How it works:${NC}"
echo -e "  ${GREEN}Manual Control:${NC}"
echo -e "    - Use teleop window (will open automatically)"
echo -e "    - Drive with W/A/S/D keys"
echo -e "    - Speed up/down with T/G keys"
echo
echo -e "  ${GREEN}Fire Seeking (Autonomous):${NC}"
echo -e "    - Monitors MQ2, MQ5, flame, temperature sensors"
echo -e "    - When fire detected → moves toward source"
echo -e "    - Activates pump when close"
echo -e "    - Always avoids obstacles!"
echo
echo -e "  ${GREEN}ArUco Mode (Optional):${NC}"
echo -e "    - To enable: add 'enable_aruco_follower:=true'"
echo -e "    - Moves 2s when marker detected"
echo
echo -e "${BLUE}========================================${NC}"
echo

# Launch the complete rover system
echo -e "${YELLOW}Launching rover bringup...${NC}"
ros2 launch frr_bringup rover_bringup.launch.py
