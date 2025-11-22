#!/bin/bash

# Fire-Fighting Robot SLAM Launcher
# This script helps you quickly launch the robot in different modes

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if workspace is sourced
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo -e "${YELLOW}Sourcing workspace...${NC}"
    source /home/alibaba/frr_ws/install/setup.bash
fi

echo -e "${BLUE}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║        Fire-Fighting Robot SLAM System Launcher            ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

echo "Select mode:"
echo -e "${GREEN}1)${NC} MAPPING MODE - Build a new map while exploring"
echo -e "${GREEN}2)${NC} LOCALIZATION MODE - Use existing map for navigation"
echo -e "${GREEN}3)${NC} Save current map"
echo -e "${GREEN}4)${NC} View ROS topics"
echo -e "${GREEN}5)${NC} Check sensor status"
echo -e "${RED}6)${NC} Exit"
echo ""
read -p "Enter choice [1-6]: " choice

case $choice in
    1)
        echo -e "${BLUE}Launching MAPPING MODE...${NC}"
        echo -e "${YELLOW}The robot will build a map as it explores.${NC}"
        echo -e "${YELLOW}Press CTRL+C when done, then use option 3 to save the map.${NC}"
        echo ""
        sleep 2
        ros2 launch frr_bringup slam_mapping.launch.py
        ;;
    2)
        echo -e "${BLUE}Launching LOCALIZATION MODE...${NC}"
        
        # Check if maps directory exists and has maps
        if [ ! -d "/home/alibaba/frr_ws/maps" ]; then
            echo -e "${RED}Error: Maps directory not found!${NC}"
            echo -e "${YELLOW}Run MAPPING MODE first to create a map.${NC}"
            exit 1
        fi
        
        # List available maps
        maps=($(ls /home/alibaba/frr_ws/maps/*.posegraph 2>/dev/null))
        
        if [ ${#maps[@]} -eq 0 ]; then
            echo -e "${RED}Error: No maps found in /home/alibaba/frr_ws/maps/${NC}"
            echo -e "${YELLOW}Run MAPPING MODE first and save a map.${NC}"
            exit 1
        fi
        
        echo "Available maps:"
        for i in "${!maps[@]}"; do
            echo -e "${GREEN}$((i+1)))${NC} ${maps[$i]}"
        done
        echo ""
        read -p "Select map number: " map_choice
        
        if [ "$map_choice" -ge 1 ] && [ "$map_choice" -le "${#maps[@]}" ]; then
            selected_map="${maps[$((map_choice-1))]}"
            # Remove .posegraph extension for the parameter
            map_file="${selected_map%.posegraph}"
            echo -e "${BLUE}Using map: ${selected_map}${NC}"
            sleep 2
            ros2 launch frr_bringup slam_localization.launch.py map_file:="${map_file}"
        else
            echo -e "${RED}Invalid selection!${NC}"
            exit 1
        fi
        ;;
    3)
        echo -e "${BLUE}Saving current map...${NC}"
        read -p "Enter map name (no extension): " map_name
        
        if [ -z "$map_name" ]; then
            echo -e "${RED}Error: Map name cannot be empty!${NC}"
            exit 1
        fi
        
        echo -e "${YELLOW}Saving map to /home/alibaba/frr_ws/maps/${map_name}${NC}"
        ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/alibaba/frr_ws/maps/${map_name}'}}"
        
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}Map saved successfully!${NC}"
            echo -e "Files created:"
            ls -lh /home/alibaba/frr_ws/maps/${map_name}*
        else
            echo -e "${RED}Error saving map!${NC}"
            echo -e "${YELLOW}Make sure SLAM Toolbox is running (option 1).${NC}"
        fi
        ;;
    4)
        echo -e "${BLUE}ROS Topics:${NC}"
        echo ""
        ros2 topic list
        echo ""
        echo -e "${GREEN}To view a topic:${NC} ros2 topic echo /topic_name"
        echo -e "${GREEN}To see topic info:${NC} ros2 topic info /topic_name"
        ;;
    5)
        echo -e "${BLUE}Checking sensor status...${NC}"
        echo ""
        
        # Check LIDAR
        echo -e "${YELLOW}LIDAR (/scan):${NC}"
        timeout 2 ros2 topic echo /scan --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ LIDAR is publishing${NC}"
        else
            echo -e "${RED}✗ LIDAR not detected${NC}"
        fi
        
        # Check IMU
        echo -e "${YELLOW}IMU (/imu/mpu6050):${NC}"
        timeout 2 ros2 topic echo /imu/mpu6050 --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ IMU is publishing${NC}"
        else
            echo -e "${RED}✗ IMU not detected${NC}"
        fi
        
        # Check Odometry
        echo -e "${YELLOW}Odometry (/odom):${NC}"
        timeout 2 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ Odometry is publishing${NC}"
        else
            echo -e "${RED}✗ Odometry not detected${NC}"
        fi
        
        # Check ESP32
        echo -e "${YELLOW}ESP32 (/esp32_telemetry):${NC}"
        timeout 2 ros2 topic echo /esp32_telemetry --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ ESP32 is connected${NC}"
            echo ""
            echo "Latest telemetry:"
            timeout 1 ros2 topic echo /esp32_telemetry --once 2>/dev/null | head -20
        else
            echo -e "${RED}✗ ESP32 not connected${NC}"
            echo -e "${YELLOW}  Check /dev/ttyACM0 connection${NC}"
        fi
        
        # Check Camera
        echo -e "${YELLOW}Camera (/camera/image_raw):${NC}"
        timeout 2 ros2 topic echo /camera/image_raw --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ Camera is publishing${NC}"
        else
            echo -e "${RED}✗ Camera not detected${NC}"
        fi
        
        echo ""
        echo -e "${BLUE}Video stream:${NC} http://$(hostname -I | awk '{print $1}'):8080"
        ;;
    6)
        echo -e "${GREEN}Goodbye!${NC}"
        exit 0
        ;;
    *)
        echo -e "${RED}Invalid choice!${NC}"
        exit 1
        ;;
esac
