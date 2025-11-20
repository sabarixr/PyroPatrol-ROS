#!/bin/bash
# Start teleop for Fire Fighter Rover
# This script works every time!

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE} Fire Fighter Rover - Teleop Starter${NC}"
echo -e "${BLUE}======================================${NC}"
echo

# Source system ROS and workspace overlay so ros2 and local packages are available.
# If system ROS is installed, source it first (so /opt/ros/* provides ros2).
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${YELLOW}Sourcing system ROS (humble)...${NC}"
    # shellcheck disable=SC1090
    source "/opt/ros/humble/setup.bash"
elif [ -n "$ROS_DISTRO" ]; then
    # If ROS_DISTRO is set but the standard setup isn't found, note it
    echo -e "${YELLOW}ROS_DISTRO is set to '$ROS_DISTRO' but /opt/ros/$ROS_DISTRO/setup.bash not found.${NC}"
fi

# Always source the workspace overlay if it exists so packages like frr_control become visible.
if [ -f "/home/alibaba/frr_ws/install/setup.bash" ]; then
    echo -e "${YELLOW}Sourcing workspace overlay...${NC}"
    # shellcheck disable=SC1090
    source "/home/alibaba/frr_ws/install/setup.bash"
else
    echo -e "${YELLOW}Workspace setup not found at /home/alibaba/frr_ws/install/setup.bash${NC}"
    echo "If you haven't built the workspace, run:"
    echo -e "  ${GREEN}colcon build --symlink-install${NC}"
fi

# Check if robot is running (with timeout to avoid hanging)
echo -e "${BLUE}Checking for robot nodes...${NC}"
if timeout 3 ros2 node list 2>/dev/null | grep -q "motor_driver_node"; then
    echo -e "${GREEN}✓ Robot system is running!${NC}"
else
    echo -e "${YELLOW}⚠️  Robot system might not be running${NC}"
    echo -e "${YELLOW}   (or ROS_DOMAIN_ID mismatch)${NC}"
    echo
    echo "If robot IS running in another terminal, press 'c' to continue"
    echo "Otherwise, press 'n' to exit and start it first"
    echo
    read -p "Continue anyway? [c/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Cc]$ ]]; then
        echo "Exiting. Start the robot first with:"
        echo -e "  ${GREEN}ros2 launch frr_bringup rover_bringup.launch.py${NC}"
        exit 1
    fi
fi

# Ask about obstacle avoidance
echo
echo "How do you want to control the robot?"
echo
echo -e "  ${GREEN}1${NC}) SAFE mode (with obstacle avoidance) ${GREEN}← RECOMMENDED${NC}"
echo -e "  ${YELLOW}2${NC}) DIRECT mode (no obstacle avoidance) ${YELLOW}← Use with caution${NC}"
echo
read -p "Choose [1/2]: " -n 1 -r mode
echo
echo

case $mode in
    2)
        echo -e "${YELLOW}⚠️  Starting in DIRECT mode${NC}"
        echo "Robot will NOT stop for obstacles!"
        sleep 1
        WITH_AVOIDANCE="false"
        ;;
    *)
        echo -e "${GREEN}✓ Starting in SAFE mode${NC}"
        echo "Obstacle avoidance is ACTIVE"
        sleep 1
        WITH_AVOIDANCE="true"
        ;;
esac

# Check if obstacle avoidance node should be running
if [ "$WITH_AVOIDANCE" = "true" ]; then
    if ! ros2 node list 2>&1 | grep -q "obstacle_avoidance_node"; then
        echo -e "${YELLOW}Starting obstacle avoidance node...${NC}"
        ros2 run frr_sensors obstacle_avoidance_node \
            --ros-args \
            -p rover_width:=0.20 \
            -p rover_length:=0.30 \
            -p safety_margin:=0.15 \
            -p stop_distance:=0.30 &
        OBSTACLE_PID=$!
        sleep 2
    fi
fi

echo
echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}Starting Teleop Node...${NC}"
echo -e "${BLUE}======================================${NC}"
sleep 1

# Start teleop in this terminal (so it can capture keyboard)
ros2 run frr_control teleop_node_clean \
    --ros-args \
    -p with_avoidance:=$WITH_AVOIDANCE

# Cleanup on exit
echo
echo -e "${GREEN}Teleop stopped.${NC}"
if [ ! -z "$OBSTACLE_PID" ]; then
    kill $OBSTACLE_PID 2>/dev/null
fi
