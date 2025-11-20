#!/bin/bash

# Parameter tuning helper for obstacle avoidance
# Allows real-time adjustment of safety parameters

source /home/alibaba/frr_ws/install/setup.bash

echo "=================================="
echo "Obstacle Avoidance Parameter Tuner"
echo "=================================="
echo ""

NODE="/obstacle_avoidance_node"

echo "Current parameters:"
echo ""

# Get current parameters
echo "1. rover_width (current):"
ros2 param get $NODE rover_width 2>/dev/null || echo "   Node not running"

echo "2. rover_length (current):"
ros2 param get $NODE rover_length 2>/dev/null || echo "   Node not running"

echo "3. safety_margin (current):"
ros2 param get $NODE safety_margin 2>/dev/null || echo "   Node not running"

echo "4. stop_distance (current):"
ros2 param get $NODE stop_distance 2>/dev/null || echo "   Node not running"

echo ""
echo "=================================="
echo "Adjustment Commands:"
echo "=================================="
echo ""

echo "# Increase safety margin (less sensitive, stops farther away):"
echo "ros2 param set $NODE safety_margin 0.20"
echo ""

echo "# Decrease safety margin (more aggressive, allows closer approach):"
echo "ros2 param set $NODE safety_margin 0.10"
echo ""

echo "# Increase stop distance (emergency stop earlier):"
echo "ros2 param set $NODE stop_distance 0.40"
echo ""

echo "# Decrease stop distance (emergency stop later):"
echo "ros2 param set $NODE stop_distance 0.20"
echo ""

echo "# Update rover dimensions if needed:"
echo "ros2 param set $NODE rover_width 0.25"
echo "ros2 param set $NODE rover_length 0.35"
echo ""

echo "=================================="
echo "Recommended Settings:"
echo "=================================="
echo ""

echo "Conservative (safe, large margins):"
echo "  safety_margin: 0.20 m"
echo "  stop_distance: 0.40 m"
echo ""

echo "Balanced (default):"
echo "  safety_margin: 0.15 m"
echo "  stop_distance: 0.30 m"
echo ""

echo "Aggressive (tight spaces, experienced):"
echo "  safety_margin: 0.10 m"
echo "  stop_distance: 0.20 m"
echo ""

echo "=================================="
echo ""

# Interactive mode
if [ "$1" == "interactive" ]; then
    echo "Interactive mode - Enter 'q' to quit"
    echo ""
    
    while true; do
        echo "Enter parameter to change (safety_margin/stop_distance) or 'q' to quit:"
        read param
        
        if [ "$param" == "q" ]; then
            echo "Exiting..."
            break
        fi
        
        echo "Enter new value (in meters):"
        read value
        
        ros2 param set $NODE $param $value
        
        echo ""
        echo "Updated! Current value:"
        ros2 param get $NODE $param
        echo ""
    done
fi

echo "Tip: Run with 'interactive' argument for guided tuning:"
echo "  ./tune_obstacle_avoidance.sh interactive"
echo ""
