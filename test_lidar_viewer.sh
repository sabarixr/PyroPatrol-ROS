#!/bin/bash
# Quick test script for LiDAR viewer

echo "Stopping old viewer..."
pkill -f check_lidar.py

echo "Starting LiDAR viewer with fixes..."
cd /home/alibaba/frr_ws
source install/setup.bash
python3 check_lidar.py
