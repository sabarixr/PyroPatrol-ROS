#!/bin/bash

# ROS2 Fire Fighter Rover Test Script
# This script helps test all the updated components

echo "=========================================="
echo "Fire Fighter Rover - System Test"
echo "=========================================="
echo ""

# Source the workspace
cd /home/alibaba/frr_ws
source install/setup.bash

echo "Available test options:"
echo "1. Test Motor Controller (with teleop)"
echo "2. Test Camera with reduced resolution"
echo "3. Test MPU6050 IMU sensor"
echo "4. Test Odometry Fusion"
echo "5. Launch Full System"
echo ""

read -p "Enter your choice (1-5): " choice

case $choice in
    1)
        echo ""
        echo "Launching Motor Controller and Teleop..."
        echo "Use keys: w/x (forward/back), a/d (turn left/right)"
        echo "          i/k (camera up/down), s (stop)"
        echo "          t/g (increase/decrease linear speed)"
        echo "          r/f (increase/decrease angular speed)"
        echo ""
        echo "Press Enter to start..."
        read
        
        # Launch motor controller in background
        ros2 run frr_control motor_driver_node &
        MOTOR_PID=$!
        sleep 2
        
        # Launch teleop in foreground
        ros2 run frr_control teleop_node
        
        # Cleanup
        kill $MOTOR_PID 2>/dev/null
        ;;
        
    2)
        echo ""
        echo "Launching Camera Node with reduced resolution (320x240)..."
        echo "View stream with: ros2 run rqt_image_view rqt_image_view"
        echo "Or check topics with: ros2 topic list"
        echo ""
        read -p "Press Enter to start..."
        
        ros2 run frr_sensors camera_node --ros-args \
            -p frame_width:=320 \
            -p frame_height:=240 \
            -p fps:=30
        ;;
        
    3)
        echo ""
        echo "Launching MPU6050 IMU Node..."
        echo "This will read data from the MPU6050 at the center of the rover"
        echo "Check data with: ros2 topic echo /imu/mpu6050"
        echo ""
        read -p "Press Enter to start..."
        
        ros2 run frr_sensors mpu6050_node
        ;;
        
    4)
        echo ""
        echo "Launching Odometry Fusion Node..."
        echo "This fuses MMA8452 (front) and MPU6050 (center) data"
        echo "Check odometry with: ros2 topic echo /odom"
        echo ""
        echo "Starting required sensors first..."
        
        # Launch MMA8452 sensor
        ros2 run frr_sensors imu_node &
        IMU_PID=$!
        sleep 2
        
        # Launch MPU6050 sensor
        ros2 run frr_sensors mpu6050_node &
        MPU_PID=$!
        sleep 2
        
        # Launch odometry fusion
        ros2 run frr_sensors odometry_node
        
        # Cleanup
        kill $IMU_PID $MPU_PID 2>/dev/null
        ;;
        
    5)
        echo ""
        echo "Launching Full System..."
        echo "This will start all nodes in separate terminals"
        echo ""
        
        # Check if tmux is available
        if command -v tmux &> /dev/null; then
            echo "Using tmux for multiple terminals..."
            
            # Create a new tmux session
            tmux new-session -d -s frr_test
            
            # Window 0: Motor controller
            tmux rename-window -t frr_test:0 'motor'
            tmux send-keys -t frr_test:0 'cd /home/alibaba/frr_ws && source install/setup.bash && ros2 run frr_control motor_driver_node' C-m
            
            # Window 1: Teleop
            tmux new-window -t frr_test:1 -n 'teleop'
            tmux send-keys -t frr_test:1 'cd /home/alibaba/frr_ws && source install/setup.bash && sleep 2 && ros2 run frr_control teleop_node' C-m
            
            # Window 2: MMA8452 IMU
            tmux new-window -t frr_test:2 -n 'imu_front'
            tmux send-keys -t frr_test:2 'cd /home/alibaba/frr_ws && source install/setup.bash && ros2 run frr_sensors imu_node' C-m
            
            # Window 3: MPU6050 IMU
            tmux new-window -t frr_test:3 -n 'imu_center'
            tmux send-keys -t frr_test:3 'cd /home/alibaba/frr_ws && source install/setup.bash && ros2 run frr_sensors mpu6050_node' C-m
            
            # Window 4: Odometry
            tmux new-window -t frr_test:4 -n 'odometry'
            tmux send-keys -t frr_test:4 'cd /home/alibaba/frr_ws && source install/setup.bash && sleep 3 && ros2 run frr_sensors odometry_node' C-m
            
            # Window 5: Camera
            tmux new-window -t frr_test:5 -n 'camera'
            tmux send-keys -t frr_test:5 'cd /home/alibaba/frr_ws && source install/setup.bash && ros2 run frr_sensors camera_node --ros-args -p frame_width:=320 -p frame_height:=240' C-m
            
            # Attach to the session
            tmux attach-session -t frr_test
            
        else
            echo "tmux not found. Launching nodes sequentially..."
            echo "Install tmux for better testing: sudo apt-get install tmux"
            echo ""
            echo "Launching all nodes in background..."
            
            ros2 run frr_sensors imu_node &
            sleep 1
            ros2 run frr_sensors mpu6050_node &
            sleep 1
            ros2 run frr_sensors odometry_node &
            sleep 1
            ros2 run frr_sensors camera_node --ros-args -p frame_width:=320 -p frame_height:=240 &
            sleep 1
            ros2 run frr_control motor_driver_node &
            sleep 2
            
            echo ""
            echo "All background nodes started. Launching teleop..."
            ros2 run frr_control teleop_node
            
            # Cleanup on exit
            killall imu_node mpu6050_node odometry_node camera_node motor_driver_node 2>/dev/null
        fi
        ;;
        
    *)
        echo "Invalid choice. Exiting..."
        exit 1
        ;;
esac

echo ""
echo "Test completed!"
