# 🔧 Hardware Configuration Updates

## ✅ **Changes Applied**

### **1. L298N Motor Driver Pin Mapping** *(Updated)*
```
L298N Pin    → GPIO Pin    Function
ENA         → GPIO 17     PWM for left motor speed
IN1         → GPIO 27     Motor A direction 1  
IN2         → GPIO 22     Motor A direction 2
IN3         → GPIO 23     Motor B direction 1
IN4         → GPIO 24     Motor B direction 2  
ENB         → GPIO 18     PWM for right motor speed
```

### **2. MMA8452 I²C Address** *(Corrected)*
- **Old**: 0x1D 
- **New**: 0x1C ✅ *(matches your ESP32 scan results)*

### **3. Camera Settings** *(Improved for FPV)*
- **Resolution**: 640×480 *(up from 320×240)*
- **Frame Rate**: 30 FPS *(up from 10 FPS)*
- **Result**: Much better FPV video quality! 🎥

### **4. ArUco Pose Explanation** *(Added)*
Added detailed explanation of what `/aruco/pose` returns:
- **Position**: X, Y, Z coordinates (meters from camera)
- **Orientation**: Quaternion rotation (x, y, z, w)
- **Use Cases**: Navigation waypoints, localization, target tracking

## 📁 **Files Updated**

### Motor Driver Configuration:
- `frr_control/motor_controller_node.py` - GPIO pin assignments
- `README.md` - Hardware connection tables (2 places)

### IMU Configuration:  
- `frr_sensors/imu_node.py` - I²C address 0x1C
- `README.md` - I²C address documentation
- `install_dependencies.sh` - Hardware setup instructions
- `test_system.sh` - Expected I²C address in comments

### Camera Configuration:
- `frr_sensors/camera_node.py` - Already had 640×480@30FPS
- `launch/rover_bringup.launch.py` - Already had higher resolution
- `README.md` - Updated camera specs and ArUco explanation

## 🚀 **Ready to Use**

The system is now configured for your exact hardware setup:

```bash
# Build and launch (already done)
cd /home/alibaba/frr_ws
colcon build --symlink-install
source install/setup.bash

# Start the rover system
ros2 launch frr_bringup rover_bringup.launch.py

# Manual control (separate terminal)
ros2 run frr_control teleop_node

# View high-quality FPV video
# Browser: http://raspberry-pi-ip:8080
```

## 🔍 **Hardware Verification**

Your I²C scan should now show:
```
╔════════════════════════════╗
║      I2C Device Scan       ║  
╚════════════════════════════╝
  ✓ 0x1C ← MMA8452 ✅
  Total: 1 device(s) found
```

The motor pins now match your updated wiring, and you'll get smooth 30 FPS video for excellent FPV experience! 🎯
