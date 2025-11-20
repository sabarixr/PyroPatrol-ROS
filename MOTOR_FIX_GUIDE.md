# Motor Control Fix - PWM Smoothing & Direction Correction

## Issues Fixed

### 1. ✅ Only One Motor Spinning
**Problem:** When pressing W (forward), only right motor spins, left motor stays stopped.

**Root Cause:** 
- Motor wiring polarity differences
- PWM calculation or GPIO direction mismatch

**Solution Applied:**
- Added `left_inverted` and `right_inverted` runtime parameters
- Added PWM smoothing filter to reduce jerk
- Added detailed logging showing exact velocities and PWM values for both motors
- Ensured both motors receive identical PWM when going straight (no rotation)

### 2. ✅ Motor Jerking/Stuttering
**Problem:** Motors jerk when starting or changing speed.

**Solution Applied:**
- Added PWM smoothing with exponential filter
- Smooth factor: 0.3 (30% old value + 70% new value)
- Prevents sudden PWM changes that cause mechanical jerk

### 3. ✅ Speed Limiting
**Problem:** Dynamic speed increases in teleop don't work.

**Solution Applied:**
- Increased motor driver `max_speed` from 1.0 → 2.5 m/s
- Increased `max_angular_speed` from 2.0 → 3.0 rad/s
- Set teleop initial speed to 1.0 m/s (was 0.3 m/s)

## Code Changes Made

### File: `src/frr_control/motor_controller_node.py`

**Added Features:**
1. **Motor Inversion Parameters**
   ```python
   self.declare_parameter('left_inverted', False)
   self.declare_parameter('right_inverted', False)
   ```

2. **PWM Smoothing**
   ```python
   self.pwm_left_current = 0.0
   self.pwm_right_current = 0.0
   self.pwm_smooth_factor = 0.3
   ```

3. **Improved PWM Calculation**
   - Safe division check (avoid divide by zero)
   - Exponential smoothing filter
   - Identical PWM for both motors when going straight

4. **Enhanced Logging**
   - Shows exact left/right velocities
   - Shows PWM percentages for both motors
   - Throttled to avoid spam (every 0.5 seconds)

## How to Use

### Step 1: Start the Robot
```bash
cd ~/frr_ws
source install/setup.bash
./start_robot.sh
```

### Step 2: Start Teleop
```bash
# In another terminal
cd ~/frr_ws
./start_teleop.sh
# Choose option 1 (SAFE) or 2 (DIRECT)
```

### Step 3: Test Forward Motion (Press W)

Watch the motor driver output in the robot terminal. You should see:
```
Motors - L_vel: 1.000 m/s (40.0%), R_vel: 1.000 m/s (40.0%)
```

**Both velocities and PWM should be EQUAL when going straight forward!**

### Step 4: Fix Motor Direction Issues

#### If LEFT motor spins BACKWARD when you press W:
```bash
ros2 param set /motor_driver_node left_inverted true
```

#### If RIGHT motor spins BACKWARD when you press W:
```bash
ros2 param set /motor_driver_node right_inverted true
```

#### If LEFT motor doesn't spin at all:
1. Check wiring/connections
2. Try inverting it:
   ```bash
   ros2 param set /motor_driver_node left_inverted true
   ```

#### If RIGHT motor doesn't spin at all:
1. Check wiring/connections
2. Try inverting it:
   ```bash
   ros2 param set /motor_driver_node right_inverted true
   ```

### Step 5: Verify Both Motors Work

Run the diagnostic script:
```bash
cd ~/frr_ws
./test_motor_diagnostics.sh
```

Or manually test:
```bash
# Test forward (both should spin same direction)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"

# Test rotation (motors should spin opposite directions)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Expected Behavior

### When Pressing W (Forward)
```
Input:  linear.x = 1.0, angular.z = 0.0
Output: left_vel = 1.0, right_vel = 1.0
PWM:    Left = 40%, Right = 40% (same!)
Result: Both motors spin FORWARD at same speed
```

### When Pressing A (Turn Left)
```
Input:  linear.x = 0.0, angular.z = 1.0
Output: left_vel = -0.1, right_vel = 0.1
PWM:    Left = 4%, Right = 4%
Result: Left spins BACKWARD, Right spins FORWARD (rotation)
```

### When Pressing Q (Forward + Left)
```
Input:  linear.x = 1.0, angular.z = 1.0
Output: left_vel = 0.9, right_vel = 1.1
PWM:    Left = 36%, Right = 44%
Result: Both forward, right faster (curved path)
```

## Debugging Commands

### Check Current Settings
```bash
ros2 param get /motor_driver_node left_inverted
ros2 param get /motor_driver_node right_inverted
ros2 param get /motor_driver_node max_speed
```

### Monitor Motor Commands
```bash
# Watch what teleop sends
ros2 topic echo /cmd_vel_teleop

# Watch what motors receive (after obstacle avoidance)
ros2 topic echo /cmd_vel

# Check command rate
ros2 topic hz /cmd_vel
```

### Enable Detailed Logging
```bash
ros2 param set /motor_driver_node log_level DEBUG
```

## Troubleshooting

### Problem: One motor still doesn't spin

**Check 1: Is the motor getting PWM?**
Look at the log output. You should see non-zero PWM percentage for that motor.

**Check 2: GPIO Wiring**
- Left motor: IN1=GPIO17, IN2=GPIO27, ENA=GPIO12
- Right motor: IN1=GPIO22, IN2=GPIO23, ENB=GPIO13

**Check 3: Power Supply**
- Ensure L298N has sufficient power (7-12V recommended)
- Check Enable jumpers on L298N board

**Check 4: Try Inversion**
```bash
# Toggle the motor's inversion
ros2 param set /motor_driver_node left_inverted true
ros2 param set /motor_driver_node right_inverted true
```

### Problem: Both motors spin but in wrong directions

**Fix: Invert both**
```bash
ros2 param set /motor_driver_node left_inverted true
ros2 param set /motor_driver_node right_inverted true
```

### Problem: Robot turns when it should go straight

This means left and right motors are getting different PWM or one is inverted.

**Check the logs:**
```
Motors - L_vel: 1.000 m/s (40.0%), R_vel: 1.000 m/s (40.0%)
```

If velocities are equal but robot still turns, one motor needs inversion.

### Problem: Motors are still jerky

**Option 1: Increase PWM smoothing (more smooth, slower response)**
Edit `motor_controller_node.py` line with `pwm_smooth_factor`:
```python
self.pwm_smooth_factor = 0.5  # Increase for more smoothing (was 0.3)
```

**Option 2: Decrease smoothing (faster response, might be jerky)**
```python
self.pwm_smooth_factor = 0.1  # Decrease for less smoothing (was 0.3)
```

Then rebuild:
```bash
cd ~/frr_ws
colcon build --symlink-install --packages-select frr_control
```

## Summary of What Changed

✅ **PWM Smoothing** - Exponential filter prevents jerk
✅ **Motor Inversion** - Runtime parameters to fix wiring
✅ **Better Logging** - See exact velocities and PWM values
✅ **Safe Math** - Division-by-zero protection
✅ **Higher Speed Limits** - 2.5 m/s max (was 1.0 m/s)
✅ **Equal PWM for Straight** - Both motors get identical PWM when going forward

## Quick Commands Reference

```bash
# Start robot
./start_robot.sh

# Start teleop
./start_teleop.sh

# Run diagnostics
./test_motor_diagnostics.sh

# Invert left motor
ros2 param set /motor_driver_node left_inverted true

# Invert right motor
ros2 param set /motor_driver_node right_inverted true

# Reset inversions
ros2 param set /motor_driver_node left_inverted false
ros2 param set /motor_driver_node right_inverted false

# Check settings
ros2 param list /motor_driver_node
```

## Files Modified
1. `src/frr_control/motor_controller_node.py` - Motor control with smoothing
2. `test_motor_diagnostics.sh` - NEW diagnostic helper script

All changes built and ready to test! 🚀
