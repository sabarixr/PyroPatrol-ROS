# Teleop and LiDAR Auto-Detection Fix

## Problem Fixed
1. **Rover doesn't move** - Obstacle avoidance node only forwarded commands when LiDAR scans arrived
2. **Dynamic speed doesn't work** - Motor driver limited to 1.0 m/s, teleop started at 0.3 m/s

## Changes Made

### 1. Obstacle Avoidance Node - Auto LiDAR Detection
**File:** `src/frr_sensors/frr_sensors/obstacle_avoidance_node.py`

**What Changed:**
- ✅ Added automatic LiDAR detection with 3-second timeout
- ✅ Two automatic modes:
  - **OBSTACLE AVOIDANCE MODE**: When LiDAR is detected and publishing scans
  - **PASSTHROUGH MODE**: When no LiDAR detected - forwards all teleop commands unchanged
- ✅ Periodic 10Hz publisher ensures commands always reach motors
- ✅ Status logging shows current mode

**How It Works:**
```
┌─────────────────────────────────────────────────┐
│  LiDAR Detected?                                │
│  (received scan in last 3 seconds)              │
└────────────┬────────────────────────────────────┘
             │
    ┌────────┴─────────┐
    │ YES              │ NO
    ▼                  ▼
┌───────────────┐  ┌──────────────────┐
│ OBSTACLE      │  │ PASSTHROUGH      │
│ AVOIDANCE     │  │ MODE             │
│ ACTIVE        │  │                  │
│               │  │ Forward all      │
│ Check zones,  │  │ teleop commands  │
│ stop if       │  │ directly to      │
│ obstacles     │  │ motors           │
└───────────────┘  └──────────────────┘
```

### 2. Teleop Node - Fixed Initial Speed
**File:** `src/frr_control/frr_control/teleop_node_clean.py`

**What Changed:**
- ✅ Initial linear speed: 0.3 m/s → **1.0 m/s**
- ✅ Initial angular speed: 0.6 rad/s → **1.0 rad/s**

### 3. Motor Driver - Increased Speed Limits
**File:** `src/frr_control/motor_controller_node.py`

**What Changed:**
- ✅ Max linear speed: 1.0 m/s → **2.5 m/s**
- ✅ Max angular speed: 2.0 rad/s → **3.0 rad/s**

Now teleop dynamic speed increases (R/T keys) will work up to 2.5 m/s!

## How to Use

### Quick Start (Normal Usage)
```bash
# Terminal 1: Start robot system
cd ~/frr_ws
./start_robot.sh

# Terminal 2: Start teleop (will auto-detect LiDAR)
./start_teleop.sh
# Choose option 1 (SAFE mode - recommended)
```

### What You'll See

**With LiDAR Connected:**
```
⚠️  LiDAR auto-detection enabled - will switch to passthrough if no LiDAR detected
✓ LiDAR detected - obstacle avoidance ACTIVE
```

**Without LiDAR (or LiDAR fails):**
```
⚠️  LiDAR auto-detection enabled - will switch to passthrough if no LiDAR detected
⚠️  No LiDAR detected - running in PASSTHROUGH mode (no obstacle avoidance)
```

**If LiDAR disconnects while running:**
```
⚠️  LiDAR data lost (no scan for 3.2s) - switching to PASSTHROUGH mode
```

### Teleop Controls (Unchanged)
- **Movement:** W/S/A/D + Q/E/Z/C for diagonals
- **Speed Control:**
  - `R` - Increase all speeds
  - `F` - Decrease all speeds
  - `T` - Increase linear speed only
  - `G` - Decrease linear speed only
  - `Y` - Increase angular speed
  - `H` - Decrease angular speed
  - `1/2/3` - Speed presets (Slow/Normal/Fast)

### Initial Speeds
- **Linear:** 1.0 m/s (was 0.3 m/s)
- **Angular:** 1.0 rad/s (was 0.6 rad/s)
- **Maximum Linear:** 2.5 m/s (was 1.0 m/s)
- **Maximum Angular:** 3.0 rad/s (was 2.0 rad/s)

## Testing Commands

### Check if nodes are running
```bash
source ~/frr_ws/install/setup.bash
ros2 node list
```
Should show:
- `/motor_driver_node`
- `/obstacle_avoidance_node`
- `/teleop_node`

### Check LiDAR status
```bash
# Check if /scan topic exists
ros2 topic list | grep scan

# Check scan frequency
ros2 topic hz /scan

# If no scan, obstacle avoidance will auto-switch to PASSTHROUGH
```

### Monitor teleop commands
```bash
# Watch teleop output
ros2 topic hz /cmd_vel_teleop

# Watch final motor commands
ros2 topic hz /cmd_vel

# Check motor speed limits
ros2 param get /motor_driver_node max_speed
```

### Change motor speed limit at runtime (optional)
```bash
# Increase to 3.0 m/s
ros2 param set /motor_driver_node max_speed 3.0

# Increase angular
ros2 param set /motor_driver_node max_angular_speed 4.0
```

## Troubleshooting

### Rover still doesn't move
1. **Check motor driver is running:**
   ```bash
   ros2 node list | grep motor_driver
   ```

2. **Check if commands reach motor driver:**
   ```bash
   ros2 topic echo /cmd_vel --once
   ```
   Press W in teleop - you should see a Twist message

3. **Check motor driver logs:**
   Look at the terminal where robot is running for GPIO errors

### LiDAR keeps switching modes
- LiDAR might be intermittent
- Check LiDAR connection/power
- Check `/scan` frequency:
  ```bash
  ros2 topic hz /scan
  ```
  Should be ~5-10 Hz for stable operation

### Speed doesn't increase beyond certain point
- Motor driver has hardware PWM limits (100%)
- Voltage/battery affects actual speed
- Check current max_speed parameter:
  ```bash
  ros2 param get /motor_driver_node max_speed
  ```

## Summary
✅ **Auto LiDAR detection** - No more manual mode switching
✅ **Passthrough mode** - Rover works even without LiDAR
✅ **Initial speed 1.0 m/s** - Starts at reasonable speed
✅ **Dynamic speed up to 2.5 m/s** - R/T keys work properly
✅ **Smooth operation** - No stop/start when LiDAR absent

## Files Modified
1. `src/frr_sensors/frr_sensors/obstacle_avoidance_node.py`
2. `src/frr_control/frr_control/teleop_node_clean.py`
3. `src/frr_control/motor_controller_node.py`

All changes tested and built successfully! ✓
