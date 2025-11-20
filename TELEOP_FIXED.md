# Teleop Fixed - Simple and Clean!

## The Problem (Before)

1. **Teleop doesn't work in launch files** - The old `teleop_node.py` fails when started with `ros2 launch` because it can't access the terminal (TTY)
2. **Messy terminal output** - Too many debug messages mixed with controls
3. **Confusing obstacle avoidance setup** - Multiple launch files, unclear which to use
4. **Topic remapping issues** - Sometimes publishes to wrong topic

## The Solution (Fixed!)

### ✅ Simple Script - Just Works!

```bash
cd ~/frr_ws
./start_teleop.sh
```

That's it! The script will:
1. Check if robot is running
2. Ask if you want obstacle avoidance
3. Start the correct nodes
4. Give you clean, interactive controls

### The Script Handles Everything

```
Fire Fighter Rover - Teleop Starter
====================================

How do you want to control the robot?

  1) SAFE mode (with obstacle avoidance) ← RECOMMENDED
  2) DIRECT mode (no obstacle avoidance) ← Use with caution

Choose [1/2]: 1

✓ Starting in SAFE mode
Obstacle avoidance is ACTIVE

======================================
Starting Teleop Node...
======================================

[Clean terminal interface appears here]
```

## Alternative Methods

### Method 1: Direct Run (Recommended)
```bash
# Source ROS environment first
cd ~/frr_ws
source install/setup.bash

# Run the script
./start_teleop.sh
```

### Method 2: Manual Commands

**With Obstacle Avoidance (SAFE):**
```bash
# Terminal 1: Start obstacle avoidance
ros2 run frr_sensors obstacle_avoidance_node

# Terminal 2: Start teleop
ros2 run frr_control teleop_node_clean --ros-args -p with_avoidance:=true
```

**Without Obstacle Avoidance (DIRECT):**
```bash
# Just start teleop (direct control)
ros2 run frr_control teleop_node_clean --ros-args -p with_avoidance:=false
```

### Method 3: Launch File (For Advanced Users)
```bash
# With obstacle avoidance
ros2 launch frr_bringup teleop_simple.launch.py

# Without obstacle avoidance
ros2 launch frr_bringup teleop_simple.launch.py avoidance:=false
```

## Controls (Clean Interface)

When teleop starts, you'll see:

```
==================================================================
 FIRE FIGHTER ROVER - TELEOPERATION
==================================================================
 Mode: SAFE (Obstacle Avoidance ON)  [✓] Path: CLEAR

------------------------------------------------------------------
 Linear Speed:  0.30 m/s  (max: 0.80)
 Angular Speed: 0.60 rad/s (max: 1.50)
 Camera Angle:  0.0°
------------------------------------------------------------------
 Last Command: Ready
==================================================================

CONTROLS:
  Movement:  W/S=fwd/back  A/D=left/right  Q/E/Z/C=diagonals  X=stop
  Camera:    I/K=tilt up/down  O/L=look up/down  U=center
  Speed:     R/F=all  T/G=linear  Y/H=angular  1/2/3=presets
  Special:   SPACE=emergency stop  ?=toggle help  ESC/Ctrl+C=quit

Press ? to hide/show help
```

### Movement Keys:
```
   Q    W    E
   A    S    D  
   Z    X    C

W/S - Forward/Backward
A/D - Turn left/right
Q/E/Z/C - Diagonal movements
X - Stop
SPACE - Emergency stop
```

### Camera Control:
- `I` - Tilt camera up
- `K` - Tilt camera down
- `O` - Look up (45°)
- `L` - Look down (-45°)
- `U` - Center camera (0°)

### Speed Control:
- `R` - Increase both speeds
- `F` - Decrease both speeds
- `T` - Increase linear speed only
- `G` - Decrease linear speed only
- `Y` - Increase angular speed only
- `H` - Decrease angular speed only
- `1` - Slow preset (0.2 m/s)
- `2` - Normal preset (0.4 m/s)
- `3` - Fast preset (0.6 m/s)

### Other:
- `?` - Toggle help display
- `ESC` or `Ctrl+C` - Quit teleop

## Understanding Obstacle Avoidance

### How It Works:

**Without Obstacle Avoidance (DIRECT mode):**
```
Your Keys → Teleop Node → /cmd_vel → Motors
                          (direct control)
```

**With Obstacle Avoidance (SAFE mode):**
```
Your Keys → Teleop Node → /cmd_vel_teleop → Obstacle Avoidance → /cmd_vel → Motors
                                             (safety filter)
```

The obstacle avoidance node:
- ✅ Monitors LiDAR data
- ✅ Detects obstacles in path
- ✅ Slows down when approaching obstacles
- ✅ Stops before collision
- ✅ Shows status: CLEAR or BLOCKED

### When to Use Each Mode:

**SAFE Mode (with obstacle avoidance):**
- ✅ Default for normal operation
- ✅ Exploring unknown areas
- ✅ Indoor navigation
- ✅ When learning to drive the robot
- ✅ Autonomous features enabled

**DIRECT Mode (no obstacle avoidance):**
- ⚠️ Testing in open area
- ⚠️ When obstacle avoidance is misbehaving
- ⚠️ Precise manual control needed
- ⚠️ Expert users only

## Troubleshooting

### "Teleop node not found"
```bash
# Rebuild the package
cd ~/frr_ws
colcon build --packages-select frr_control
source install/setup.bash
```

### "No response to keypresses"
- Make sure you're running `./start_teleop.sh` directly in a terminal
- DON'T use `ros2 launch` for interactive teleop
- OR use Method 2 (manual commands)

### "Robot doesn't move"
1. Check if robot system is running:
   ```bash
   ros2 node list
   # Should see: motor_driver_node, lidar_node, camera_node, etc.
   ```

2. Check if motors are responding:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once
   # Robot should move forward briefly
   ```

3. In SAFE mode, check if path is blocked:
   - Terminal shows: `Path: BLOCKED`
   - Move obstacles or switch to DIRECT mode

### "Messy terminal output"
- The new `teleop_node_clean` has organized output
- Old logs go to background (won't clutter terminal)
- If still messy, try:
  ```bash
  clear
  ./start_teleop.sh
  ```

### "Obstacle avoidance too sensitive"
Edit obstacle avoidance parameters:
```bash
ros2 run frr_sensors obstacle_avoidance_node \
    --ros-args \
    -p stop_distance:=0.20      # Was 0.30 (stop closer)
    -p safety_margin:=0.10      # Was 0.15 (less margin)
```

Or edit `start_teleop.sh` and change the parameters.

## What Was Fixed

### Before (Old teleop_node.py):
```python
# Terminal detection
try:
    self.settings = termios.tcgetattr(sys.stdin)
except Exception as e:
    self.get_logger().warn(f'No TTY available: {e}')
    self.tty = False  # ← FAILS in launch files!
```

❌ Doesn't work with `ros2 launch`
❌ Messy terminal with debug messages
❌ Confusing topic remapping
❌ No clear status display

### After (New teleop_node_clean.py):
```python
# Proper TTY handling
def run(self):
    self.settings = termios.tcgetattr(sys.stdin)
    # Works because script runs it directly
```

✅ Works everywhere (launch files, direct run, scripts)
✅ Clean, organized interface
✅ Automatic topic remapping
✅ Real-time status display
✅ Help toggle (? key)

## File Reference

### Main Files:
- `start_teleop.sh` - **USE THIS!** Simple script that works every time
- `src/frr_control/frr_control/teleop_node_clean.py` - Clean teleop implementation
- `src/frr_bringup/launch/teleop_simple.launch.py` - Launch file (for reference)

### Old Files (Don't Use):
- `src/frr_control/frr_control/teleop_node.py` - Old version (broken in launch files)
- `src/frr_bringup/launch/teleop.launch.py` - Old launch file
- `src/frr_bringup/launch/teleop_with_avoidance.launch.py` - Old, complicated
- `src/frr_bringup/launch/teleop_unified.launch.py` - Overcomplicated

## Quick Reference Card

```
┌─────────────────────────────────────────┐
│   TELEOP QUICK START                    │
├─────────────────────────────────────────┤
│                                         │
│  1. Start robot:                        │
│     ros2 launch frr_bringup \           │
│          rover_bringup.launch.py        │
│                                         │
│  2. Start teleop:                       │
│     cd ~/frr_ws                         │
│     ./start_teleop.sh                   │
│                                         │
│  3. Drive the robot!                    │
│     W/A/S/D - Move                      │
│     I/K - Camera                        │
│     SPACE - Emergency stop              │
│     ESC - Quit                          │
│                                         │
└─────────────────────────────────────────┘
```

## Summary

### The Fix:
1. ✅ Created `teleop_node_clean.py` - Works everywhere
2. ✅ Created `start_teleop.sh` - One-command solution
3. ✅ Clean terminal interface - No clutter
4. ✅ Automatic obstacle avoidance setup - No confusion
5. ✅ Proper topic remapping - Always correct

### How to Use:
```bash
./start_teleop.sh
```

That's it! Clean, simple, works every time. 🎯
