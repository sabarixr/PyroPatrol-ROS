#!/bin/bash
# Show teleop quick help

cat << 'EOF'

╔══════════════════════════════════════════════════════════════════════╗
║                  TELEOP - NOW FIXED AND SIMPLE!                       ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║  THE PROBLEM (Before):                                                ║
║    ❌ Teleop doesn't work with ros2 launch                           ║
║    ❌ Messy terminal output                                          ║
║    ❌ Confusing obstacle avoidance setup                             ║
║                                                                       ║
║  THE SOLUTION (Now):                                                  ║
║    ✅ One simple command that works every time                       ║
║    ✅ Clean, organized interface                                     ║
║    ✅ Easy obstacle avoidance toggle                                 ║
║                                                                       ║
╠══════════════════════════════════════════════════════════════════════╣
║  HOW TO START TELEOP (Recommended):                                   ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║    cd ~/frr_ws                                                        ║
║    ./start_teleop.sh                                                  ║
║                                                                       ║
║  That's it! The script will:                                          ║
║    • Check if robot is running                                        ║
║    • Ask about obstacle avoidance                                     ║
║    • Start everything correctly                                       ║
║    • Give you clean controls                                          ║
║                                                                       ║
╠══════════════════════════════════════════════════════════════════════╣
║  ALTERNATIVE METHODS:                                                 ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║  Manual with obstacle avoidance:                                      ║
║    ros2 run frr_control teleop_node_clean \                          ║
║         --ros-args -p with_avoidance:=true                           ║
║                                                                       ║
║  Manual without obstacle avoidance:                                   ║
║    ros2 run frr_control teleop_node_clean \                          ║
║         --ros-args -p with_avoidance:=false                          ║
║                                                                       ║
║  Launch file (advanced):                                              ║
║    ros2 launch frr_bringup teleop_simple.launch.py                   ║
║                                                                       ║
╠══════════════════════════════════════════════════════════════════════╣
║  CONTROLS (When teleop is running):                                   ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║  Movement:           Camera:           Speed:                         ║
║    Q   W   E           I = Up           R/F = All                     ║
║    A   S   D           K = Down         T/G = Linear                  ║
║    Z   X   C           U = Center       Y/H = Angular                 ║
║                        O/L = Look       1/2/3 = Presets               ║
║  W/S = Forward/Back                                                   ║
║  A/D = Turn left/right    Special:                                    ║
║  X = Stop                  SPACE = Emergency stop                     ║
║  Q/E/Z/C = Diagonals       ? = Toggle help                            ║
║                            ESC = Quit                                 ║
║                                                                       ║
╠══════════════════════════════════════════════════════════════════════╣
║  OBSTACLE AVOIDANCE:                                                  ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║  SAFE Mode (with avoidance):                                          ║
║    ✓ Robot stops before hitting obstacles                            ║
║    ✓ Shows real-time status: CLEAR or BLOCKED                        ║
║    ✓ Recommended for normal use                                      ║
║                                                                       ║
║  DIRECT Mode (without avoidance):                                     ║
║    ⚠ Robot goes wherever you command                                 ║
║    ⚠ No automatic stopping                                           ║
║    ⚠ Use in open areas only                                          ║
║                                                                       ║
╠══════════════════════════════════════════════════════════════════════╣
║  TROUBLESHOOTING:                                                     ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║  "No response to keys":                                               ║
║    → Use ./start_teleop.sh (not ros2 launch)                         ║
║    → Make sure you're in an interactive terminal                     ║
║                                                                       ║
║  "Robot doesn't move":                                                ║
║    → Check robot is running: ros2 node list                          ║
║    → In SAFE mode, check if path is blocked                          ║
║    → Try DIRECT mode to test motors                                  ║
║                                                                       ║
║  "Teleop not found":                                                  ║
║    → Rebuild: cd ~/frr_ws && colcon build --packages-select \        ║
║                     frr_control && source install/setup.bash         ║
║                                                                       ║
╠══════════════════════════════════════════════════════════════════════╣
║  COMPLETE WORKFLOW:                                                   ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║  1. Start the robot:                                                  ║
║     cd ~/frr_ws                                                       ║
║     source install/setup.bash                                         ║
║     ros2 launch frr_bringup rover_bringup.launch.py                  ║
║                                                                       ║
║  2. In another terminal, start teleop:                                ║
║     cd ~/frr_ws                                                       ║
║     source install/setup.bash                                         ║
║     ./start_teleop.sh                                                 ║
║                                                                       ║
║  3. Choose mode:                                                      ║
║     1) SAFE mode (with obstacle avoidance) ← Recommended             ║
║     2) DIRECT mode (no obstacle avoidance)                           ║
║                                                                       ║
║  4. Drive the robot!                                                  ║
║     Use W/A/S/D keys to move                                          ║
║     Press SPACE for emergency stop                                    ║
║     Press ESC or Ctrl+C to quit                                       ║
║                                                                       ║
╠══════════════════════════════════════════════════════════════════════╣
║  FILES:                                                               ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║  📄 start_teleop.sh                      ← USE THIS!                 ║
║  📄 TELEOP_FIXED.md                      ← Full documentation        ║
║  📄 src/frr_control/frr_control/teleop_node_clean.py                 ║
║  📄 src/frr_bringup/launch/teleop_simple.launch.py                   ║
║                                                                       ║
╚══════════════════════════════════════════════════════════════════════╝

For complete documentation, see: TELEOP_FIXED.md

EOF
