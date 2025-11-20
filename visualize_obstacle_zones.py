#!/usr/bin/env python3
"""
Visual demonstration of obstacle detection zones
Run this to see how the 4-corner detection system works
"""

import math

def print_detection_zones():
    print("=" * 70)
    print("OBSTACLE DETECTION ZONE VISUALIZATION")
    print("=" * 70)
    print()
    
    # Rover dimensions
    print("🤖 ROVER SPECIFICATIONS:")
    print("   - Length: 30 cm")
    print("   - Width: 20 cm")
    print()
    print("📡 LIDAR POSITION (from center):")
    print("   - Front: 10.0 cm")
    print("   - Back: 16.0 cm")
    print("   - Left: 8.0 cm")
    print("   - Right: 10.5 cm")
    print()
    
    # Detection zones
    print("🎯 DETECTION ZONES:")
    print()
    print("         FRONT ZONE (-45° to +45°)")
    print("              ┌─────────┐")
    print("              │ 🚫 STOP │")
    print("              │ IF FWD  │")
    print("         ═════╪═════════╪═════")
    print("              │         │")
    print("  LEFT ZONE   │    🤖   │   RIGHT ZONE")
    print("  (-135° to   │   LiDAR │   (45° to 135°)")
    print("   -45°)      │    📡   │")
    print("  🚫 STOP     │         │   🚫 STOP")
    print("  IF LEFT     │         │   IF RIGHT")
    print("         ═════╪═════════╪═════")
    print("              │ 🚫 STOP │")
    print("              │ IF REV  │")
    print("              └─────────┘")
    print("         BACK ZONE (135° to 225°)")
    print()
    
    # Safety margins
    print("⚠️  SAFETY MARGINS:")
    print("   - Safety Margin: 15 cm (configurable)")
    print("   - Emergency Stop: 30 cm (configurable)")
    print()
    
    # Motion examples
    print("🎮 MOTION SAFETY EXAMPLES:")
    print()
    
    examples = [
        ("Forward", "Checks FRONT zone", "Stops if obstacle ahead"),
        ("Reverse", "Checks BACK zone", "Stops if obstacle behind"),
        ("Turn Left", "Checks LEFT zone", "Stops turn, suggests right"),
        ("Turn Right", "Checks RIGHT zone", "Stops turn, suggests left"),
        ("Forward + Left", "Checks FRONT & LEFT", "Stops if either blocked"),
        ("Forward + Right", "Checks FRONT & RIGHT", "Stops if either blocked"),
        ("Reverse + Left", "Checks BACK & RIGHT*", "Stops if either blocked"),
        ("Reverse + Right", "Checks BACK & LEFT*", "Stops if either blocked"),
    ]
    
    for motion, zones, action in examples:
        print(f"   {motion:15s} → {zones:25s} → {action}")
    
    print()
    print("   * Note: In reverse, steering is inverted")
    print()
    
    # Emergency scenarios
    print("🚨 EMERGENCY STOP SCENARIOS:")
    print("   1. Obstacle within 30cm (emergency threshold)")
    print("   2. Obstacles on opposite sides (trapped)")
    print("      - Front AND Back blocked")
    print("      - Left AND Right blocked")
    print()
    
    # Angle calculation example
    print("📐 ANGLE CALCULATION EXAMPLE:")
    print()
    print("   LiDAR reading at angle θ = 30° with distance r = 0.5m")
    print()
    print("   Cartesian coordinates:")
    print(f"   x = r × cos(θ) = 0.5 × cos(30°) = {0.5 * math.cos(math.radians(30)):.3f}m")
    print(f"   y = r × sin(θ) = 0.5 × sin(30°) = {0.5 * math.sin(math.radians(30)):.3f}m")
    print()
    print("   Zone determination:")
    print("   θ = 30° → -45° ≤ 30° ≤ 45° → FRONT ZONE ✓")
    print()
    print("   Obstacle check:")
    print("   x = 0.433m > 0 (in front)")
    print("   x < (0.10 + 0.15 + 0.15) = 0.40m → Obstacle detected! 🚫")
    print("             ↑     ↑     ↑")
    print("          LiDAR  Rover  Safety")
    print("         Offset Length Margin")
    print()
    
    # Top view diagram
    print("🗺️  TOP VIEW DIAGRAM:")
    print()
    print("                    ↑ X (Forward)")
    print("                    │")
    print("                    │")
    print("         ┌──────────┼──────────┐")
    print("         │          │          │")
    print("         │        📡 LiDAR     │  ← 10cm forward")
    print("         │          │          │")
    print("    Y ←──┼──────────●──────────┼── 8cm left")
    print("  (Left) │      Rover Center   │")
    print("         │                     │  ← 10.5cm right")
    print("         │                     │")
    print("         └─────────────────────┘")
    print("                    │")
    print("                    │  ← 16cm back")
    print()
    print("         All obstacles detected relative to LiDAR position")
    print("         All dimensions compensated in software")
    print()
    
    # Performance specs
    print("⚡ PERFORMANCE SPECIFICATIONS:")
    print("   - Update Rate: 50 Hz (20ms per cycle)")
    print("   - LiDAR Range: 0.15m to 12m")
    print("   - Angular Resolution: ~4° per measurement")
    print("   - Processing Latency: <5ms")
    print("   - Response Time: <40ms (worst case)")
    print()
    
    # Configuration
    print("⚙️  RUNTIME CONFIGURATION:")
    print()
    print("   # Make obstacle avoidance more conservative:")
    print("   ros2 param set /obstacle_avoidance_node safety_margin 0.20")
    print()
    print("   # Make obstacle avoidance more aggressive:")
    print("   ros2 param set /obstacle_avoidance_node safety_margin 0.10")
    print()
    print("   # Change emergency stop distance:")
    print("   ros2 param set /obstacle_avoidance_node stop_distance 0.40")
    print()
    
    print("=" * 70)
    print()

def test_zone_classification():
    """Test zone classification for various angles"""
    print("🧪 ZONE CLASSIFICATION TEST:")
    print()
    print("   Angle  →  Zone")
    print("   " + "-" * 30)
    
    test_angles = [0, 30, 45, 60, 90, 120, 135, 150, 180, -30, -45, -60, -90, -120, -135, -150]
    
    for angle in test_angles:
        if -45 <= angle <= 45:
            zone = "FRONT"
        elif 45 < angle <= 135:
            zone = "RIGHT"
        elif angle > 135 or angle < -135:
            zone = "BACK"
        elif -135 <= angle < -45:
            zone = "LEFT"
        else:
            zone = "UNKNOWN"
        
        print(f"   {angle:4d}°  →  {zone}")
    
    print()

def main():
    print()
    print_detection_zones()
    test_zone_classification()
    print()
    print("💡 TIP: Run the actual system with:")
    print("   ros2 launch frr_bringup lidar_navigation.launch.py")
    print()
    print("📊 Monitor obstacle detection with:")
    print("   ros2 topic echo /obstacle_detected")
    print()
    print("🔍 Visualize in RViz (if available):")
    print("   rviz2")
    print("   Add: LaserScan topic (/scan)")
    print("   Add: Odometry topic (/lidar_odom)")
    print()

if __name__ == "__main__":
    main()
