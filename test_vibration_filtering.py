#!/usr/bin/env python3
"""
Test script to visualize MPU6050 vibration from LiDAR motor
Shows raw vs filtered data
"""

import smbus
import time
import math
from collections import deque
import numpy as np

class VibrationTester:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.addr = 0x68
        
        # Initialize sensor
        self.init_sensor()
        
        # Filtering parameters
        self.filter_window_size = 5
        self.alpha = 0.7
        
        # Buffers
        self.accel_x_buffer = deque(maxlen=self.filter_window_size)
        self.filtered_ax = 0.0
        
    def init_sensor(self):
        """Initialize MPU6050"""
        try:
            # Wake up
            self.bus.write_byte_data(self.addr, 0x6B, 0x00)
            time.sleep(0.1)
            
            # Set DLPF to 5Hz (filters vibration)
            self.bus.write_byte_data(self.addr, 0x1A, 0x06)
            time.sleep(0.05)
            
            print("✓ MPU6050 initialized with 5Hz DLPF")
        except Exception as e:
            print(f"✗ Failed to initialize: {e}")
            raise
    
    def read_accel_x(self):
        """Read X-axis acceleration"""
        high = self.bus.read_byte_data(self.addr, 0x3B)
        low = self.bus.read_byte_data(self.addr, 0x3C)
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value / 16384.0  # Convert to g
    
    def apply_filters(self, raw_value):
        """Apply moving average + low-pass filter"""
        # Moving average
        self.accel_x_buffer.append(raw_value)
        ma_value = sum(self.accel_x_buffer) / len(self.accel_x_buffer)
        
        # Low-pass filter
        self.filtered_ax = self.alpha * ma_value + (1 - self.alpha) * self.filtered_ax
        
        return ma_value, self.filtered_ax
    
    def test_vibration(self, duration=10):
        """Test vibration levels for specified duration"""
        print("\n" + "="*70)
        print("MPU6050 Vibration Analysis")
        print("="*70)
        print("Testing for {} seconds...".format(duration))
        print("Legend: RAW = unfiltered, MA = moving average, FILT = low-pass filtered")
        print("-"*70)
        print(f"{'Time':<6} {'RAW (g)':<12} {'MA (g)':<12} {'FILT (g)':<12} {'Noise':<8}")
        print("-"*70)
        
        raw_values = []
        filtered_values = []
        start_time = time.time()
        sample_count = 0
        
        try:
            while time.time() - start_time < duration:
                # Read raw value
                raw = self.read_accel_x()
                
                # Apply filters
                ma, filtered = self.apply_filters(raw)
                
                # Store for statistics
                raw_values.append(raw)
                filtered_values.append(filtered)
                
                # Calculate instantaneous noise
                noise = abs(raw - filtered)
                
                # Print every 10 samples (every ~0.2 seconds)
                if sample_count % 10 == 0:
                    elapsed = time.time() - start_time
                    print(f"{elapsed:5.1f}s {raw:+.4f}      {ma:+.4f}      {filtered:+.4f}      {noise:.4f}")
                
                sample_count += 1
                time.sleep(0.02)  # 50 Hz
                
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        
        # Calculate statistics
        print("-"*70)
        print("\n📊 VIBRATION STATISTICS:")
        print("="*70)
        
        raw_std = np.std(raw_values)
        filtered_std = np.std(filtered_values)
        noise_reduction = ((raw_std - filtered_std) / raw_std) * 100
        
        raw_peak_to_peak = max(raw_values) - min(raw_values)
        filtered_peak_to_peak = max(filtered_values) - min(filtered_values)
        
        print(f"Raw Data:")
        print(f"  Mean:           {np.mean(raw_values):+.4f}g")
        print(f"  Std Dev:        {raw_std:.4f}g")
        print(f"  Peak-to-Peak:   {raw_peak_to_peak:.4f}g")
        print(f"  Min/Max:        {min(raw_values):+.4f}g / {max(raw_values):+.4f}g")
        print()
        print(f"Filtered Data:")
        print(f"  Mean:           {np.mean(filtered_values):+.4f}g")
        print(f"  Std Dev:        {filtered_std:.4f}g")
        print(f"  Peak-to-Peak:   {filtered_peak_to_peak:.4f}g")
        print(f"  Min/Max:        {min(filtered_values):+.4f}g / {max(filtered_values):+.4f}g")
        print()
        print(f"📈 Noise Reduction: {noise_reduction:.1f}%")
        print(f"📉 Vibration Smoothing: {(1 - filtered_peak_to_peak/raw_peak_to_peak)*100:.1f}%")
        
        # Interpretation
        print("\n🔍 INTERPRETATION:")
        print("="*70)
        
        if raw_std > 0.05:
            print("⚠️  HIGH VIBRATION DETECTED!")
            print("   Raw std dev > 0.05g indicates significant vibration")
            print("   This is likely from the LiDAR motor spinning")
        else:
            print("✓ Vibration levels are low")
        
        if noise_reduction > 50:
            print(f"✓ Filter is working well ({noise_reduction:.0f}% noise reduction)")
        elif noise_reduction > 30:
            print(f"⚠️  Filter is helping but could be better ({noise_reduction:.0f}% reduction)")
        else:
            print(f"⚠️  Filter not very effective ({noise_reduction:.0f}% reduction)")
        
        if filtered_std < 0.02:
            print("✓ Filtered data is smooth and usable for navigation")
        else:
            print("⚠️  Filtered data still has noticeable vibration")
        
        print("\n💡 RECOMMENDATIONS:")
        print("="*70)
        
        if raw_std > 0.1:
            print("1. LiDAR vibration is VERY HIGH - consider:")
            print("   - Damping material between LiDAR and rover")
            print("   - Rubber standoffs for LiDAR mounting")
            print("   - Check if LiDAR is unbalanced")
        
        if filtered_std > 0.03:
            print("2. Even with filtering, vibration is noticeable:")
            print("   - Increase filter window size (5 → 10)")
            print("   - Decrease alpha (0.7 → 0.5) for more smoothing")
            print("   - Add hardware damping")
        
        if noise_reduction < 40:
            print("3. Filter effectiveness can be improved:")
            print("   - Check DLPF setting (should be 5Hz)")
            print("   - Verify sensor mounting is secure")
            print("   - Consider kalman filter for better results")
        
        print("\n" + "="*70)
        print(f"Total samples collected: {sample_count}")
        print("="*70)

def main():
    print("="*70)
    print("MPU6050 Vibration Test Tool")
    print("="*70)
    print()
    print("This tool will:")
    print("  1. Read raw MPU6050 data")
    print("  2. Apply vibration filtering")
    print("  3. Show the difference")
    print("  4. Analyze vibration levels")
    print()
    print("Start the LiDAR first if you want to test with it running!")
    print()
    
    input("Press Enter to start testing (Ctrl+C to stop early)...")
    
    try:
        tester = VibrationTester()
        tester.test_vibration(duration=10)
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        return 1
    
    print("\nTest complete!")
    return 0

if __name__ == '__main__':
    exit(main())
