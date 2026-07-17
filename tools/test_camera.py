#!/usr/bin/env python3
"""
Test script to find working camera device on Raspberry Pi
"""

import cv2
import sys

def test_camera(device_path):
    """Test if a camera device works"""
    print(f"\nTesting {device_path}...")
    
    try:
        # Try to open the camera
        cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        
        if not cap.isOpened():
            print(f"  ❌ Failed to open {device_path}")
            return False
        
        # Try to read a frame
        ret, frame = cap.read()
        
        if not ret or frame is None:
            print(f"  ❌ Opened but failed to read frame from {device_path}")
            cap.release()
            return False
        
        # Success!
        height, width = frame.shape[:2]
        print(f"  ✅ SUCCESS! Got frame: {width}x{height}")
        print(f"     Device: {device_path}")
        print(f"     Resolution: {width}x{height}")
        print(f"     Frame shape: {frame.shape}")
        
        # Save a test image
        test_image = f'/tmp/test_camera_{device_path.replace("/dev/video", "")}.jpg'
        cv2.imwrite(test_image, frame)
        print(f"     Saved test image: {test_image}")
        
        cap.release()
        return True
        
    except Exception as e:
        print(f"  ❌ Exception: {e}")
        return False

def main():
    print("=" * 60)
    print("Raspberry Pi Camera Test")
    print("=" * 60)
    
    # Test common video devices
    devices_to_test = [
        '/dev/video0',
        '/dev/video10',
        '/dev/video11',
        '/dev/video12',
        '/dev/video13',
        '/dev/video14',
        '/dev/video15',
        '/dev/video16',
        '/dev/video18',
        '/dev/video20',
        '/dev/video21',
        '/dev/video22',
        '/dev/video23',
        '/dev/video31',
    ]
    
    working_devices = []
    
    for device in devices_to_test:
        if test_camera(device):
            working_devices.append(device)
    
    print("\n" + "=" * 60)
    print("RESULTS:")
    print("=" * 60)
    
    if working_devices:
        print(f"✅ Found {len(working_devices)} working camera device(s):")
        for device in working_devices:
            print(f"   - {device}")
        print(f"\nRecommendation: Use {working_devices[0]} in your launch file")
    else:
        print("❌ No working camera devices found!")
        print("\nTroubleshooting:")
        print("  1. Check camera connection: vcgencmd get_camera")
        print("  2. Enable camera in raspi-config")
        print("  3. Try: sudo modprobe bcm2835-v4l2")
        print("  4. Reboot the Pi")
    
    print("=" * 60)

if __name__ == '__main__':
    main()
