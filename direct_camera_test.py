#!/usr/bin/env python3
"""
Direct camera test without ROS - testing V4L2 access on Ubuntu 22.04
"""

import cv2
import sys
import os

def test_camera_direct():
    print("🚒 Direct Camera Test (No ROS)")
    print("=" * 40)
    
    # Test camera device
    camera_device = '/dev/video0'
    
    if not os.path.exists(camera_device):
        print(f"❌ Camera device {camera_device} not found")
        return False
    
    print(f"✅ Camera device {camera_device} exists")
    
    # Try to open with V4L2 backend only
    try:
        print(f"🔍 Opening camera with V4L2 backend...")
        cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        
        if not cap.isOpened():
            print("❌ Failed to open camera")
            return False
            
        print("✅ Camera opened successfully")
        
        # Set minimal properties
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        cap.set(cv2.CAP_PROP_FPS, 10)
        
        # Get actual properties
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        
        print(f"📐 Resolution: {width}x{height}")
        print(f"⏱️  FPS: {fps}")
        
        # Try to capture a frame
        print("📸 Attempting to capture frame...")
        ret, frame = cap.read()
        
        if not ret or frame is None:
            print("❌ Failed to capture frame")
            cap.release()
            return False
            
        print(f"✅ Frame captured successfully: {frame.shape}")
        
        # Capture a few more frames to test stability
        for i in range(5):
            ret, frame = cap.read()
            if ret and frame is not None:
                print(f"✅ Frame {i+2} captured: {frame.shape}")
            else:
                print(f"❌ Frame {i+2} failed")
                break
        
        cap.release()
        print("🎉 Camera test completed successfully!")
        return True
        
    except Exception as e:
        print(f"❌ Camera test failed with exception: {str(e)}")
        return False

if __name__ == '__main__':
    success = test_camera_direct()
    sys.exit(0 if success else 1)
