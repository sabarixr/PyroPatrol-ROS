#!/usr/bin/env python3
"""
Interactive Camera Settings Adjuster
Helps fine-tune exposure, contrast, and other camera settings
"""

import cv2
import numpy as np

def test_camera_settings():
    print("🚒 Camera Settings Adjuster")
    print("=" * 40)
    
    # Open camera
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ Failed to open camera")
        return
    
    # Initial settings
    settings = {
        'exposure': -6,      # -11 to -1
        'brightness': 0,     # -64 to 64 
        'contrast': 32,      # 0 to 100
        'saturation': 50,    # 0 to 100
        'gain': 0,           # 0 to 100
        'wb_temp': 4000      # 2800 to 6500
    }
    
    def apply_settings():
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual exposure
        cap.set(cv2.CAP_PROP_EXPOSURE, settings['exposure'])
        cap.set(cv2.CAP_PROP_BRIGHTNESS, settings['brightness'])
        cap.set(cv2.CAP_PROP_CONTRAST, settings['contrast'])
        cap.set(cv2.CAP_PROP_SATURATION, settings['saturation'])
        cap.set(cv2.CAP_PROP_GAIN, settings['gain'])
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)  # Manual white balance
        cap.set(cv2.CAP_PROP_WB_TEMPERATURE, settings['wb_temp'])
    
    def print_controls():
        print("\n📹 Camera Controls:")
        print("Q/A: Exposure    ", f"({settings['exposure']})")
        print("W/S: Brightness  ", f"({settings['brightness']})")  
        print("E/D: Contrast    ", f"({settings['contrast']})")
        print("R/F: Saturation  ", f"({settings['saturation']})")
        print("T/G: Gain        ", f"({settings['gain']})")
        print("Y/H: WB Temp     ", f"({settings['wb_temp']})")
        print("SPACE: Reset to defaults")
        print("ESC: Exit")
    
    apply_settings()
    print_controls()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Add settings overlay
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (300, 200), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        y_pos = 30
        cv2.putText(frame, f"Exposure: {settings['exposure']}", (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y_pos += 20
        cv2.putText(frame, f"Brightness: {settings['brightness']}", (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y_pos += 20
        cv2.putText(frame, f"Contrast: {settings['contrast']}", (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y_pos += 20
        cv2.putText(frame, f"Saturation: {settings['saturation']}", (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y_pos += 20
        cv2.putText(frame, f"Gain: {settings['gain']}", (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y_pos += 20
        cv2.putText(frame, f"WB Temp: {settings['wb_temp']}", (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        cv2.imshow('Camera Settings Test', frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        # Exposure control
        if key == ord('q') and settings['exposure'] < -1:
            settings['exposure'] += 1
            apply_settings()
            print_controls()
        elif key == ord('a') and settings['exposure'] > -11:
            settings['exposure'] -= 1
            apply_settings()
            print_controls()
            
        # Brightness control
        elif key == ord('w') and settings['brightness'] < 64:
            settings['brightness'] += 5
            apply_settings()
            print_controls()
        elif key == ord('s') and settings['brightness'] > -64:
            settings['brightness'] -= 5
            apply_settings()
            print_controls()
            
        # Contrast control
        elif key == ord('e') and settings['contrast'] < 100:
            settings['contrast'] += 5
            apply_settings()
            print_controls()
        elif key == ord('d') and settings['contrast'] > 0:
            settings['contrast'] -= 5
            apply_settings()
            print_controls()
            
        # Saturation control
        elif key == ord('r') and settings['saturation'] < 100:
            settings['saturation'] += 5
            apply_settings()
            print_controls()
        elif key == ord('f') and settings['saturation'] > 0:
            settings['saturation'] -= 5
            apply_settings()
            print_controls()
            
        # Gain control
        elif key == ord('t') and settings['gain'] < 100:
            settings['gain'] += 5
            apply_settings()
            print_controls()
        elif key == ord('g') and settings['gain'] > 0:
            settings['gain'] -= 5
            apply_settings()
            print_controls()
            
        # White balance control
        elif key == ord('y') and settings['wb_temp'] < 6500:
            settings['wb_temp'] += 200
            apply_settings()
            print_controls()
        elif key == ord('h') and settings['wb_temp'] > 2800:
            settings['wb_temp'] -= 200
            apply_settings()
            print_controls()
            
        # Reset to defaults
        elif key == ord(' '):
            settings = {
                'exposure': -6,
                'brightness': 0,
                'contrast': 32,
                'saturation': 50,
                'gain': 0,
                'wb_temp': 4000
            }
            apply_settings()
            print_controls()
            
        # Exit
        elif key == 27:  # ESC key
            break
    
    cap.release()
    cv2.destroyAllWindows()
    
    print("\n✅ Final Camera Settings:")
    for setting, value in settings.items():
        print(f"{setting}: {value}")

if __name__ == '__main__':
    test_camera_settings()
