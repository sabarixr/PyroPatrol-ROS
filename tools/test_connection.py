#!/usr/bin/env python3
"""
Quick test script to verify WebSocket and camera stream are working
"""

import asyncio
import websockets
import json

async def test_websocket():
    uri = "ws://192.168.1.13:8765"
    
    print(f"Connecting to {uri}...")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("✓ WebSocket connected!")
            
            # Wait for welcome message
            welcome = await websocket.recv()
            print(f"Received: {welcome}")
            
            # Send a test command (STOP)
            test_cmd = {
                'topic': '/esp32_command',
                'data': 'STOP'
            }
            await websocket.send(json.dumps(test_cmd))
            print(f"✓ Sent STOP command")
            
            # Send a joystick command
            joystick_cmd = {
                'topic': '/cmd_vel',
                'twist': {
                    'linear': {'x': 0.0},
                    'angular': {'z': 0.0}
                }
            }
            await websocket.send(json.dumps(joystick_cmd))
            print(f"✓ Sent joystick command")
            
            # Listen for telemetry
            print("\nListening for telemetry (5 seconds)...")
            try:
                for i in range(5):
                    telemetry = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                    print(f"  Telemetry: {telemetry[:100]}...")
            except asyncio.TimeoutError:
                print("  (no telemetry received)")
            
            print("\n✅ WebSocket test complete!")
            
    except Exception as e:
        print(f"❌ WebSocket error: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure ws_bridge_node is running")
        print("2. Check robot IP is correct (192.168.1.13)")
        print("3. Verify port 8765 is open: sudo ufw allow 8765/tcp")

if __name__ == "__main__":
    print("="*50)
    print("Fire Rover Connection Test")
    print("="*50)
    print()
    
    # Test camera stream
    print("Camera Stream:")
    print("  URL: http://192.168.1.13:8080/video.mjpg")
    print("  Open this in a browser to verify camera works")
    print()
    
    # Test WebSocket
    asyncio.run(test_websocket())
