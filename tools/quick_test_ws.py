#!/usr/bin/env python3
"""Quick WebSocket test - connects and sends one command"""
import asyncio
import websockets
import json
import traceback

async def test():
    ws = None
    try:
        print("Connecting to ws://192.168.1.13:8765...")
        ws = await websockets.connect("ws://192.168.1.13:8765")
        print("✓ Connected!")
        
        # Receive welcome
        msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
        print(f"Received: {msg}")
        
        # Send STOP command
        cmd = json.dumps({'topic': '/esp32_command', 'data': 'STOP'})
        await ws.send(cmd)
        print(f"Sent STOP command")
        
        # Wait for response
        await asyncio.sleep(1)
        print("✓ Test passed!")
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        traceback.print_exc()
    finally:
        if ws:
            await ws.close()

if __name__ == '__main__':
    asyncio.run(test())
