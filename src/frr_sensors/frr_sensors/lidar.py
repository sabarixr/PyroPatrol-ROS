#!/usr/bin/env python3
import serial
import struct
import time


FRAME_LEN = 22


def open_lidar(port="/dev/ttyUSB0", baud=115200):
    """Open serial connection to LiDAR."""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.1
        )
        print(f"✅ LiDAR connected on {port} @ {baud} baud")
        return ser
    except serial.SerialException as e:
        print(f"❌ Could not open serial port: {e}")
        exit(1)


def parse_frame(data):
    """Parse a 22-byte LiDAR frame."""
    if len(data) != FRAME_LEN:
        return None

    if data[0] != 0xAA or data[1] != 0x55:
        return None

    # Starting angle (hundredths of degrees)
    angle_start = struct.unpack_from("<H", data, 2)[0] / 100.0

    # Extract 9 distance values (mm → meters)
    distances = []
    for offset in range(4, 4 + 18, 2):
        dist_mm = struct.unpack_from("<H", data, offset)[0]
        distances.append(dist_mm / 1000.0)

    return angle_start, distances


def main():
    ser = open_lidar()

    while True:
        try:
            frame = ser.read(FRAME_LEN)
            result = parse_frame(frame)

            if result is None:
                continue

            angle_start, distances = result

            print(f"\n📡 LiDAR Frame")
            print(f"  Start Angle: {angle_start:.2f}°")
            print(f"  Distances (m): {['%.3f' % d for d in distances]}")

        except KeyboardInterrupt:
            print("\n🚫 Stopped by user")
            break
        except Exception as e:
            print(f"⚠️ Error: {e}")

    ser.close()


if __name__ == "__main__":
    main()
