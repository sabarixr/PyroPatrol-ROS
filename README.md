# PyroPatrol Rover Control

PyroPatrol Rover Control is a ROS 2 project for a fire-response rover built around a Raspberry Pi 4 and an ESP32-S3. It brings together local navigation, live video, a Flutter operator app, and an MSCNN-based fire perception model that estimates fire direction and severity from a three-sector scan.

The system is built around local sensing and reactive behavior. It does not use SLAM or global mapping.

## GitHub Description

`ROS 2 + ESP32-S3 fire-response rover with local navigation, live video, Flutter control app, and MSCNN-based fire direction perception.`

## Quick Start

```bash
./start_rover.sh
```

Or launch directly with ROS 2:

```bash
ros2 launch frr_bringup esp32_rover_bringup.launch.py
```

## Documentation

- [System Overview](docs/overview.md)
- [ROS Components](docs/ros-components.md)
- [Hardware Guide](docs/hardware-guide.md)
- [AI Architecture And Findings](docs/ai-architecture.md)
- [Implementation Notes](docs/implementation-notes.md)
- [Architecture Notes](ARCHITECTURE.md)

## Repository Layout

```text
PyroPatrol-ROS/
├── apps/rider_app/                 # Flutter operator app
├── AI/                             # Training workspace and findings
├── docs/                           # Split project documentation
├── src/                            # ROS 2 packages
├── tools/                          # Diagnostics and helper scripts
├── esp_code.ino                    # ESP32-S3 firmware entry point
├── start_rover.sh                  # Quick launcher
├── ARCHITECTURE.md                 # Extended architecture notes
├── LICENSE
└── README.md
```

## Main Parts

- `frr_bringup`: integrated launch files
- `frr_control`: mission controller, ESP32 bridge, WebSocket bridge
- `frr_navigation`: motion and navigation behaviors
- `frr_sensors`: camera, telemetry parsing, fire perception
- `frr_video`: MJPEG streaming
- `apps/rider_app`: Flutter control application

## License

Apache License 2.0. See [LICENSE](LICENSE).
