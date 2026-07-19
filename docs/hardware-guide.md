# Hardware Guide

## Compute

- Raspberry Pi 4 for ROS 2 and high-level logic
- ESP32-S3 N16R8 for low-level actuation and telemetry collection

## Sensors

- YDLidar X2 for local obstacle sensing and area identification (L-junction, T-junction, Dead-end, or avoid obstacles)
- MPU6050 IMU
- MQ2 smoke sensor
- MQ5 gas sensor
- flame sensors across scan sectors
- temperature sensor input
- ultrasonic front sensor as a lidar fallback
- CSI or USB camera for video and ArUco detection

## Actuation

- L298N (preferably, I advise using a less noisy one) motor driver for left and right DC motors
- water pump controlled through relay output
- scan or camera servos depending on build

## ESP32 Command Interface

Commands sent from the Pi to the ESP32 include:

- `DRIVE <left> <right>`
- `STOP`
- `PUMP_ON`
- `PUMP_OFF`
- `SCAN`
- `STATUS`

Telemetry is returned as JSON over serial.

## Practical Hardware Split

### Raspberry Pi Side

- ROS 2 nodes
- mission controller
- AI inference
- video streaming
- app connectivity

### ESP32 Side

- motor PWM and direction control
- pump switching
- telemetry collection
- scan sample publication

## Wiring Notes

This repository assumes the ESP32 is the hardware-side controller and the Pi is the decision-making side. Exact wiring can vary slightly by build revision, but the logical ownership should stay the same:

- Pi decides
- ESP32 actuates

For broader architecture context, also see [ARCHITECTURE.md](../ARCHITECTURE.md).
