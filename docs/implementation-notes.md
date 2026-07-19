# Implementation Notes

## What Was Implemented

The AI-assisted fire-perception pipeline follows this flow:

1. Build a dataset from ESP32 telemetry grouped by scan and sector.
2. Synthesize severity from `fire_present` and `distance_to_fire_cm`.
3. Train a dual-head MSCNN for direction and severity.
4. Package the final inference assets into `frr_sensors`.
5. Run live inference in `fire_perception_node`.
6. Feed direction and severity into `mission_controller`.
7. Let mission logic steer and optionally trigger the pump.

## What Changed In ROS

### `fire_perception_node`

- collects `LEFT`, `FRONT`, and `RIGHT` sector samples
- pads or clips to `15` timesteps
- scales the features
- runs PyTorch inference
- publishes a unified prediction message

### `mission_controller`

- subscribes to `/mission/fire_perception`
- treats the AI prediction as the main fire-direction signal during autonomous fire mode
- uses severity as a gating signal for pump activation when the fire is centered

### `frr_sensors` Packaging

- includes the runtime MSCNN model files under the package share directory
- loads them through the installed package path rather than the training notebook path

## Why This Packaging Matters

The runtime node should not depend on the training workspace being present on the robot.

That is why the final inference path was moved into:

- `src/frr_sensors/models/`

This keeps deployment simpler and makes the ROS package self-contained for inference.

## Operator Path

The Flutter app in `apps/rider_app/` connects through the WebSocket bridge and gives the operator:

- connection setup
- command/control input
- live video view
- mode switching

## Related Files

- `src/frr_sensors/frr_sensors/fire_perception_node.py`
- `src/frr_sensors/frr_sensors/fire_model.py`
- `src/frr_control/frr_control/mission_controller.py`
- `src/frr_bringup/launch/esp32_rover_bringup.launch.py`
