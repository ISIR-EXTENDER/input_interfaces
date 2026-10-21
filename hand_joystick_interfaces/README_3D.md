# OAK 3D Hand Joystick

## Overview

The supported 3D hand-control path uses an OAK-D camera. The
`oak_hand_landmarks_node` producer publishes reference-relative metric
landmarks, and `hand_joystick_node` converts one selected landmark into
normalized XYZ joystick axes.

```text
OAK-D RGBD camera
  -> mediapipe_mocap/oak_hand_landmarks_node
  -> /hand_landmarks (sensor_msgs/PointCloud)
  -> hand_joystick_interfaces/hand_joystick_node
  -> /hand_joy (sensor_msgs/Joy)
  -> joystick_mapper/joystick_mapper_node
  -> /joystick_cartesian_command (geometry_msgs/TwistStamped)
  -> cartesian_manager
```

The removed depth-inference video node was never part of the installed
`mediapipe_mocap` package. Offline RGB video remains available for testing the
2D pipeline, but it does not produce metric 3D landmarks.

## Quick Start

First follow the [MediaPipe environment setup](../../tools/mediapipe_mocap/README.md#environment)
for `.venv_mediapipe` and its constrained Python dependencies, including
DepthAI. Then build and source the required packages:

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
source .venv_mediapipe/bin/activate
colcon build --packages-up-to hand_joystick_interfaces \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Prepare each additional terminal with the same workspace directory,
ROS setup, virtual environment, and `install/setup.bash`.
Launch the OAK producer, joystick consumer, and mapper:

```bash
ros2 launch hand_joystick_interfaces oak_hand_joystick_launch.py
```

Useful overrides:

```bash
ros2 launch hand_joystick_interfaces oak_hand_joystick_launch.py \
  visualize:=true \
  show_control_overlay:=true \
  dead_zone:=0.05 \
  saturation_zone:=0.4 \
  normalization_mode:=vector \
  landmark_index:=0
```

Omitted arguments are loaded from
`config/explorer_3d_hand_parameters.yaml` and the producer's
`config/oak_hand_landmarks_node.yaml`.

## Data Contract

`/hand_landmarks` contains exactly 21 MediaPipe-ordered points when a valid
hand and valid depth reconstruction are available:

- Coordinates are reference-relative RGB-camera coordinates in meters.
- The selected reference is captured from the first valid metric hand.
- Filtering occurs before reference, publication, and visualization.
- No cloud is published while the hand or its metric reconstruction is
  invalid.

The joystick watchdog owns the zero-command response when clouds stop
arriving. When valid clouds resume, commands use the existing reference;
tracking loss does not request a new reference.

## Reference Reset

The first valid hand initializes the reference automatically. Recenter it with
a rising edge on `/reset_reference`:

```bash
ros2 topic pub /reset_reference std_msgs/msg/Bool "{data: false}" --once
ros2 topic pub /reset_reference std_msgs/msg/Bool "{data: true}" --once
```

Reset requests are rate-limited by `reset_reference_cooldown_sec`.

## Control Configuration

The authoritative consumer parameters live under
`hand_joystick_interface.ros__parameters` in
`config/explorer_3d_hand_parameters.yaml`:

- `dead_zone`: inactive distance around the reference, in meters.
- `saturation_zone`: distance at which the command reaches full scale.
- `normalization_mode`: `axis` for independent component scaling or `vector`
  for direction-preserving scaling.
- `landmark_index`: MediaPipe landmark used as the joystick input.
- `joy_output_topic`: raw normalized XYZ joystick topic.
- `cartesian_command_topic`: mapped command used by the optional RViz marker.
- `lost_hand_cooldown_sec`: watchdog delay before publishing a zero command.

When the producer control overlay is enabled, the combined launch mirrors the
consumer dead zone, saturation zone, normalization mode, and landmark index to
the producer's display-only overlay parameters. These settings never alter
the published point cloud.

## Topics

Subscribed by `hand_joystick_node`:

- `/hand_landmarks` (`sensor_msgs/PointCloud`)

Published by `hand_joystick_node`:

- `/hand_joy` (`sensor_msgs/Joy`)
- `/cartesian_command_marker` (`visualization_msgs/Marker`), when enabled

## Troubleshooting

No `/hand_landmarks` output:

- Confirm `oak_hand_landmarks_node` is running.
- Confirm the OAK camera is available and depth is aligned to RGB.
- Check the producer log for invalid or missing depth samples.

No `/joystick_cartesian_command` movement:

- Confirm `/hand_landmarks` contains 21 points.
- Check `landmark_index`, `/hand_joy`, and the mapper axis indexes/signs.
- Reduce `dead_zone` or `saturation_zone` if the configured metric distances
  are too large.

Jittery control:

- Keep One Euro filtering enabled in the producer.
- Increase `dead_zone`.
- Select a more stable landmark such as the wrist or an MCP joint.

## See Also

- [Hand joystick README](README.md)
- [MediaPipe Mocap README](../../tools/mediapipe_mocap/README.md)


## Mapper Configuration

The OAK launch starts the mapper by default using
`config/hand_joystick_mapper.yaml`. Normalized landmark XYZ axes are converted
to Explorer translation `[axis 0, axis 2, -axis 1]` in `base_link`, with zero
angular velocity. Displacement normalization remains in the hand consumer;
mapper deadzone is zero to avoid applying it twice.

Use `mapper_config_file:=/absolute/path/to/mapper.yaml` to change robot mapping.
Use `start_mapper:=false` when Explorer bringup already owns the mapper, and
pass the hand mapper YAML as its `joystick_config_file`.
See the main README for complete manager integration commands.

## Validation and Robot Bringup

Follow the main README's [automated and manual validation](README.md#validation)
for camera-free regression tests and the expected command/stop behavior. Check
OAK-specific reconstruction separately: valid depth should produce 21 metric
landmarks; invalid depth should stop cloud publication and trigger the consumer
watchdog.

The [robot integration procedure](README.md#mapper-and-cartesian-manager-integration)
builds `cartesian_manager`, starts Explorer, and avoids a duplicate mapper.
Substitute `oak_hand_joystick_launch.py start_mapper:=false` for the USB launch
in that procedure.

Documentation-only edits require no build, node restart, or web page refresh.
After code or launch/configuration changes, rebuild the affected packages,
source the installation in each terminal, and restart the affected nodes.
