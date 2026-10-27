# Hand Joystick Interface

Control Cartesian translation from hand movement using a USB webcam or an
OAK-D camera. MediaPipe detects the hand, this package normalizes its
reference-relative displacement, and `joystick_mapper` selects the robot axes.

```text
Camera → MediaPipe → /hand_landmarks → hand_joystick_node
       → /hand_joy → joystick_mapper → /hand_joystick_cartesian_command
       → cartesian_manager → /cartesian_command
```

## Setup and Build

Use Ubuntu 24.04, ROS 2 Jazzy, and Python 3.12. First follow the
[MediaPipe environment setup](../../tools/mediapipe_mocap/README.md#environment)
to install the camera driver and create `.venv_mediapipe` with its constrained
Python dependencies. These dependencies are not installed by `colcon build`.

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
source .venv_mediapipe/bin/activate
colcon build --packages-up-to hand_joystick_interfaces \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Prepare **each additional terminal** before using the commands below:

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
source .venv_mediapipe/bin/activate
source install/setup.bash
```

## Start a Camera

Run one camera pipeline at a time. Each launch starts the camera, MediaPipe,
the hand joystick node, and the mapper. It publishes Cartesian commands;
the robot stack is started separately.

### USB webcam

```bash
ros2 launch hand_joystick_interfaces usb_cam_hand_joystick_launch.py \
  visualize:=true show_control_overlay:=true
```

Choose the webcam in [explorer_hand_parameters.yaml](config/explorer_hand_parameters.yaml):

```yaml
usb_cam:
  ros__parameters:
    video_device: "/dev/video0"
```

A stable path under `/dev/v4l/by-id/` can also be used. To override the YAML for
one launch:

```bash
ros2 launch hand_joystick_interfaces usb_cam_hand_joystick_launch.py \
  video_device:=/dev/video2 visualize:=true
```

Camera settings such as `framerate`, `image_width`, `image_height`, and
`pixel_format` can also be passed as launch arguments. Priority is:
explicit launch argument → `usb_cam` section of the selected hand YAML →
`mediapipe_mocap/config/usb_cam.yaml`.

### OAK-D

```bash
ros2 launch hand_joystick_interfaces oak_hand_joystick_launch.py \
  visualize:=true show_control_overlay:=true
```

This pipeline uses metric 3D landmarks. See the [OAK-D guide](README_3D.md)
for depth and reference behavior.

## Configure Control

| File | Purpose |
| --- | --- |
| [explorer_hand_parameters.yaml](config/explorer_hand_parameters.yaml) | Webcam selection, hand normalization, reference and visualization settings. |
| [explorer_3d_hand_parameters.yaml](config/explorer_3d_hand_parameters.yaml) | OAK-D hand normalization, reference and visualization settings. |
| [hand_joystick_mapper.yaml](config/hand_joystick_mapper.yaml) | Robot axis indexes, signs, and command frame. |

The camera launches use these hand-node settings by default:

| Parameter | USB webcam | OAK-D | Meaning |
| --- | --- | --- | --- |
| `dead_zone` | `0.05` | `0.02` | Displacement ignored around the reference. |
| `saturation_zone` | `0.4` | `0.25` | Displacement that gives full output. |
| `normalization_mode` | `vector` | `axis` | Scale the vector magnitude or each axis independently. |
| `landmark_index` | `0` | `0` | Tracked landmark: `0` wrist, `8` index tip, `9` middle-finger MCP. |
| `lost_hand_cooldown_sec` | `0.2` | `0.2` | Delay without valid landmarks before publishing one zero command. |

Webcam distances use normalized image coordinates relative to the reference;
OAK-D distances are in meters. `Joy.axes` contains three normalized axes in
landmark XYZ order and no buttons. Commands resume with the existing reference
when tracking returns; loss of tracking does not recenter the hand.

The hand node applies dead zone, saturation, and rounding. The default mapper
uses translation `[axis 0, axis 2, -axis 1]` in `base_link`, with zero rotation.
Its deadzone is `0.0` because normalization already happened in the hand node.

Useful launch arguments:

| Argument | Use |
| --- | --- |
| `config_file:=/absolute/path/to/hand.yaml` | Select the hand and camera integration settings. |
| `mapper_config_file:=/absolute/path/to/mapper.yaml` | Select the robot axis mapping. |
| `start_mapper:=false` | Use the mapper already started by robot bringup. |
| `dead_zone`, `saturation_zone`, `normalization_mode`, `landmark_index` | Override hand control settings and their preview overlay together. |

Explicit control arguments override the selected hand YAML. The camera
launches copy the effective control settings to MediaPipe's display-only
overlay. `show_control_overlay` enables that preview; it does not modify the
landmarks or commands.

After changing code, launch files, or YAML, rebuild the package, source
`install/setup.bash` again, and restart the affected nodes.

## Connect to Explorer

Build the manager and its dependencies from a prepared terminal:

```bash
colcon build --packages-up-to cartesian_manager hand_joystick_interfaces \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

In terminal 1, start the robot stack with the hand mapper configuration:

```bash
ros2 launch cartesian_manager explorer.launch.py use_simulation:=true \
  joystick_config_file:="$(ros2 pkg prefix hand_joystick_interfaces)/share/hand_joystick_interfaces/config/hand_joystick_mapper.yaml"
```

For the real robot, use `use_simulation:=false` with the robot connected and
`can0` already configured.

In terminal 2, start the camera without a second mapper:

```bash
ros2 launch hand_joystick_interfaces usb_cam_hand_joystick_launch.py \
  start_mapper:=false visualize:=true show_control_overlay:=true
```

For OAK-D, replace the launch filename with `oak_hand_joystick_launch.py`.
Keep `start_mapper:=false`: Explorer bringup already starts the mapper.
The manager's enabled `hand_joystick` input consumes
`/hand_joystick_cartesian_command`.

## Inspect and Recenter

Run each inspection command in a separate prepared terminal, or stop it with
Ctrl+C before running the next one:

```bash
ros2 topic echo /hand_joy
```

```bash
ros2 topic echo /hand_joystick_cartesian_command
```

The optional RViz arrow uses `/cartesian_command_marker`, namespace
`cartesian_command`. Its parameters in the hand YAML are
`cartesian_command_marker_topic`, `cartesian_command_marker_frame_id`, and
`cartesian_command_marker_scale_x`, `cartesian_command_marker_scale_y`,
`cartesian_command_marker_scale_z`. The hand node reads the mapper output
selected by `cartesian_command_topic` to draw this arrow.

Recenter on the next valid hand frame by sending a falling then rising edge:

```bash
ros2 topic pub /reset_reference std_msgs/msg/Bool "{data: false}" --once
ros2 topic pub /reset_reference std_msgs/msg/Bool "{data: true}" --once
```

MediaPipe owns the reference. Reset requests are rate-limited by
`reset_reference_cooldown_sec` (default `0.25` s).
