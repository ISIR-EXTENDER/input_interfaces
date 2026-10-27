# OAK-D Hand Control

The OAK-D pipeline reconstructs hand landmarks in RGB-camera coordinates,
relative to a reference, in meters. The hand joystick node normalizes the
displacement and the mapper converts it to Cartesian translation.

Use the main README for [setup and build](README.md#setup-and-build),
[robot bringup](README.md#connect-to-explorer), and
[validation](README.md#validate).

## Start

In a terminal prepared as described in the main README:

```bash
ros2 launch hand_joystick_interfaces oak_hand_joystick_launch.py \
  visualize:=true show_control_overlay:=true
```

If Explorer bringup already starts the mapper, add `start_mapper:=false`.

## Configuration

Hand control settings are in
[explorer_3d_hand_parameters.yaml](config/explorer_3d_hand_parameters.yaml).
Camera and depth defaults are in
[oak_hand_landmarks_node.yaml](../../tools/mediapipe_mocap/config/oak_hand_landmarks_node.yaml).
The [MediaPipe guide](../../tools/mediapipe_mocap/README.md) describes those
camera settings and the required DepthAI environment.

| Hand parameter | Default |
| --- | --- |
| `dead_zone` | `0.02` m |
| `saturation_zone` | `0.25` m |
| `normalization_mode` | `axis` |
| `landmark_index` | `0` (wrist) |
| `lost_hand_cooldown_sec` | `0.2` s |

For example, override the control range for one launch:

```bash
ros2 launch hand_joystick_interfaces oak_hand_joystick_launch.py \
  dead_zone:=0.03 saturation_zone:=0.30 \
  visualize:=true show_control_overlay:=true
```

The launch mirrors these control settings to the preview overlay. Camera
capture can be overridden with `fps`, `rgb_width`, and `rgb_height`.
Robot axis mapping is defined in
[hand_joystick_mapper.yaml](config/hand_joystick_mapper.yaml).

## Tracking and Reference

- A valid hand with valid depth produces 21 landmarks in MediaPipe order.
- Filtering is applied before reference capture, publication, and display.
- With the default `auto_reference_on_first_detection: true`, the first valid
  metric hand initializes the reference.
- Invalid detection or depth stops cloud publication. The hand joystick
  watchdog then publishes one zero command after its timeout.
- When tracking returns, commands resume with the existing reference.

To choose a new reference, place the hand at the desired neutral position and
send an [explicit reset](README.md#inspect-and-recenter).

## Troubleshooting

| Symptom | Check |
| --- | --- |
| No `/hand_landmarks` | Camera connection, producer logs, depth alignment, and valid depth samples. |
| Landmarks arrive but no `/hand_joy` | Exactly 21 points, selected landmark, and hand-node logs. |
| Joy arrives but no `/hand_joystick_cartesian_command` | Mapper running and subscribed to `/hand_joy`. |
| Zero commands while the hand moves | Displacement relative to the reference, `dead_zone`, and `saturation_zone`. |
| Jittery commands | Producer filtering, a larger dead zone, or a more stable tracked landmark. |
