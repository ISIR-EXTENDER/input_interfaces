# Hand Joystick Interface

This package provides a ROS 2 interface for controlling robots using 2D or 3D hand tracking data from MediaPipe, converting hand landmark positions to joystick-like teleoperation commands.

## Overview

The `hand_joystick_interfaces` package processes hand landmark data from `mediapipe_mocap` nodes and converts it into control commands suitable for robot teleoperation.

## Features

- Subscribes to MediaPipe hand landmarks (PointCloud with 21 landmarks)
- Configurable landmark selection (wrist, finger tips, palm center, etc.)
- Converts 2D or 3D hand positions to teleoperation commands
- Configurable control zones and dead zones
- Reference position tracking for relative control
- Publishes normalized `sensor_msgs/msg/Joy` axes for `joystick_mapper`
- Combined launches publish Cartesian commands for `cartesian_manager`

## Nodes

### hand_joystick_node

Subscribes to hand landmark data from `mediapipe_mocap` and publishes normalized XYZ joystick axes.

**Subscribed Topics:**
- `/hand_landmarks` (sensor_msgs/msg/PointCloud) - Hand tracking data with 21 landmarks

**Published Topics:**
- `/hand_joy` (`sensor_msgs/msg/Joy`) - Three normalized landmark XYZ axes; no buttons
- `/cartesian_command_marker` (`visualization_msgs/msg/Marker`) - Mapped translation arrow, when enabled

The mapper started by the launch files publishes `/joystick_cartesian_command`
(`geometry_msgs/msg/TwistStamped`) for `cartesian_manager`.

**Parameters:**
- `dead_zone`: Dead zone radius to ignore small movements (default: 0.05)
- `saturation_zone`: Distance from reference point where output saturates at ±1 (default: 0.2)
- `normalization_mode`: `axis` for independent component ramps or `vector`
  for direction-preserving magnitude normalization (default: `vector`)
- `landmark_index`: Which landmark to track - 0=wrist, 8=index finger tip, 9=middle finger MCP (default: 8)
- `lost_hand_cooldown_sec`: Time without valid hand landmarks before the node publishes one zero command; valid input resumes using the existing reference (default: `0.2`)

These are the node's built-in defaults. The launch files load their YAML
configuration instead: USB/video use `saturation_zone: 0.4` and
`landmark_index: 0`; OAK uses `dead_zone: 0.02`, `saturation_zone: 0.25`,
`normalization_mode: axis`, and `landmark_index: 0`.

**Output Behavior:**
- Output is normalized between -1 and +1 in `Joy.axes`, in landmark XYZ order
- Within `dead_zone`: output = 0
- Between `dead_zone` and `saturation_zone`: output magnitude scales linearly from 0 to ±1
- Beyond `saturation_zone`: output saturates at ±1
- In `axis` mode, dead-zone and saturation are applied independently to XYZ
- In `vector` mode, a spherical dead-zone and saturation preserve direction

## MediaPipe Hand Landmarks

The hand_landmarks_node.py publishes 21 landmarks per hand:
- 0: Wrist
- 1-4: Thumb (CMC, MCP, IP, TIP)
- 5-8: Index finger (MCP, PIP, DIP, TIP)
- 9-12: Middle finger (MCP, PIP, DIP, TIP)
- 13-16: Ring finger (MCP, PIP, DIP, TIP)
- 17-20: Pinky (MCP, PIP, DIP, TIP)

Webcam landmarks are image-normalized; OAK landmarks are published as
reference-relative RGB-camera coordinates in meters. Dead-zone and saturation
mapping are applied by the joystick consumer.

## Prerequisites and Build

The supported environment is Ubuntu 24.04, ROS 2 Jazzy, and Python 3.12.
First follow the [MediaPipe environment setup](../../tools/mediapipe_mocap/README.md#environment)
to create `.venv_mediapipe`, install the constrained MediaPipe/DepthAI Python
dependencies, and install the ROS USB camera driver when using a webcam.
`colcon build` does not install those Python dependencies. Keep the documented
NumPy 1.x constraints for compatibility with ROS Jazzy extensions.

Build the hand pipeline and its workspace dependencies:

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
source .venv_mediapipe/bin/activate
colcon build --packages-up-to hand_joystick_interfaces \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Before running any launch, inspection, or test command below, prepare **each
terminal** with:

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
source .venv_mediapipe/bin/activate
source install/setup.bash
```

See [robot integration](#mapper-and-cartesian-manager-integration) for the
additional build required to launch `cartesian_manager` and the robot stack.

## Usage

### Live USB Camera Pipeline

Launch `usb_cam`, MediaPipe hand landmarks, and the hand joystick interface
together:

```bash
ros2 launch hand_joystick_interfaces usb_cam_hand_joystick_launch.py
```

Example with explicit camera settings and local landmarks visualization:

```bash
ros2 launch hand_joystick_interfaces usb_cam_hand_joystick_launch.py \
	video_device:=/dev/video0 \
	framerate:=30.0 \
	image_width:=1280 \
	image_height:=720 \
	pixel_format:=mjpeg2rgb \
	visualize:=true \
	show_control_overlay:=true
```

Camera defaults are loaded from
`mediapipe_mocap/config/usb_cam.yaml`. The optional `video_device`,
`framerate`, `image_width`, `image_height`, `pixel_format`, and `frame_id`
arguments override individual YAML values.
Overlay and joystick control arguments are optional overrides. When omitted,
the launch reads `dead_zone`, `saturation_zone`, `normalization_mode`, and
`landmark_index` once from the consumer section and translates them to the
display-only producer parameters. Explicit terminal arguments have highest
priority and are applied to both consumer and overlay.

### Live OAK-D S2 RGBD Pipeline

Launch OAK RGBD capture, 3D MediaPipe hand landmarks, and the hand joystick interface together:

```bash
ros2 launch hand_joystick_interfaces oak_hand_joystick_launch.py
```

Example with explicit OAK settings and local landmarks visualization:

```bash
ros2 launch hand_joystick_interfaces oak_hand_joystick_launch.py \
	fps:=30.0 \
	rgb_width:=640 \
	rgb_height:=400 \
	visualize:=true \
	show_control_overlay:=true
```

This launch file uses:
- `mediapipe_mocap/config/oak_hand_landmarks_node.yaml` for the base OAK RGBD node parameters
- `hand_joystick_interfaces/config/explorer_3d_hand_parameters.yaml` for the explorer 3D hand joystick defaults

Consumer control settings remain authoritative. The optional producer overlay
mirrors them for debugging but does not modify the landmark cloud.

For USB, video, and OAK combined launches, overlay metric precedence is:

```text
explicit terminal argument
> consumer section in the consumer/integration YAML
> producer YAML
> node built-in default
```

`show_control_overlay` remains producer display policy. Its precedence is the
explicit terminal argument, producer section in the integration YAML, producer
YAML, then node default.

Launch arguments use an internal `__use_yaml__` sentinel when omitted, so
launch-code defaults cannot silently mask YAML changes.

### Complete System Test with Video

Test the complete pipeline with video files using the integrated launch file:

```bash
ros2 launch hand_joystick_interfaces test_video_2d_hand_joystick_launch.py folder_path:=$HOME/test_videos_hands/ fps:=30
```

This launch file starts:
1. **video_publisher** - Publishes video frames from a folder
2. **hand_landmarks_node** - Detects hand landmarks using MediaPipe
3. **hand_joystick_node** - Normalizes hand displacement into XYZ `Joy` axes
4. **joystick_mapper_node** - Maps axes to Cartesian translation

Optional visualization is provided directly by `hand_landmarks_node`.
`show_control_overlay:=true` adds a preview of the same dead-zone, saturation,
and normalization settings passed to the joystick consumer.

### Complete System Launch

Alternatively, start components separately in three prepared terminals.
The combined video launch above is preferred because it mirrors the consumer
control settings to the producer overlay automatically. With separate nodes,
keep the explicit overlay values below aligned with the consumer YAML whenever
you change its configuration.

First, start the offline video publisher:

```bash
ros2 run offline_media_publisher video_publisher --ros-args \
	--params-file "$(ros2 pkg prefix offline_media_publisher)/share/offline_media_publisher/config/video_publisher.yaml" \
	-p folder_path:=$HOME/test_videos_hands/ \
	-p fps:=30
```

In another terminal, start the MediaPipe hand landmarks node:

```bash
ros2 run mediapipe_mocap hand_landmarks_node --ros-args \
  --params-file "$(ros2 pkg prefix mediapipe_mocap)/share/mediapipe_mocap/config/hand_landmarks_node.yaml" \
  --params-file "$(ros2 pkg prefix hand_joystick_interfaces)/share/hand_joystick_interfaces/config/explorer_hand_parameters.yaml" \
  -p overlay_dead_zone:=0.05 \
  -p overlay_saturation_zone:=0.4 \
  -p overlay_normalization_mode:=vector \
  -p tracked_landmark_index:=0
```

Then start the hand joystick interface:

```bash
ros2 launch hand_joystick_interfaces hand_joystick_launch.py
```

### Running

Running the executable directly publishes raw `/hand_joy` only. It requires a
landmark producer and a separately started mapper for Cartesian output:

```bash
ros2 run hand_joystick_interfaces hand_joystick_node
```

To start the hand consumer and mapper together (a landmark producer is still required):

```bash
ros2 launch hand_joystick_interfaces hand_joystick_launch.py
```

### Monitoring Output

Inspect raw normalized axes in one prepared terminal:

```bash
ros2 topic echo /hand_joy
```

Inspect the mapped command in another prepared terminal, or stop the previous
`echo` with Ctrl+C first:

```bash
ros2 topic echo /joystick_cartesian_command
```

The raw `Joy.axes` values are normalized XYZ displacement. The mapped output
uses `twist.linear`; its angular components are zero.

### Reset Reference From Terminal

Trigger a single reset:

```bash
ros2 topic pub /reset_reference std_msgs/msg/Bool "{data: true}" --once
```

For repeated resets, publish a falling edge then a rising edge:

```bash
ros2 topic pub /reset_reference std_msgs/msg/Bool "{data: false}" --once
ros2 topic pub /reset_reference std_msgs/msg/Bool "{data: true}" --once
```

One-liner equivalent:

```bash
ros2 topic pub /reset_reference std_msgs/msg/Bool "{data: false}" --once && ros2 topic pub /reset_reference std_msgs/msg/Bool "{data: true}" --once
```

Note: resets are rate-limited by `reset_reference_cooldown_sec` (default: `0.25` s).

## Configuration

Edit the configuration files in the `config/` directory to customize the behavior for different robots or control schemes.

- `explorer_hand_parameters.yaml`: webcam/2D hand joystick defaults
- `explorer_3d_hand_parameters.yaml`: OAK-D S2 RGBD 3D hand joystick defaults

## Dependencies

- rclcpp
- geometry_msgs
- sensor_msgs
- joystick_mapper


## Mapper and Cartesian Manager Integration

The supported data flow is:

```text
hand_landmarks (PointCloud)
  -> hand_joystick_node
  -> /hand_joy (Joy)
  -> joystick_mapper_node
  -> /joystick_cartesian_command (TwistStamped)
  -> cartesian_manager
  -> /cartesian_command
```

The hand node owns displacement dead zone, saturation, normalization, and
rounding. `Joy.axes` contains three axes in landmark XYZ order, stamped at
publication, with no buttons. The producer still owns reference capture/reset.
The watchdog publishes one zero Joy after tracking loss; the manager's input
timeout also handles stopped command streams. Shutdown zero publication is
best effort, so the downstream timeout remains necessary.

`config/hand_joystick_mapper.yaml` owns robot axis indexes, signs, and command
frame. Its Explorer translation map is `[axis 0, axis 2, -axis 1]`, with angular
axes and all buttons disabled. Mapper deadzone is `0.0` because the hand node
already applied the displacement dead zone. Physical joystick configurations
in `joystick_mapper` remain separate.

All four hand launches accept:

- `start_mapper:=true` (default): start the mapper alongside the hand pipeline.
- `mapper_config_file:=/absolute/path/to/mapper.yaml`: override the mapper config.

The consumer parameters `joy_output_topic` and `cartesian_command_topic`
default to `/hand_joy` and `/joystick_cartesian_command`. The latter selects the
mapped command used by the optional RViz marker. Marker settings are
`cartesian_command_marker_topic`, `cartesian_command_marker_frame_id`, and
`cartesian_command_marker_scale_x`, `_y`, `_z`. The default RViz topic is
`/cartesian_command_marker` and its marker namespace is `cartesian_command`.
If you change topics in custom YAML, update both the consumer and mapper
configurations to match.

For an existing standalone manager, start any hand pipeline with its defaults;
the manager's enabled joystick input should consume `/joystick_cartesian_command`.

For Explorer bringup, also build the manager and its workspace dependencies:

```bash
cd /home/emoullet/dev/extender_workspace
source /opt/ros/jazzy/setup.bash
source .venv_mediapipe/bin/activate
colcon build --packages-up-to cartesian_manager hand_joystick_interfaces \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Restart running hand/mapper/manager nodes after rebuilding so they use the new
installation; source `install/setup.bash` again in each terminal. No web page
refresh is required.

Explorer bringup already starts a mapper. In a prepared terminal, give it the
hand configuration:

```bash
ros2 launch cartesian_manager explorer.launch.py use_simulation:=true \
  joystick_config_file:="$(ros2 pkg prefix hand_joystick_interfaces)/share/hand_joystick_interfaces/config/hand_joystick_mapper.yaml"
```

Then start exactly one hand pipeline in another terminal:

```bash
ros2 launch hand_joystick_interfaces usb_cam_hand_joystick_launch.py start_mapper:=false
# Or: oak_hand_joystick_launch.py / test_video_2d_hand_joystick_launch.py
```

Use `use_simulation:=false` for the existing hardware bringup, with the robot
connected and its `can0` interface already configured. Its physical
`joy_node` may remain running, but the mapper now subscribes to `/hand_joy`.
Do not start another mapper publishing the same Cartesian joystick topic.

## Validation

### Automated checks without camera or robot

After the build above, run from a prepared terminal:

```bash
cd /home/emoullet/dev/extender_workspace
colcon test --packages-select hand_joystick_interfaces joystick_mapper \
  --event-handlers console_direct+
colcon test-result --verbose
```

These checks cover normalization, parameter descriptions, installed launch
configuration and overlay mirroring, plus synthetic landmark-to-Cartesian
commands, tracking loss/recovery, and markers. The ROS integration tests need
local DDS communication. They do not validate camera detection, robot
connectivity, or robot motion.

### Manual pipeline and robot checks

Launch exactly one USB, OAK, or video pipeline. Use `start_mapper:=false` when
the robot bringup already starts the mapper. In separate prepared terminals,
inspect the topics described above. Check that:

- `/hand_joy` contains three normalized XYZ axes and an empty button array.
- `/joystick_cartesian_command` has linear axes `[axis 0, axis 2, -axis 1]`,
  zero angular components, and frame `base_link` with the default hand mapper.
- The dead zone gives zero output; increasing displacement reaches saturation.
- Removing the hand causes one zero command after `lost_hand_cooldown_sec`
  (normally `0.2` s, plus the watchdog timer interval). When tracking resumes,
  commands use the existing reference; no automatic recenter is performed.
- An explicit `/reset_reference` request recenters the producer on the next
  valid hand frame. The overlay and mapped RViz marker agree with the commands.

With the robot stack running, also inspect:

```bash
ros2 control list_controllers
```

Check that `qontrol_explorer`, `gripper_controller`, and
`joint_state_broadcaster` are `active`, then inspect the manager output in
another prepared terminal:

```bash
ros2 topic echo /cartesian_command
```

The manager may shape or rate-limit this output, so it need not equal the raw
mapper command. Validate commanded directions and the stop on tracking loss
in simulation before checking the same behavior on the real robot.
