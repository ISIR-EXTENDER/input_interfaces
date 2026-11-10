# mouse_joystick_interface

`mouse_joystick_interface` serves a browser-based two-axis joystick and publishes
its normalized coordinates as either the existing
`extender_msgs/msg/TeleopCommand` output or a `sensor_msgs/msg/Joy` message.

## Installation

The commands below assume ROS 2 Jazzy is already installed on Ubuntu and this
repository is available in `~/extender_workspace/src`. Adjust the workspace path
to match your checkout. A web browser is required to use the joystick.

1. Install the build and dependency-management tools:

   ```bash
   sudo apt update
   sudo apt install python3-colcon-common-extensions python3-rosdep
   ```

   If rosdep has not been initialized on this machine, run `sudo rosdep init`
   once before continuing.

2. Load ROS 2 and install the dependencies of the interface and its local
   message package:

   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~/extender_workspace
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
   ```

   Keep `src/tools/extender_msgs` in the workspace: the server imports
   `TeleopCommand` even when configured to publish Joy messages. The HTTP server
   uses Python's standard library; no additional pip or npm installation is
   needed.

3. Build the interface together with its workspace dependencies, then load the
   resulting environment:

   ```bash
   colcon build --symlink-install --packages-up-to mouse_joystick_interface
   source install/setup.bash
   ```

   In each new terminal, source `/opt/ros/jazzy/setup.bash` and the workspace's
   `install/setup.bash` before running the package.

## Running

```bash
ros2 launch mouse_joystick_interface mouse_joystick_launch.py
```

The launch file loads `config/mouse_joystick_params.yaml` and starts
`joystick_mapper` with `config/mouse_joystick_mapper.yaml`. Set
`start_mapper:=false` if another launch already starts the mapper, or use
`mapper_config_file:=/path/to/config.yaml` to select a different mapper configuration.
When the launch starts the mapper, its button indexes and modes override the
corresponding interface button types at startup: `toggle` and `trigger` use a
momentary `press`, while `hold`, `momentary`, and `pressed` use an interface
`toggle`. Unassigned buttons retain their interface configuration. Conflicting
types for one index or indexes outside 0–11 stop the launch. Missing or unknown
mapper modes use the mapper's default for that function. These overrides do not
change either YAML file; restart the launch after changing the mapper config.
The browser labels assigned buttons with their mapper function names (joined by
` / ` when multiple functions share a button). Unassigned buttons keep their
numbers. Labels are also determined at startup; when `start_mapper:=false`,
all buttons display their numbers.
With `start_mapper:=false`, no overrides are applied and `mapper_config_file`
is not read.
The default mapper reads `/mouse_joy` and publishes Cartesian commands on
`/mouse_joystick_cartesian_command`. The browser UI is
available at the configured HTTP host and port: with the supplied configuration,
open <http://127.0.0.5:8765> if the browser does not open automatically. This
address is local to the machine running the node. The supplied configuration
selects Joy output on `/mouse_joy`; move the joystick and check the messages from
another terminal with the ROS 2 and workspace environments loaded:

```bash
ros2 topic echo /mouse_joy
```

Set `joystick_diameter_px` in `config/mouse_joystick_params.yaml` to a positive
integer to choose the joystick area's diameter in CSS pixels (default: 400).
If the interface exceeds the window, all controls scale down together without
scrolling and a warning is logged in the server terminal. Enlarging the window
restores the configured dimensions.
Restart the node and refresh the page after changing the value. The setting
applies to both Joy and twist output modes.

Set `interaction_mode` to `click` (default) or `mouseover` to choose how the
joystick responds when the page opens. The mode button can still switch between
them; refreshing the page restores the configured mode. Restart the node after
changing the YAML configuration.
In mouseover mode, the current joystick position is republished while the
pointer remains in the area, even when it is stationary. Leaving the area or
hiding the page sends a neutral command and stops that repeated publishing.

The page shows the latest local mode published by the mouse mapper, or `—`
until one is received. When this launch starts the mapper, the mode topic is
derived from `mapper_name` in `mapper_config_file`; an empty name uses the
legacy `/joystick_mapper/active_mode`. With `start_mapper:=false`, set
`active_mode_topic` in the interface YAML to the external mapper's topic.
The separate Interaction button controls Click/Mouseover as before.

## Output configuration

Set `output_type` to one of:

- `twist` (default): publish a `TeleopCommand` on `teleop_topic`. The browser's
  `x` and `y` values populate `twist.linear.x` and `twist.linear.y`, and the
  command mode is translation.
- `joy`: publish a timestamped `sensor_msgs/msg/Joy` on `joy_topic`. The web UI
  displays twelve configurable buttons mapped to Joy button indexes `0` through
  `11`.

The button controls are hidden in `twist` mode. In Joy mode, pressing or
releasing a button publishes the complete current axes and button state
immediately. Each `joy_buttons.button_<index>.type` parameter accepts:

- `press`: remain active while held, then release on pointer release,
  cancellation, focus loss, or page hiding.
- `toggle`: switch between active and inactive on each pointer press and remain
  latched through focus or visibility changes.

The node continues publishing at approximately 30 Hz while a press button is
held or a toggle button is latched on. Reloading the page initializes and
publishes an all-zero button state. The browser obtains the active mode and its
ordered `joy_button_types` list from the read-only `GET /config` endpoint.
Toggle controls use an amber pill design with a visible `TOGGLE` label, while
press controls retain the rectangular blue-active design.

Joy mode uses the following mapping parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `joy_axis_count` | `2` | Size of the published `axes` array. |
| `joy_axes.x.index` | `0` | Axis receiving the browser `x` value. |
| `joy_axes.x.scale` | `1.0` | Multiplier applied to browser `x`. |
| `joy_axes.y.index` | `1` | Axis receiving the browser `y` value. |
| `joy_axes.y.scale` | `1.0` | Multiplier applied to browser `y`. |
| `joy_buttons.button_<0-11>.type` | `press` | Per-button `press` or `toggle` behavior. |
| `joy_buttons.width_px` | `89` | Desired width of each button in CSS pixels. |
| `joy_buttons.height_px` | `44` | Minimum button height in CSS pixels. |

Button dimensions must be positive integers and are loaded at startup. The
whole interface scales down on small screens; long labels can make
buttons taller than the configured minimum. Restart the node and refresh the
page after changing these dimensions.

Unused axes are zero. Mapped values are clamped to `[-1, 1]` after scaling.
Joy mode rejects non-positive array sizes, duplicate or out-of-range indexes,
and non-finite scales at startup. Joy HTTP payloads must contain exactly twelve
integer button values, each `0` or `1`; omitting the field defaults every button
to zero for backward compatibility. Unsupported button types are rejected when
Joy mode starts. An unsupported `output_type` is always rejected at startup.
