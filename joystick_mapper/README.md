# joystick_mapper

`joystick_mapper` converts `sensor_msgs/msg/Joy` messages into
`geometry_msgs/msg/TwistStamped` Cartesian velocity commands.

The package keeps joystick-specific details outside downstream command managers:
axis indexes, axis signs, deadzones, and mode buttons are configured here, while
other packages can consume a normal Cartesian twist command.

## Topics

By default, the node subscribes to:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/joy` | `sensor_msgs/msg/Joy` | Raw joystick axes and buttons. |

By default, the node publishes:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/joystick_cartesian_command` | `geometry_msgs/msg/TwistStamped` | Cartesian command generated from joystick axes. |
| `/mode_request` | `std_msgs/msg/String` | Structured mode requests from joystick buttons. |

The output frame is configured with `output_frame_id`, defaulting to `base_link`.

## How It Works

Each output twist component is mapped from one joystick axis:

| Output | Parameter |
| --- | --- |
| `twist.linear.x` | `axes.linear_x.index` / `axes.linear_x.scale` |
| `twist.linear.y` | `axes.linear_y.index` / `axes.linear_y.scale` |
| `twist.linear.z` | `axes.linear_z.index` / `axes.linear_z.scale` |
| `twist.angular.x` | `axes.angular_x.index` / `axes.angular_x.scale` |
| `twist.angular.y` | `axes.angular_y.index` / `axes.angular_y.scale` |
| `twist.angular.z` | `axes.angular_z.index` / `axes.angular_z.scale` |

`index` selects the entry in `sensor_msgs/msg/Joy.axes`.

`scale` multiplies the value after the deadzone is applied. Use negative values
to invert an axis.

`index: -1` disables that output component. Disabled or out-of-range axes publish
`0.0`.

The `deadzone` parameter is applied to every mapped axis with
`signal_processing::applyScaledDeadZone()`.

## State Buttons

Button parameters use joystick button indexes:

| Parameter | Published state | Topic |
| --- | --- | --- |
| `jaco_button_index` | `geometric/jaco` / `geometric/both` | `/mode_request` |
| `snake_button_index` | `geometric/snake` / `geometric/both` | `/mode_request` |
| `home_button_index` | `behaviour/joint_target/home` | `/mode_request` |

Each button has a mode parameter:

| Mode | Meaning |
| --- | --- |
| `toggle` | Press once to activate, press again to deactivate or return to the default state. |
| `hold` | Activate only while the button is held. Releasing the button returns to the default or cancel state. |
| `trigger` | Publish the request once on the rising edge. This is useful for one-shot command buttons. |

`momentary` and `pressed` are accepted aliases for `hold`.

Geometric buttons default to `toggle`: pressing `jaco_button_index` switches to
`geometric/jaco`, and pressing it again returns to `geometric/both`. If the mode
is `hold`, the mapper publishes `geometric/jaco` on press and `geometric/both`
on release.

The home button defaults to `trigger`, so it sends
`behaviour/joint_target/home` each time it is pressed. If `home_button_mode` is
`hold`, releasing the button publishes `behaviour/passthrough`. If it is
`toggle`, the first press publishes `behaviour/joint_target/home` and the second
press publishes `behaviour/passthrough`.

B1 and B2 are local mapper modes. They do not publish `/mode_request`; they only
switch which configured axis map is used for the outgoing
`TwistStamped`.

| Parameter | Effect |
| --- | --- |
| `local_mode_button_index` | Toggles between B1/default axes and B2 axes. |
| `local_mode_button_mode` | `toggle` switches B1/B2 on each press; `hold` uses B2 only while held. |

Use B1/B2 for joystick-local layouts such as translation-only, rotation-only,
or a 2D joystick that swaps between XY translation and Z/RZ control.

The top-level `axes` block is B1 and the default mode. `modes.b2.axes` is the
alternate map selected by the local mode button.

## Build

From the ROS 2 workspace root:

```bash
colcon build --packages-select joystick_mapper
source install/setup.bash
```

If you also need to rebuild its local dependencies:

```bash
colcon build --packages-up-to joystick_mapper
source install/setup.bash
```

## Run

Start your joystick driver first, for example `joy_node`, so that `/joy` is being
published.

Then launch the mapper:

```bash
ros2 launch joystick_mapper joystick_mapper.launch.py
```

To launch with another config file:

```bash
ros2 launch joystick_mapper joystick_mapper.launch.py \
  config_file:=/path/to/joystick_config.yaml
```

You can inspect the generated command with:

```bash
ros2 topic echo /joystick_cartesian_command
```

## Parameter Reference

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `joy_topic` | string | `/joy` | Raw joystick input topic. |
| `output_topic` | string | `/joystick_cartesian_command` | Cartesian command output topic. |
| `mode_request_topic` | string | `/mode_request` | Structured mode request topic. |
| `output_frame_id` | string | `base_link` | Frame id used in output commands. |
| `deadzone` | double | `0.2` | Axis deadzone, must be in `[0.0, 1.0)`. |
| `axes.<name>.index` | int | varies | Joystick axis index, or `-1` to disable. |
| `axes.<name>.scale` | double | `1.0` | Multiplier after deadzone processing. |
| `local_mode_button_index` | int | `-1` | Button that toggles between B1/default axes and B2 axes. |
| `local_mode_button_mode` | string | `toggle` | Activation mode for the local B2 axis-map button. |
| `modes.b2.axes.<name>.index` | int | varies | Optional B2 axis index override. |
| `modes.b2.axes.<name>.scale` | double | `1.0` | Optional B2 axis scale override. |
| `jaco_button_index` | int | `-1` | Button for `jaco` geometric mode. |
| `jaco_button_mode` | string | `toggle` | Activation mode for the Jaco geometric button. |
| `snake_button_index` | int | `-1` | Button for `snake` geometric mode. |
| `snake_button_mode` | string | `toggle` | Activation mode for the snake geometric button. |
| `home_button_index` | int | `-1` | Button for the `home` joint target behaviour. |
| `home_button_mode` | string | `trigger` | Activation mode for the home command button. |
