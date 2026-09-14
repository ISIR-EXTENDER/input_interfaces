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
| `/gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` | One-element gripper position command. |

Each local mode can declare its own `angular_output_frame_id` for the angular component. The mapper publishes the current mode's angular frame in `TwistStamped.header.frame_id`, while the linear motion remains aligned to the base frame convention for compatibility with the command interface. If a mode does not specify one, it falls back to `base_link` (Available options : `["base_link", "effector_frame", "hybrid_frame"`).

## How It Works

Each output twist component is mapped from one joystick axis, configured per mode
under `modes.<mode_name>.axes` (see [Local Modes](#local-modes) below):

| Output | Parameter |
| --- | --- |
| `twist.linear.x` | `modes.<mode_name>.axes.linear_x.index` / `.scale` |
| `twist.linear.y` | `modes.<mode_name>.axes.linear_y.index` / `.scale` |
| `twist.linear.z` | `modes.<mode_name>.axes.linear_z.index` / `.scale` |
| `twist.angular.x` | `modes.<mode_name>.axes.angular_x.index` / `.scale` |
| `twist.angular.y` | `modes.<mode_name>.axes.angular_y.index` / `.scale` |
| `twist.angular.z` | `modes.<mode_name>.axes.angular_z.index` / `.scale` |

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
| `gripper_button_index` | `gripper_close_position` / `gripper_open_position` | `gripper_command_topic` |

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

The gripper button defaults to `toggle`: each press alternates between publishing
`gripper_close_position` and `gripper_open_position` as a one-element
`std_msgs/msg/Float64MultiArray` on `gripper_command_topic`. If
`gripper_button_mode` is `hold`, pressing publishes the close position and
releasing publishes the open position.

## Local Modes

Local modes are freely-named axis maps listed under `modes.names` and cycled
with `local_mode_button_index`. They do not publish `/mode_request`; they only
switch which configured axis map is used for the outgoing
`TwistStamped`.

Each name in `modes.names` must have a corresponding `modes.<name>.axes` block
and may also define `modes.<name>.angular_output_frame_id` (see [How It Works](#how-it-works)).
The mapper starts in `modes.names[0]`; linear components keep the base-frame convention,
while the angular component can use the mode-specific angular frame.

| Parameter | Effect |
| --- | --- |
| `modes.names` | Ordered list of mode names to cycle through, e.g. `["b1", "b2", "precision"]`. |
| `modes.<mode_name>.angular_output_frame_id` | Angular frame used for the mode's rotational component and published in the twist header. Defaults to `base_link`.  Available options : `["base_link", "effector_frame", "hybrid_frame"`|
| `local_mode_button_index` | Cycles through `modes.names`, in order, wrapping back to the first entry. |
| `local_mode_button_mode` | `toggle`/`trigger` cycle through all modes on each press; `hold` only distinguishes the first two entries of `modes.names` (mode 0 while released, mode 1 while held). |

Use local modes for joystick-local layouts such as translation-only, rotation-only,
or a 2D joystick that swaps between XY translation and Z/RZ control.

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
| `deadzone` | double | `0.2` | Axis deadzone, must be in `[0.0, 1.0)`. |
| `modes.names` | string array | `["b1"]` | Ordered list of local mode names to cycle through. |
| `modes.<mode_name>.angular_output_frame_id` | string | `base_link` | Angular frame published in the output twist header while that mode is active; linear motion keeps the base-frame convention. |
| `modes.<mode_name>.axes.<name>.index` | int | `-1` | Joystick axis index for that mode, or `-1` to disable. |
| `modes.<mode_name>.axes.<name>.scale` | double | `1.0` | Multiplier after deadzone processing, for that mode. |
| `local_mode_button_index` | int | `-1` | Button that cycles through `modes.names`. |
| `local_mode_button_mode` | string | `toggle` | Activation mode for the local mode cycle button. |
| `jaco_button_index` | int | `-1` | Button for `jaco` geometric mode. |
| `jaco_button_mode` | string | `toggle` | Activation mode for the Jaco geometric button. |
| `snake_button_index` | int | `-1` | Button for `snake` geometric mode. |
| `snake_button_mode` | string | `toggle` | Activation mode for the snake geometric button. |
| `home_button_index` | int | `-1` | Button for the `home` joint target behaviour. |
| `home_button_mode` | string | `trigger` | Activation mode for the home command button. |
| `gripper_button_index` | int | `-1` | Button that toggles gripper close/open commands. |
| `gripper_button_mode` | string | `toggle` | Activation mode for the gripper command button. |
| `gripper_command_topic` | string | `/gripper_controller/commands` | Forward command controller topic for gripper positions. |
| `gripper_open_position` | double | `0.2` | Position sent when opening the gripper. |
| `gripper_close_position` | double | `1.1` | Position sent when closing the gripper. |
