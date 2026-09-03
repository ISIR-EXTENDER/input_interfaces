# mouse_joystick_interface

`mouse_joystick_interface` serves a browser-based two-axis joystick and publishes
its normalized coordinates as either the existing
`extender_msgs/msg/TeleopCommand` output or a `sensor_msgs/msg/Joy` message.

## Running

```bash
ros2 launch mouse_joystick_interface mouse_joystick_launch.py
```

The launch file loads `config/mouse_joystick_params.yaml`. The browser UI is
available at the configured HTTP host and port.

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

Joy mode uses the following mapping parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `joy_axis_count` | `2` | Size of the published `axes` array. |
| `joy_axes.x.index` | `0` | Axis receiving the browser `x` value. |
| `joy_axes.x.scale` | `1.0` | Multiplier applied to browser `x`. |
| `joy_axes.y.index` | `1` | Axis receiving the browser `y` value. |
| `joy_axes.y.scale` | `1.0` | Multiplier applied to browser `y`. |
| `joy_buttons.button_<0-11>.type` | `press` | Per-button `press` or `toggle` behavior. |

Unused axes are zero. Mapped values are clamped to `[-1, 1]` after scaling.
Joy mode rejects non-positive array sizes, duplicate or out-of-range indexes,
and non-finite scales at startup. Joy HTTP payloads must contain exactly twelve
integer button values, each `0` or `1`; omitting the field defaults every button
to zero for backward compatibility. Unsupported button types are rejected when
Joy mode starts. An unsupported `output_type` is always rejected at startup.
