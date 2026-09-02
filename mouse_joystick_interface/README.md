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
- `joy`: publish a timestamped `sensor_msgs/msg/Joy` on `joy_topic`. The buttons
  array is empty.

Joy mode uses the following mapping parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `joy_axis_count` | `2` | Size of the published `axes` array. |
| `joy_axes.x.index` | `0` | Axis receiving the browser `x` value. |
| `joy_axes.x.scale` | `1.0` | Multiplier applied to browser `x`. |
| `joy_axes.y.index` | `1` | Axis receiving the browser `y` value. |
| `joy_axes.y.scale` | `1.0` | Multiplier applied to browser `y`. |

Unused axes are zero. Mapped values are clamped to `[-1, 1]` after scaling.
Joy mode rejects non-positive array sizes, duplicate or out-of-range indexes,
and non-finite scales at startup. An unsupported `output_type` is always
rejected at startup.
