# Keyboard Interface

ROS 2 input interface that publishes key state as `std_msgs/Bool`: `data=true`
on a configured key press and `data=false` on its release. Auto-repeat presses
are suppressed, so each physical press produces one rising edge.

## Parameters

Configure mappings in `config/keyboard_param.yaml` using `key=/topic` entries:

```yaml
keyboard_interface_node:
  ros__parameters:
    key_topic_mappings:
      - "space=/keyboard/space"
      - "enter=/keyboard/enter"
      - "a=/keyboard/a"
```

Supported special keys depend on `pynput` names such as `space`, `enter`, and `esc`.

The remaining parameters control how events are delivered to ROS:

- `publish_qos_depth` sets the publisher queue depth (default: `10`).
- `publish_period_sec` sets how often queued keyboard events are published
  (default: `0.01`).

## Run the node

```bash
ros2 launch keyboard_interface keyboard_interface.launch.py
```

The node requires access to a graphical desktop session because `pynput` reads
keyboard events from the host display server.
