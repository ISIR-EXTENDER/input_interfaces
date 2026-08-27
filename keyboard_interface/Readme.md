# Keyboard Interface

ROS 2 input interface that publishes `std_msgs/Bool(data=true)` on configured topics when configured keys are pressed.

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

## Run the node

```bash
ros2 launch keyboard_interface keyboard_interface.launch.py
```