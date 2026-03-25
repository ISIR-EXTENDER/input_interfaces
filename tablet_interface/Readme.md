# Tablet Interface

`tablet_interface` is the ROS 2 websocket backend used by `extender_ui`. It translates generic UI messages into ROS topics and forwards robot feedback back to the UI.

## Architecture

The backend now follows the same split as the UI:

- generic core: websocket transport, teleop publishing, runtime state
- bridges: isolated ROS-side behavior for each domain
- compatibility layer: pétanque support kept intact during the refactor

Main modules:

- `ros_teleop_publisher.py`: node orchestration
- `ws_server.py`, `ws_handlers.py`, `ws_models.py`: websocket transport and validation
- `runtime_state.py`: shared backend state sent to the UI
- `petanque_bridge.py`
- `measure_bridge.py`
- `sandbox_bridge.py`
- `actuator_bridge.py`
- `camera_bridge.py`

## Main websocket contract

- `teleop_cmd` -> publishes `extender_msgs/TeleopCommand`
- `ui_button` -> publishes `std_msgs/String`
- `ui_scalar` -> publishes `std_msgs/Float64`
- `camera_frame` -> publishes `sensor_msgs/CompressedImage`

Compatibility messages still supported for existing pétanque screens:

- `state_cmd`
- `petanque_cfg`
- `measure_request`
- `measure_refresh`

## Sandbox path

Minimal sandbox teleop flow:

1. UI sends `teleop_cmd`
2. `tablet_interface` republishes `/teleop_cmd`
3. `sandbox_controller` consumes `/teleop_cmd`

Sandbox feedback forwarded to UI state:

- `ee_pose`
- `tcp_speed_mps`
- `joint_positions`

## Camera direction

The backend now accepts `camera_frame`, republishes it as ROS `CompressedImage`, and makes browser-captured frames available to ROS nodes. This is the preferred path for future camera, RGB-D, perception, and visual-servoing features.

## Development

This package uses the workspace-level `uv` config.

```bash
cd /home/susana/workspace/extender_workspace
uv sync
cd src/input_interfaces/tablet_interface
make run-node
make test
```

## Contributing

- keep generic behavior in the core/websocket layer
- put app-specific ROS behavior in bridge modules
- preserve pétanque compatibility unless the team explicitly removes it
- prefer generic messages over new custom websocket message types when possible
