# Tablet Interface

`tablet_interface` is the tablet backend bridge between `extender_ui` and ROS 2. It receives websocket commands from the UI, publishes `extender_msgs/TeleopCommand` on `/teleop_cmd`, and provides a small set of generic bridges for sandbox-style screens.

## Scope

This package currently supports three layers of behavior:

- core teleop: websocket `teleop_cmd` -> ROS `/teleop_cmd`
- compatibility adapters for the existing pétanque flow
- generic sandbox actions through `ui_button` and `ui_scalar`

## Development workflow

This package uses the workspace-level `uv` configuration from `extender_workspace`.

From the workspace root:

```bash
cd /home/susana/workspace/extender_workspace
uv sync
```

From the package directory:

```bash
cd src/input_interfaces/tablet_interface
make run-node
make run-ws-client
make test
```

Notes:

- there is no package-local `uv` project in `tablet_interface`
- `make test` uses `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` to avoid unrelated ROS pytest plugins interfering with these unit tests
- `make run-node` sources ROS and the workspace install before starting the node

## Sandbox teleop path

Minimal tablet-to-sandbox integration looks like this:

1. `extender_ui` sends `teleop_cmd`
2. `tablet_interface` maps and republishes it on `/teleop_cmd`
3. `sandbox_controller` subscribes to `/teleop_cmd`
4. the demo `update()` logic in `sandbox_controller` applies the received twist to robot behavior

Recommended controller setting:

```yaml
sandbox_controller:
  ros__parameters:
    input_topic_name: "/teleop_cmd"
```

## WebSocket messages

### Teleop

```json
{
  "type": "teleop_cmd",
  "seq": 42,
  "mode": 3,
  "linear": { "x": 0.2, "y": -0.1, "z": 0.0 },
  "angular": { "x": 0.0, "y": 0.0, "z": 0.1 }
}
```

### Generic sandbox actions

`ui_button` publishes `std_msgs/String` to the requested topic unless the topic is handled by a compatibility adapter.

```json
{
  "type": "ui_button",
  "topic": "/sandbox/action",
  "payload": "start"
}
```

`ui_scalar` publishes `std_msgs/Float64`.

```json
{
  "type": "ui_scalar",
  "topic": "/sandbox/max_velocity",
  "value": 1.25
}
```

### Compatibility messages

These are still supported so older screens keep working:

- `state_cmd`
- `petanque_cfg`
- `measure_request`
- `measure_refresh`

## Sandbox feedback forwarded to the UI

When available, the websocket `state` payload can include:

- `ee_pose` from `/sandbox_controller/ee_pose`
- `tcp_speed_mps` computed from `/sandbox_controller/velocity_command`
- `joint_positions` from `/sandbox_controller/joint_pose`

Default topic parameters:

- `sandbox_ee_pose_topic`
- `sandbox_velocity_command_topic`
- `sandbox_joint_pose_topic`

## Main ROS parameters

Teleop mapping:

- `teleop_cmd_topic`
- `publish_rate_hz`
- `linear_scale`
- `angular_scale`
- `linear_axes`
- `linear_signs`
- `angular_axes`
- `angular_signs`
- `swap_xy`

Websocket server:

- `bind_host`
- `bind_port`
- `ws_path`
- `state_publish_hz`

Robot/application bridges:

- `state_machine_topic`
- `gripper_topic`
- `hub_digital_output_topic`
- `petanque_param_service`
- `measure_request_image_topic`
- `measure_result_image_topic`
- `measure_result_vectors_topic`

Robot-specific presets live in:

- `config/tablet_interface_parameters_explorer.yaml`
- `config/tablet_interface_parameters_kinova.yaml`

## Internal structure

The package is organized so transport, validation, and ROS side effects are easier to reason about:

- `config.py`: ROS parameter declaration/loading
- `ros_teleop_publisher.py`: node orchestration and ROS bridges
- `generic_publishers.py`: cached generic ROS publishers
- `measure_codec.py`: image payload encoding/decoding helpers
- `ws_models.py`: websocket payload validation
- `ws_handlers.py`: websocket message routing
- `ws_server.py`: FastAPI/Uvicorn transport layer

## Verification

Unit tests:

```bash
make test
```

Websocket test client:

```bash
make run-ws-client
```

Expected teleop proof behavior:

- moving the tablet controls produces non-zero `/teleop_cmd`
- sandbox controller receives `/teleop_cmd`
- `/sandbox_controller/velocity_command` changes while teleop is active
