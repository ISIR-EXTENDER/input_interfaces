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
- `topic_subscribe` -> creates generic ROS subscriptions and streams `topic_snapshot`

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

## ROS Topic Monitor

The backend can now forward arbitrary ROS topic snapshots to the UI. This is intended for
small debug and observability messages such as visual-servoing status, tag detections,
commands, errors, and scalar/vector state. Keep image and video streams on the dedicated
camera path.

Configured topics are declared with:

```yaml
topic_snapshot_hz: 10.0
topic_monitor_specs:
  - "/tag_detections|extender_msgs/msg/SharedControlGoalArray"
  - "/visual_servoing/velocity_command|geometry_msgs/msg/TwistStamped"
  - "/visual_servoing/error_TAGtoTAGd|geometry_msgs/msg/TwistStamped"
```

The UI can also request topics at runtime:

```json
{
  "type": "topic_subscribe",
  "topics": [
    {
      "topic": "/tag_detections",
      "message_type": "extender_msgs/msg/SharedControlGoalArray"
    }
  ]
}
```

When a monitored ROS message arrives, the websocket emits:

```json
{
  "type": "topic_snapshot",
  "topic": "/tag_detections",
  "message_type": "extender_msgs/msg/SharedControlGoalArray",
  "updated_at_ms": 123456,
  "revision": 1,
  "data": {},
  "error": null
}
```

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
