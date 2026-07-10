# Tablet Interface

`tablet_interface` is the ROS 2 websocket backend used by
[`extender_ui`](https://github.com/ISIR-EXTENDER/extender_ui). It receives
generic UI messages from the tablet, validates them, republishes them to ROS 2,
and streams robot state, topic snapshots, and backend events back to the UI.

The current production-style frontend app for new work is **Sandbox V0.0**.
Petanque support is still preserved for legacy compatibility and examples, but
new controller/UI development should use Sandbox V0.0 with `sandbox_controller`.

<p align="center">
  <img alt="ROS 2" src="https://img.shields.io/badge/ROS%202-Humble-22314e?style=for-the-badge" />
  <img alt="Python" src="https://img.shields.io/badge/Python-3.10-3776ab?style=for-the-badge&logo=python&logoColor=white" />
  <img alt="WebSocket" src="https://img.shields.io/badge/WebSocket-backend-4b5563?style=for-the-badge" />
</p>

<p align="center">
  <a href="#current-state">Current State</a> ·
  <a href="#sandbox-v00">Sandbox V0.0</a> ·
  <a href="#architecture">Architecture</a> ·
  <a href="#websocket-contract">WebSocket Contract</a> ·
  <a href="#ros-contract">ROS Contract</a> ·
  <a href="#configuration">Configuration</a> ·
  <a href="#development">Development</a> ·
  <a href="#bloom-migration">Bloom Migration</a>
</p>

## Current State

- WebSocket server for `extender_ui` at `/ws/control`.
- Teleoperation bridge from frontend joystick/slider/mode state to
  `extender_msgs/msg/TeleopCommand`.
- Generic typed ROS publishers for frontend widgets.
- Topic monitor bridge for small diagnostic ROS messages.
- Browser-captured camera frame bridge to ROS compressed image topics.
- Sandbox feedback bridge for end-effector pose, velocity command, and joint
  pose state.
- Legacy Petanque, measure, gripper, and actuator bridges kept for
  compatibility.
- Runtime state and backend events streamed back to the UI.

## Sandbox V0.0

Sandbox V0.0 is the recommended app/backend path for new Extender development.
The intended flow is:

```text
extender_ui Sandbox V0.0 screen
  -> websocket message
  -> tablet_interface
  -> ROS topic
  -> sandbox_controller or perception node
  -> ROS feedback
  -> tablet_interface
  -> UI state, topic monitor, or event
```

Current Sandbox V0.0 screens on the frontend:

| Frontend screen | Backend support |
| --- | --- |
| `sandbox_control` | `teleop_cmd`, sandbox feedback state, typed widget publishing. |
| `sandbox_teleop_config` | Teleop mapping and reusable widget configuration. |
| `control_panel` | Teleop, webcam/camera frame flow, gripper, visual-servoing controls, and compact telemetry. |
| `snake_control` | `teleop_cmd` plus typed boolean publication to `/snake_control/enable`. |
| `visual_servoing` | Typed ON/OFF and save-tag commands. |
| `visual_servoing_monitor` | Topic snapshots for AprilTag and servo telemetry. |

Use [`sandbox_controller`](https://github.com/ISIR-EXTENDER/sandbox_controller)
for new robot control experiments. Petanque bridges remain available, but they
should not be the default path for new features.

## Architecture

`tablet_interface` is intentionally split into a generic websocket/ROS core and
domain bridges.

```text
browser / tablet UI
  -> ws_server.py
  -> ws_handlers.py
  -> bridge or publisher module
  -> ROS 2 publisher/subscriber/service
  -> runtime_state.py / websocket response
  -> browser / tablet UI
```

Main modules:

| Module | Responsibility |
| --- | --- |
| `main.py` | Package entry point. |
| `ros_teleop_publisher.py` | ROS node orchestration, timers, and bridge wiring. |
| `ws_server.py` | WebSocket server lifecycle. |
| `ws_handlers.py` | Incoming message dispatch and outgoing UI responses. |
| `ws_models.py` | Pydantic websocket message validation models. |
| `runtime_state.py` | Shared backend state streamed to the UI. |
| `safety_gate.py` | Teleop watchdog and safety filtering. |
| `teleop_mapping.py` | UI vector mapping to ROS `TeleopCommand`. |
| `generic_publishers.py` | Generic string/scalar/bool ROS publishers. |
| `typed_message_publishers.py` | Typed ROS message publishing for configurable UI widgets. |
| `topic_monitor_bridge.py` | Generic ROS topic subscriptions and `topic_snapshot` output. |
| `sandbox_bridge.py` | Sandbox controller feedback subscriptions. |
| `camera_bridge.py` | Browser `camera_frame` to ROS compressed image bridge. |
| `actuator_bridge.py` | Gripper and actuator-related output bridges. |
| `petanque_bridge.py` | Legacy Petanque state-machine compatibility. |
| `measure_bridge.py` | Legacy Petanque measure compatibility. |

## WebSocket Contract

Default endpoint:

```text
ws://localhost:8765/ws/control
```

Incoming messages from the UI:

| Message type | Purpose |
| --- | --- |
| `teleop_cmd` | Main normalized teleoperation command. |
| `ui_button` | Generic string command from button widgets. |
| `ui_scalar` | Generic numeric command from slider/gain widgets. |
| `ui_bool` | Generic boolean command. |
| `ui_typed` | Typed ROS message publication from configurable widgets. |
| `camera_frame` | Browser-captured image frame as a data URL. |
| `topic_subscribe` | Runtime request for ROS topic snapshots. |
| `gripper_cmd` | Gripper open/close command. |
| `state_cmd` | Legacy Petanque state-machine command. |
| `petanque_cfg` | Legacy Petanque parameter update command. |
| `measure_request` | Legacy measure request with captured image. |
| `measure_refresh` | Legacy measure result refresh request. |

Outgoing messages to the UI:

| Message type | Purpose |
| --- | --- |
| `state` | Backend connectivity, watchdog, mode, gripper, and sandbox feedback. |
| `event` | Backend info/warning/error events, including subscription failures. |
| `topic_snapshot` | Latest compact ROS message snapshot for monitored topics. |
| `measure_result` | Legacy measure result payload. |

### Teleoperation Message

```json
{
  "type": "teleop_cmd",
  "seq": 42,
  "mode": 0,
  "linear": { "x": 0.2, "y": -0.1, "z": 0.0 },
  "angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
}
```

The backend validates finite values, applies mapping/scaling, applies safety
rules, and republishes the command as ROS.

### Typed UI Message

```json
{
  "type": "ui_typed",
  "topic": "/snake_control/enable",
  "message_type": "std_msgs/msg/Bool",
  "payload_text": "{data: true}",
  "widget_id": "snake-enable"
}
```

This powers widgets such as `ROS Message Toggle` and `Momentary ROS Message`.
Use it when a frontend control should publish a specific ROS message type and
payload.

### Topic Subscribe Message

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

When a monitored ROS message arrives, the backend emits:

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

Topic monitors are for small diagnostic messages. The backend rejects image and
video topic types such as `sensor_msgs/msg/Image` and
`sensor_msgs/msg/CompressedImage` so large video streams do not pass through the
topic snapshot path.

## ROS Contract

### Teleoperation

| ROS topic | Message | Direction |
| --- | --- | --- |
| `/teleop_cmd` | `extender_msgs/msg/TeleopCommand` | backend -> controller |

The frontend widget `topic` fields such as `/cmd/joystick` or `/cmd/mode` are
UI configuration metadata. They do not change the backend teleop output topic.
Use the `teleop_cmd_topic` parameter to change the ROS output topic.

### Sandbox Feedback

| Parameter | Default topic | Message | Direction |
| --- | --- | --- | --- |
| `sandbox_ee_pose_topic` | `/sandbox_controller/ee_pose` | `geometry_msgs/msg/PoseStamped` | controller -> backend -> UI |
| `sandbox_velocity_command_topic` | `/sandbox_controller/velocity_command` | `geometry_msgs/msg/TwistStamped` | controller -> backend -> UI |
| `sandbox_joint_pose_topic` | `/sandbox_controller/joint_pose` | joint state style payload | controller -> backend -> UI |

### Snake Control

| ROS topic | Message | Direction |
| --- | --- | --- |
| `/snake_control/enable` | `std_msgs/msg/Bool` | UI -> backend -> controller |

Momentary button contract:

```text
press   -> /snake_control/enable std_msgs/msg/Bool {data: true}
release -> /snake_control/enable std_msgs/msg/Bool {data: false}
```

The same joystick velocity is sent in B1 and B2. The frontend changes only the
teleop mode (`B1 -> mode: 0`, `B2 -> mode: 3`); the backend/controller
interprets the mode.

### Visual Servoing

| Topic | Message | Direction |
| --- | --- | --- |
| `/ui/visual_servoing/on` | `std_msgs/msg/Bool` | UI -> ROS |
| `/ui/visual_servoing/save` | `std_msgs/msg/String` | UI -> ROS |
| `/tag_detections` | `extender_msgs/msg/SharedControlGoalArray` | AprilTag detector -> UI / visual servoing |
| `/visual_servoing/velocity_command` | `geometry_msgs/msg/TwistStamped` | visual servoing -> UI |
| `/visual_servoing/error_TAGtoTAGd` | `geometry_msgs/msg/TwistStamped` | visual servoing -> UI |

The UI should display video through stream widgets and monitor only the compact
ROS messages above through `topic_subscribe`.

### Camera Frames

The backend accepts browser-captured `camera_frame` messages and republishes the
image as `sensor_msgs/msg/CompressedImage`.

Legacy/default Petanque parameters:

| Parameter | Default topic |
| --- | --- |
| `measure_request_image_topic` | `/petanque/measure/request_image/compressed` |
| `measure_result_image_topic` | `/petanque/measure/result_image/compressed` |
| `measure_result_vectors_topic` | `/petanque/measure/result_vectors` |

For ROS camera pipelines such as AprilTag detection, prefer direct ROS camera
topics for image transport:

```text
/image_raw
/camera_info
```

## Configuration

Default parameters are declared in `tablet_interface/config.py` and can be
overridden with YAML files in `config/`.

Included profiles:

| File | Purpose |
| --- | --- |
| `config/tablet_interface_parameters.yaml` | Default profile. |
| `config/tablet_interface_parameters_explorer.yaml` | Explorer/sandbox-oriented profile used by `make run-node`. |
| `config/tablet_interface_parameters_kinova.yaml` | Kinova-oriented profile. |

Important parameters:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `teleop_cmd_topic` | `/teleop_cmd` | ROS output topic for teleop commands. |
| `publish_rate_hz` | `30.0` in code, profile-specific in YAML | Teleop publish timer rate. |
| `linear_scale` | `0.2` in code, profile-specific in YAML | Linear command scaling. |
| `angular_scale` | `0.5` in code, profile-specific in YAML | Angular command scaling. |
| `default_mode` | `0` | Mode used when client mode is not accepted. |
| `accept_mode_from_client` | `true` | Whether UI can select teleop mode. |
| `state_publish_hz` | `5.0` in code, profile-specific in YAML | Backend state broadcast rate. |
| `bind_host` | `0.0.0.0` | WebSocket bind host. |
| `bind_port` | `8765` | WebSocket bind port. |
| `ws_path` | `/ws/control` | WebSocket path. |
| `topic_snapshot_hz` | `10.0` | Topic monitor snapshot rate. |
| `topic_monitor_specs` | visual-servoing topics | Startup topic monitor subscriptions. |
| `param_call_timeout_sec` | `1.5` | ROS parameter service timeout. |

Example launch with a specific profile:

```bash
ros2 run tablet_interface tablet_interface_node --ros-args \
  --params-file src/input_interfaces/tablet_interface/config/tablet_interface_parameters_explorer.yaml
```

## Development

### Required Extender Packages

For normal development and sandbox testing, build these packages in the Extender
ROS workspace:

| Package / repo | Required for |
| --- | --- |
| [`robot_interfaces/extender_msgs`](https://github.com/ISIR-EXTENDER/robot_interfaces) | `TeleopCommand` and shared Extender messages. |
| [`sandbox_controller`](https://github.com/ISIR-EXTENDER/sandbox_controller) | Sandbox teleop and feedback loop. |
| [`tools/apriltag_detector`](https://github.com/ISIR-EXTENDER/tools/tree/main/apriltag_detector) | AprilTag detections for visual servoing. |
| [`visual_servoing`](https://github.com/ISIR-EXTENDER/visual_servoing) | Visual-servoing controller and telemetry topics. |
| [`extender_ui`](https://github.com/ISIR-EXTENDER/extender_ui) | Frontend runtime and screen builder. |

### Install Workspace Dependencies

From the Extender ROS workspace:

```bash
uv sync
colcon build --symlink-install --packages-select tablet_interface
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Run The Backend

From this package:

```bash
make run-node
```

Equivalent explicit command:

```bash
source /opt/ros/humble/setup.bash
source ../../../install/setup.bash
uv run python -m tablet_interface.main --ros-args \
  --params-file config/tablet_interface_parameters_explorer.yaml
```

### Test The WebSocket

With the backend running:

```bash
make run-ws-client
```

The frontend expects:

```text
ws://127.0.0.1:8765/ws/control
```

### Run Tests

```bash
make test
```

The Makefile disables external pytest plugin autoloading so a sourced ROS
environment does not leak unrelated plugins into the package tests.

## Working With The Frontend

Recommended local loop:

1. Start `tablet_interface` with `make run-node`.
2. Start `extender_ui` with `npm run dev`.
3. Open Sandbox V0.0 in the frontend.
4. Use `control_panel`, `snake_control`, `visual_servoing`, or
   `visual_servoing_monitor` depending on the workflow.
5. Check backend `event` messages and UI topic monitors before debugging lower
   layers.

Backend state messages include:

- `connected`
- `cmd_age_ms`
- `watchdog_timeout_ms`
- `last_seq`
- `publishing_rate_hz`
- `current_mode`
- `gripper_state`
- `ee_pose`
- `tcp_speed_mps`
- `joint_positions`

## Compatibility

This backend README is aligned with the current `extender_ui` README update and
the active Extender workspace split.

| Component | Repository | Commit checked | Notes |
| --- | --- | --- | --- |
| Frontend | [`extender_ui`](https://github.com/ISIR-EXTENDER/extender_ui) | `9c3d0db docs: update project readme (#29)` | Documents Sandbox V0.0 and backend contracts. |
| Backend/input interfaces | [`input_interfaces`](https://github.com/ISIR-EXTENDER/input_interfaces) | Current branch | This README update. |
| Controllers | [`controllers`](https://github.com/ISIR-EXTENDER/controllers) | `c6bbebc feat: add snake mode to cartesian_velocity controller (#7)` | Shared robot controllers. |
| Sandbox controller | [`sandbox_controller`](https://github.com/ISIR-EXTENDER/sandbox_controller) | `0411619 fix: use synced joint positions for feedback (#4)` | Reference controller for new backend/UI smoke tests. |
| Robot messages | [`robot_interfaces`](https://github.com/ISIR-EXTENDER/robot_interfaces) | `1543180 Merge pull request #5 from ssrpo/fix/remove-stale-joint-pose-helper` | Provides shared ROS messages. |
| Tools | [`tools`](https://github.com/ISIR-EXTENDER/tools) | `800bed7 Merge pull request #4 from MegMll/topic/add_snake` | Provides `apriltag_detector`. |
| Visual servoing | [`visual_servoing`](https://github.com/ISIR-EXTENDER/visual_servoing) | `bc6a33a first commit` | Robin's current visual-servoing package. |

## Bloom Migration

[`Bloom`](https://github.com/ISIR-EXTENDER/bloom) is the WIP next-generation
robot UI platform. It is being developed as a monorepo that combines frontend,
backend API, widget contracts, runtime safety rules, storage, and ROS adapters.

The goal is for Bloom to replace both `extender_ui` and the current
`tablet_interface` backend flow once equivalent robot workflows are validated.
Until then, this package remains the stable backend for integration week and
Sandbox V0.0 development.

Migration rule of thumb:

1. Keep shipping stable Extender work in `extender_ui` + `tablet_interface`.
2. Use Sandbox V0.0 as the reference workflow for new controller integrations.
3. Port accepted workflows into Bloom incrementally.
4. Replace this backend only after the matching Bloom workflow is tested with
   the robot stack and accepted by the team.

## Contributing

- Write README content, comments, PR descriptions, and shared docs in English.
- Keep generic websocket behavior in the core transport/handler layer.
- Put domain-specific ROS behavior in bridge modules.
- Prefer generic messages over new custom websocket message types when possible.
- Preserve legacy Petanque compatibility unless the team explicitly removes it.
- Keep video transport separate from topic monitoring.
- Document any new ROS topic contract in both this README and the frontend
  README.
