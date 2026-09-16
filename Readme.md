# Input Interfaces

This directory contains ROS 2 input backends for Extender robots.

Input interfaces translate hardware- or UI-specific input into ROS messages that
the rest of the stack can consume. New work should prefer generic output topics
and structured mode requests over robot-specific teleoperation messages.

## Packages

### `joystick_mapper`

Recommended joystick and 3D mouse mapper for Cartesian control.

`joystick_mapper` subscribes to `sensor_msgs/msg/Joy` and publishes:

- `geometry_msgs/msg/TwistStamped` Cartesian velocity commands,
- `std_msgs/msg/String` mode requests such as `geometric/jaco`,
  `geometric/snake`, and `behaviour/joint_target/home`.

It keeps joystick-specific details in configuration: axis indexes, signs,
deadzones, local B1/B2 axis maps, and button activation modes (`toggle`,
`hold`, or `trigger`).

Use this package for new joystick integrations, especially with
`cartesian_manager`.

See [`joystick_mapper/README.md`](joystick_mapper/README.md).

### `joystick_interface`

Legacy ROS teleoperation from joysticks and 3D mice.

This package is kept temporarily for existing launch files and workflows. It is
planned to disappear once the remaining users have migrated to
`joystick_mapper`, roughly during the next month. Avoid adding new features
here; put new joystick behavior in `joystick_mapper` instead.

See [`joystick_interface/Readme.md`](joystick_interface/Readme.md).

### `keyboard_interface`

Configurable keyboard input that publishes each mapped key state to a dedicated
`std_msgs/msg/Bool` topic. Presses publish `true`, releases publish `false`, and
keyboard auto-repeat events are suppressed.

See [`keyboard_interface/README.md`](keyboard_interface/README.md).

### `tablet_interface`

Websocket backend used by `extender_ui`.

It now provides:

- generic teleop publishing
- generic UI actions for sandbox-style apps
- camera frame ingress from the browser into ROS topics
- compatibility bridges for the existing pétanque workflow

See [`tablet_interface/Readme.md`](tablet_interface/Readme.md).

### `visual_servoing`

AprilTag-based visual servoing input for camera-guided Cartesian motion.

`visual_servoing` subscribes to:

- `extender_msgs/msg/SharedControlGoalArray` tag detections on
  `/tag_detections`,
- `std_msgs/msg/Bool` enable/disable commands on `/ui/visual_servoing/on`,
- `std_msgs/msg/String` save requests on `/ui/visual_servoing/save`.

It publishes:

- `geometry_msgs/msg/TwistStamped` Cartesian velocity commands on
  `/visual_servoing/velocity_command`,
- `geometry_msgs/msg/TwistStamped` debug/error telemetry on
  `/visual_servoing/error_TAGtoTAGd`.

The launch file resolves its parameter and calibration files from the installed
`visual_servoing` package share. When routing visual servoing through
`cartesian_manager`, make sure `cartesian_manager.topics.visual_servoing_command`
matches `/visual_servoing/velocity_command`, or remap/rename the publisher to
the manager's configured input topic.

## Contribution rule

New input backends should follow the same pattern:

- keep the transport layer generic
- isolate robot/app-specific behavior in dedicated modules
- document the public topics and messages clearly

For joystick-style hardware, prefer adding a configuration file or a small
extension to `joystick_mapper` before creating a new package.
