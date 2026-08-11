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

### `tablet_interface`

Websocket backend used by `extender_ui`.

It now provides:

- generic teleop publishing
- generic UI actions for sandbox-style apps
- camera frame ingress from the browser into ROS topics
- compatibility bridges for the existing pétanque workflow

See [`tablet_interface/Readme.md`](tablet_interface/Readme.md).

## Contribution rule

New input backends should follow the same pattern:

- keep the transport layer generic
- isolate robot/app-specific behavior in dedicated modules
- document the public topics and messages clearly

For joystick-style hardware, prefer adding a configuration file or a small
extension to `joystick_mapper` before creating a new package.
