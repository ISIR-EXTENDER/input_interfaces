# Input Interfaces

This directory contains ROS 2 input backends for Extender robots.

## Packages

### `joystick_interface`

Classic ROS teleoperation from joysticks and 3D mice.

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
