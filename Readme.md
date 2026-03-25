# Input Interfaces

This directory contains ROS2 packages that provide input interfaces for robot teleoperation and control in the extender project. These interfaces allow users to provide commands to robots through various input devices and methods.

## Available Input Interfaces

### Joystick Interface (`joystick_interface`)

The joystick interface package enables teleoperation using standard joysticks and 3D mice (SpaceMouse). It processes joystick inputs and publishes structured teleoperation commands.

For detailed documentation, see [`joystick_interface/Readme.md`](joystick_interface/Readme.md).

**Key Features:**
- Support for 2D joysticks (`/joy` topic) and SpaceMouse (`/spacenav/joy` topic)
- Multiple control modes: translation+rotation, rotation-only, translation-only, and full 6DOF
- Configurable axis mappings and scaling factors
- Integrated gripper control for Franka robots
- Custom `TeleopCommand` message with twist and mode information

**Nodes:**
- `joystick_input_node`: Main teleoperation node
- `franka_gripper_node`: Gripper control node

### Tablet Interface (`tablet_interface`)

The tablet interface package is the websocket backend used by `extender_ui`. It publishes `extender_msgs/TeleopCommand` on `/teleop_cmd`, preserves the legacy pétanque bridges, and now exposes generic `ui_button` / `ui_scalar` actions plus `camera_frame` image ingress for sandbox-style applications.

For detailed documentation, see [`tablet_interface/Readme.md`](tablet_interface/Readme.md).

**Key Features:**
- WebSocket bridge for the tablet UI
- Generic scalar/button topic publishing for sandbox experiments
- Generic browser-camera frame publishing into ROS topics
- Optional sandbox controller feedback forwarding to the UI
- Workspace-level `uv` development workflow


## Adding New Input Interfaces

To add a new input interface:

1. Create a new ROS2 package in this directory
2. Follow the naming convention: `{device}_interface`
3. Implement nodes that publish to standardized command topics
4. Provide configuration files and documentation
5. Update this README
