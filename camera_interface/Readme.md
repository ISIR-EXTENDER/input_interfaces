# camera_interface

One camera bring-up, one set of topic names, whichever camera the robot has.

## Why

Every consumer in this workspace wants the same three things: an image, its calibration, and a compressed
image small enough to send somewhere. What they get today depends on the camera:

| Camera | Publishes |
| --- | --- |
| `kinova_vision` (Kinova gen3 integrated) | `/camera/color/image_raw`, `/camera/color/camera_info` |
| `usb_cam` (Explorer's USB camera, any webcam) | relative `image_raw`, `camera_info` |
| `camera_ros` (libcamera, Raspberry Pi modules) | `~/image_raw`, `~/camera_info` under its node name |

So each consumer learned one camera, and changing camera meant changing the consumer. This package is the
seam between them.

## Use

```bash
# Explorer's USB camera
ros2 launch camera_interface camera.launch.py \
  driver:=usb_cam \
  params_file:=$(ros2 pkg prefix camera_interface)/share/camera_interface/config/explorer_camera.yaml

# The Kinova gen3's integrated camera
ros2 launch camera_interface camera.launch.py driver:=kinova_vision

# Or, when the camera was started by hand, or came up with the arm
ros2 launch camera_interface camera.launch.py driver:=none

# Any webcam, for a bench test
ros2 launch camera_interface camera.launch.py driver:=usb_cam \
  params_file:=.../config/usb_camera.yaml
```

Whatever the driver, the topics come out at:

- `/camera/color/image_raw` — `sensor_msgs/msg/Image`
- `/camera/color/camera_info` — `sensor_msgs/msg/CameraInfo`
- `/camera/color/image_raw/compressed` — `sensor_msgs/msg/CompressedImage`

`namespace:=` moves all three together, so a second camera can run beside the first.

### Arguments

| Argument | Default | What it does |
| --- | --- | --- |
| `driver` | `usb_cam` | `usb_cam`, `camera_ros`, `kinova_vision`, or `none` for a camera somebody else started. |
| `params_file` | none | A robot's camera parameters, from `config/`. |
| `namespace` | `/camera/color` | Where the normalized topics come out. |
| `republish_compressed` | `false` | Only for a driver that publishes raw without the compressed transport plugin. |
| `container_name` | none | Join a running component container instead of starting one. |

## Notes worth knowing

**The compressed topic usually costs nothing.** `usb_cam` and `camera_ros` publish it themselves through
`image_transport`, as long as `compressed_image_transport` is installed, which it is on the Jazzy baseline.
`republish_compressed:=true` is for a driver that does not.

**Frames are not copied when they do not have to be.** `usb_cam` and `camera_ros` register `rclcpp`
components, so they run inside a container with intra-process comms. Pass `container_name` to put the camera
in the same container as a detector.

**`kinova_vision` is included, not respawned.** Its own launch file owns two nodes, the RTSP stream
configuration, a `depth_image_proc` container and two static transforms. Rebuilding that here would go stale
the first time any of it changed, so `driver:=kinova_vision` includes it and passes the namespace. It takes
the parent and appends `color/` itself, so `/camera/color` means passing it `camera:=camera`; a namespace not
ending in `/color` is refused out loud rather than published somewhere unexpected.

**`apriltag_detector` cannot follow a namespace.** It subscribes to `/image_raw` and `/camera_info`
absolutely, in `detector.cpp`, so a launch file that starts it alongside this one has to remap those two.
`apriltag_remappings()` in `launch/camera.launch.py` returns exactly what it needs.

**Calibration belongs to the camera, not the robot.** Point `camera_info_url` at the calibration for the
physical camera. `apriltag_detector/config/explorer_camera_calib.yaml` carries the measured one for
Explorer's.

**The Kinova camera frames need one choice made, upstream.** Both halves exist and they collide:

- `kortex_description`'s `gen3_macro.xacro` defines `camera_link`, `camera_color_frame` and
  `camera_depth_frame` under `end_effector_link`, behind a `vision` xacro argument.
- `kinova_vision.launch.py` publishes its own static `camera_link` to `camera_color_frame` and
  `camera_depth_frame`.

Turn both on and those two frames get two parents. Today the question does not arise on our stack, because
`cartesian_manager`'s `kinova.launch.py` builds `gen3.xacro`, whose `vision` argument defaults to `false` and
is never passed: the camera frames are simply absent from the description, and `camera_link` has no parent,
so nothing can place the image relative to the arm.

Making it work means `vision:=true` on the description **and** suppressing the launch's static transforms.
`kinova_vision.launch.py` has no argument for that today, so it is an upstream change in both places, not
something this package can do.

## Tests

```bash
cd camera_interface && python3 -m pytest test/
```

They read the launch description without a camera, a driver package or a ROS graph, because the promise this
package makes is about topic names and that is where it is kept or broken.
