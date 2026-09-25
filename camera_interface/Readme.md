# camera_interface

One camera bring-up and one set of topic names, whichever camera the robot has.

| Camera | Driver |
| --- | --- |
| Explorer's USB camera, any webcam | `usb_cam` |
| Raspberry Pi camera modules (libcamera) | `camera_ros` |
| Kinova gen3 integrated camera | `kinova_vision` |

Whatever the driver, the topics come out at:

- `/camera/color/image_raw` — `sensor_msgs/msg/Image`
- `/camera/color/image_raw/compressed` — `sensor_msgs/msg/CompressedImage`
- `/camera/color/camera_info` — `sensor_msgs/msg/CameraInfo`

## Use

```bash
# Explorer's USB camera
ros2 launch camera_interface camera.launch.py driver:=usb_cam \
  params_file:=$(ros2 pkg prefix camera_interface)/share/camera_interface/config/explorer_camera.yaml

# Any webcam, for a bench test
ros2 launch camera_interface camera.launch.py driver:=usb_cam \
  params_file:=$(ros2 pkg prefix camera_interface)/share/camera_interface/config/usb_camera.yaml

# Kinova gen3 integrated camera
ros2 launch camera_interface camera.launch.py driver:=kinova_vision

# A camera started elsewhere
ros2 launch camera_interface camera.launch.py driver:=none
```

| Argument | Default | Description |
| --- | --- | --- |
| `driver` | `usb_cam` | `usb_cam`, `camera_ros`, `kinova_vision`, or `none`. |
| `params_file` | none | Camera parameters, from `config/`. |
| `namespace` | `/camera/color` | Where the three topics come out. |
| `republish_compressed` | `false` | For a driver that publishes raw without the compressed transport plugin. |
| `container_name` | none | Load the camera into a running component container. |

## Notes

- `usb_cam` and `camera_ros` run as components, so a detector in the same container reads frames without a copy.
- `kinova_vision` is included, not respawned, and needs the arm on its network (`192.168.1.10` by default).
- `apriltag_detector` subscribes to absolute `/image_raw` and `/camera_info`; `apriltag_remappings()` in the
  launch file returns the remaps it needs.
- Point `camera_info_url` at the physical camera's calibration.
- The Kinova camera frames are not in the description today: `cartesian_manager` builds `gen3.xacro` without
  `vision:=true`, and `kinova_vision.launch.py` publishes its own static transforms for the same frames.

## Tests

```bash
python3 -m pytest camera_interface/test
```
