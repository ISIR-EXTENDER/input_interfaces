# Tablet Interface

ROS 2 input interface that publishes `/teleop_cmd` from a WebSocket client.

## Run the node

```bash
ros2 run tablet_interface tablet_interface_node --ros-args --params-file config/tablet_interface_parameters_explorer.yaml
```

## Makefile (uv run)

Use the Makefile to run the node and tools with uv:

```bash
make -C . run-node
make -C . run-ws-client
make -C . test
```

## Developer WS client test

The test client streams `cmd` messages at 50 Hz and prints `state` messages from the server.

```bash
python3 scripts/ws_client_test.py --host 127.0.0.1 --port 8765 --path /ws/control
```

### WebSocket command format

UI sends normalized values in [-1, 1]. Backend applies axis mapping and scaling.

```json
{
	"type": "teleop_cmd",
	"seq": 42,
	"mode": 0,
	"linear": { "x": 0.2, "y": -0.1, "z": 0.0 },
	"angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
}
```

For pétanque integration, backend also supports:

```json
{ "type": "state_cmd", "command": "teleop" }
```

Allowed `command` values: `teleop`, `activate_throw`, `go_to_start`, `throw`, `pick_up`, `stop`.
These are published as `std_msgs/String` on `/petanque_state_machine/change_state`.

```json
{ "type": "petanque_cfg", "total_duration": 1.0 }
```

This updates `/petanque_throw` parameter `total_duration` through `/petanque_throw/set_parameters`.

`ui_button` messages are also accepted for compatibility. If `topic` matches
`/petanque_state_machine/change_state`, backend forwards `payload` to the same bridge.

### Mapping and scaling

The backend can remap and invert tablet axes before publishing `/teleop_cmd`.
This is configured through the ROS params file:

- `linear_axes` / `linear_signs`
- `angular_axes` / `angular_signs`
- `linear_scale`, `angular_scale`
- `swap_xy`

By default, tablet mapping is identity (`x->x`, `y->y`, `z->z`) and can be tuned per robot.
Robot profiles are provided in:
- `config/tablet_interface_parameters_explorer.yaml`
- `config/tablet_interface_parameters_kinova.yaml`

### Petanque measurements behavior

When `petanque_measurements_enabled` is true, the node provides a measurement pipeline:

- Input image topic: `petanque_measurement_image_topic` (`sensor_msgs/Image`)
- Input points topic: `petanque_measurement_points_topic` (`std_msgs/Float32MultiArray` with `[x1, y1, x2, y2]`)
- Overlay output topic: `petanque_measurement_overlay_topic` (`sensor_msgs/Image`, `bgr8`)
- Distance output topic: `petanque_measurement_distance_topic` (`std_msgs/Float32`, meters)

The algorithm detects sphere-like circles in the RGB image, uses known sphere diameter and camera intrinsics to recover depth, then measures distance between reconstructed 3D points linked to the two user clicks.

Intrinsics can be configured with:
- `petanque_intrinsics_mode: "image"` (default): intrinsics are estimated from image size and `petanque_assumed_hfov_deg`.
- `petanque_intrinsics_mode: "fixed"`: intrinsics are taken from `petanque_camera_fx/fy/cx/cy`.

Important: the user points remain authoritative; no click correction/snap is applied.

Main params to tune:
- `petanque_sphere_diameter_m`
- `petanque_click_to_circle_threshold_px`
- `petanque_hough_*` and min/max radius params

### End-to-end test launch (image + mock points)

Use the test launch to run:
- `offline_media_publisher/image_publisher`
- `tablet_interface_node` with petanque measurements enabled
- `mock_points_publisher` sending synthetic `[x1, y1, x2, y2]`

```bash
ros2 launch tablet_interface petanque_measurements_test.launch.py \
	image_folder:=/path/to/test/images \
	image_fps:=10 \
	point_ax:=220 point_ay:=260 point_bx:=420 point_by:=260
```

Outputs to inspect:
- `/petanque_measurements/overlay_image`
- `/petanque_measurements/distance_m`

**Expected behavior**
- While the script runs, `/teleop_cmd` should be non-zero.
- `/teleop_cmd` follows the latest UI command and mode.
