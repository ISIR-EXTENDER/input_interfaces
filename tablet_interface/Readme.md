# Tablet Interface

ROS 2 input interface that publishes `/teleop_cmd` from a WebSocket client.

## Run the node

```bash
ros2 run tablet_interface tablet_interface_node --ros-args --params-file config/tablet_interface_parameters.yaml
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

UI sends normalized values in [-1, 1]. Backend applies scaling and safety gating.

```json
{
	"type": "teleop_cmd",
	"seq": 42,
	"mode": 0,
	"linear": { "x": 0.2, "y": -0.1, "z": 0.0 },
	"angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
}
```

**Expected behavior**
- While the script runs, `/teleop_cmd` should be non-zero.
- When the script stops or disconnects, `/teleop_cmd` should return to zero after the watchdog timeout.
