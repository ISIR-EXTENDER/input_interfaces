from __future__ import annotations

import asyncio

try:
    import uvicorn
    from fastapi import FastAPI, WebSocket
    from fastapi.websockets import WebSocketState
    from pydantic import ValidationError
except Exception:  # pragma: no cover
    uvicorn = None  # type: ignore
    FastAPI = None  # type: ignore
    WebSocket = None  # type: ignore
    WebSocketState = None  # type: ignore
    ValidationError = Exception  # type: ignore

from tablet_interface.ros_teleop_publisher import TabletInterfaceNode
from tablet_interface.safety_gate import Twist


def run_uvicorn_server(node: TabletInterfaceNode) -> None:
    if uvicorn is None or FastAPI is None:
        node.get_logger().error(
            "fastapi/uvicorn not available. WebSocket server cannot start."
        )
        return

    try:
        from tablet_interface.ws_models import CmdMessage, EventMessage
    except Exception as exc:  # pragma: no cover
        node.get_logger().error(
            f"pydantic not available. WebSocket server cannot start: {exc}"
        )
        return

    host = node.get_parameter("bind_host").value
    port = int(node.get_parameter("bind_port").value)
    ws_path = node.get_parameter("ws_path").value
    linear_scale = float(node.get_parameter("linear_scale").value)
    angular_scale = float(node.get_parameter("angular_scale").value)
    z_scale = float(node.get_parameter("z_scale").value)

    app = FastAPI()

    async def _send_event(
        websocket: WebSocket,
        code: str,
        severity: str,
        message: str,
    ) -> None:
        event = EventMessage(type="event", severity=severity, code=code, message=message)
        await websocket.send_json(event.model_dump())

    async def _state_sender(websocket: WebSocket) -> None:
        interval = 1.0 / max(node.state_publish_hz, 1e-3)
        while True:
            await asyncio.sleep(interval)
            state = node.get_state()
            message = {
                "type": "state",
                "connected": state["connected"],
                "cmd_age_ms": state["cmd_age_ms"],
                "watchdog_timeout_ms": state["watchdog_timeout_ms"],
                "last_seq": state["last_seq"],
                "publishing_rate_hz": state["publishing_rate_hz"],
                "current_mode": state["current_mode"],
            }
            await websocket.send_json(message)

    @app.websocket(ws_path)
    async def ws_endpoint(websocket: WebSocket) -> None:
        await websocket.accept()
        node.set_connected(True)
        node.get_logger().info("WS client connected")
        await _send_event(websocket, "WS_CONNECTED", "info", "WS connected")

        state_task = asyncio.create_task(_state_sender(websocket))
        try:
            while True:
                payload = await websocket.receive_json()
                try:
                    cmd = CmdMessage.model_validate(payload)
                except ValidationError as exc:
                    node.get_logger().warning(f"WS cmd invalid: {exc}")
                    await _send_event(
                        websocket,
                        code="CMD_INVALID",
                        severity="warning",
                        message=str(exc),
                    )
                    continue

                twist = Twist()
                twist.linear.x = cmd.linear.x * linear_scale
                twist.linear.y = cmd.linear.y * linear_scale
                twist.linear.z = cmd.linear.z * z_scale
                twist.angular.x = cmd.angular.x * angular_scale
                twist.angular.y = cmd.angular.y * angular_scale
                twist.angular.z = cmd.angular.z * angular_scale

                node.update_latest_cmd(
                    twist=twist,
                    mode=int(cmd.mode),
                    seq=int(cmd.seq),
                    received_ms=node._now_ms(),
                )
                node.get_logger().debug(
                    f"WS cmd accepted seq={cmd.seq} mode={cmd.mode}"
                )
        except Exception:
            pass
        finally:
            state_task.cancel()
            node.set_connected(False)
            node.get_logger().info("WS client disconnected")
            if WebSocketState is not None and websocket.client_state == WebSocketState.CONNECTED:
                try:
                    await _send_event(
                        websocket,
                        "WS_DISCONNECTED",
                        "info",
                        "WS disconnected",
                    )
                except Exception:
                    pass

    node.get_logger().info(f"WebSocket listening on ws://{host}:{port}{ws_path}")
    uvicorn.run(app, host=host, port=port, log_level="info")
