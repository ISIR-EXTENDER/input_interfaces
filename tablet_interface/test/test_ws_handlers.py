from __future__ import annotations

import asyncio

from tablet_interface.ws_handlers import build_state_message, handle_ws_payload


class FakeSender:
    def __init__(self) -> None:
        self.messages: list[dict[str, object]] = []

    async def send_json(self, data: dict[str, object]) -> None:
        self.messages.append(data)


class FakeLogger:
    def __init__(self) -> None:
        self.debugs: list[str] = []
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def debug(self, message: str) -> None:
        self.debugs.append(message)

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class FakeNode:
    def __init__(self) -> None:
        self.state_machine_topic = "/petanque_state_machine/change_state"
        self.hub_digital_output_topic = "/hub/digital_output"
        self.logger = FakeLogger()
        self.bool_calls: list[tuple[str, bool]] = []
        self.scalar_calls: list[tuple[str, float]] = []
        self.typed_calls: list[tuple[str, str, str]] = []
        self.camera_calls: list[tuple[str, str]] = []
        self.measure_snapshot: dict[str, object] = {
            "image_data_url": "data:image/png;base64,AA==",
            "vectors_json": '{"source":"opencv"}',
            "updated_at_ms": 1234,
        }

    def get_logger(self) -> FakeLogger:
        return self.logger

    def publish_ui_bool(self, topic: str, value: bool) -> bool:
        self.bool_calls.append((topic, value))
        return True

    def publish_ui_scalar(self, topic: str, value: float) -> bool:
        self.scalar_calls.append((topic, value))
        return True

    def publish_ui_typed(
        self,
        topic: str,
        message_type: str,
        payload_text: str,
    ) -> bool:
        self.typed_calls.append((topic, message_type, payload_text))
        return True

    def publish_camera_frame(self, *, topic: str, image_data_url: str) -> bool:
        self.camera_calls.append((topic, image_data_url))
        return True

    def get_measure_result_snapshot(self) -> dict[str, object]:
        return dict(self.measure_snapshot)


def test_build_state_message_preserves_sandbox_feedback_fields() -> None:
    message = build_state_message(
        {
            "connected": True,
            "cmd_age_ms": 15,
            "watchdog_timeout_ms": 0,
            "last_seq": 7,
            "publishing_rate_hz": 30.0,
            "current_mode": 2,
            "gripper_state": "open",
            "ee_pose": {"x": 0.1, "y": -0.2, "z": 0.3},
            "tcp_speed_mps": 0.42,
            "joint_positions": [1.0, 2.0, 3.0],
        }
    )

    assert message == {
        "type": "state",
        "connected": True,
        "cmd_age_ms": 15,
        "watchdog_timeout_ms": 0,
        "last_seq": 7,
        "publishing_rate_hz": 30.0,
        "current_mode": 2,
        "gripper_state": "open",
        "ee_pose": {"x": 0.1, "y": -0.2, "z": 0.3},
        "tcp_speed_mps": 0.42,
        "joint_positions": [1.0, 2.0, 3.0],
    }


def test_handle_ws_payload_ui_scalar_emits_success_event() -> None:
    node = FakeNode()
    sender = FakeSender()

    asyncio.run(
        handle_ws_payload(
            node,
            sender,
            {
                "type": "ui_scalar",
                "topic": "/cmd/max_velocity",
                "value": 0.75,
                "widget_id": "sandbox-max-velocity",
            },
        )
    )

    assert node.scalar_calls == [("/cmd/max_velocity", 0.75)]
    assert sender.messages == [
        {
            "type": "event",
            "severity": "info",
            "code": "UI_SCALAR_OK",
            "message": "ui_scalar topic=/cmd/max_velocity value=0.750",
        }
    ]


def test_handle_ws_payload_ui_bool_emits_success_event() -> None:
    node = FakeNode()
    sender = FakeSender()

    asyncio.run(
        handle_ws_payload(
            node,
            sender,
            {
                "type": "ui_bool",
                "topic": "/sandbox/digital_output",
                "value": True,
                "widget_id": "sandbox-toggle-output",
            },
        )
    )

    assert node.bool_calls == [("/sandbox/digital_output", True)]
    assert sender.messages == [
        {
            "type": "event",
            "severity": "info",
            "code": "UI_BOOL_OK",
            "message": "ui_bool topic=/sandbox/digital_output value=true",
        }
    ]


def test_handle_ws_payload_ui_typed_emits_success_event() -> None:
    node = FakeNode()
    sender = FakeSender()

    asyncio.run(
        handle_ws_payload(
            node,
            sender,
            {
                "type": "ui_typed",
                "topic": "/petanque_state_machine/change_state",
                "message_type": "std_msgs/msg/String",
                "payload_text": "{data: 'activate_throw'}",
                "widget_id": "toggle-typed",
            },
        )
    )

    assert node.typed_calls == [
        (
            "/petanque_state_machine/change_state",
            "std_msgs/msg/String",
            "{data: 'activate_throw'}",
        )
    ]
    assert sender.messages == [
        {
            "type": "event",
            "severity": "info",
            "code": "UI_TYPED_OK",
            "message": "ui_typed topic=/petanque_state_machine/change_state message_type=std_msgs/msg/String",
        }
    ]


def test_handle_ws_payload_measure_refresh_sends_cached_result_then_event() -> None:
    node = FakeNode()
    sender = FakeSender()

    asyncio.run(handle_ws_payload(node, sender, {"type": "measure_refresh"}))

    assert sender.messages == [
        {
            "type": "measure_result",
            "image_data_url": "data:image/png;base64,AA==",
            "vectors_json": '{"source":"opencv"}',
            "updated_at_ms": 1234,
        },
        {
            "type": "event",
            "severity": "info",
            "code": "MEASURE_REFRESH_OK",
            "message": "sent cached measure result",
        },
    ]


def test_handle_ws_payload_camera_frame_emits_success_event() -> None:
    node = FakeNode()
    sender = FakeSender()

    asyncio.run(
        handle_ws_payload(
            node,
            sender,
            {
                "type": "camera_frame",
                "topic": "/tablet/camera/front/compressed",
                "image_data_url": "data:image/jpeg;base64,AAAAAAAAAAAAAA==",
                "widget_id": "camera-front",
            },
        )
    )

    assert node.camera_calls == [
        (
            "/tablet/camera/front/compressed",
            "data:image/jpeg;base64,AAAAAAAAAAAAAA==",
        )
    ]
    assert sender.messages == [
        {
            "type": "event",
            "severity": "info",
            "code": "CAMERA_FRAME_OK",
            "message": "camera_frame topic=/tablet/camera/front/compressed",
        }
    ]
