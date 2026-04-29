from __future__ import annotations

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from tablet_interface.measure_bridge import MeasureBridge
from tablet_interface.runtime_state import TabletRuntimeState


class FakeLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[CompressedImage] = []

    def publish(self, msg: CompressedImage) -> None:
        self.messages.append(msg)


def create_runtime_state() -> TabletRuntimeState:
    return TabletRuntimeState(
        default_mode=2,
        publish_rate_hz=60.0,
        gripper_open_position=0.2,
        gripper_close_position=1.1,
        measure_demo_vectors_json='{"source":"demo"}',
        measure_demo_image_data_url="data:image/png;base64,demo",
    )


def create_bridge() -> tuple[MeasureBridge, FakeLogger, FakePublisher, TabletRuntimeState]:
    logger = FakeLogger()
    publisher = FakePublisher()
    runtime_state = create_runtime_state()
    bridge = MeasureBridge(
        logger=logger,
        request_image_publisher=publisher,
        runtime_state=runtime_state,
        request_image_topic="/petanque/measure/request_image/compressed",
        result_image_topic="/petanque/measure/result_image/compressed",
        result_vectors_topic="/petanque/measure/result_vectors",
    )
    return bridge, logger, publisher, runtime_state


def test_publish_request_image_accepts_png_data_url() -> None:
    bridge, logger, publisher, _ = create_bridge()

    ok = bridge.publish_request_image("data:image/png;base64,AA==")

    assert ok is True
    assert len(publisher.messages) == 1
    assert publisher.messages[0].format == "png"
    assert bytes(publisher.messages[0].data) == b"\x00"
    assert logger.infos == [
        "Published measure request image: topic=/petanque/measure/request_image/compressed format=png bytes=1"
    ]


def test_publish_request_image_rejects_invalid_payload() -> None:
    bridge, logger, publisher, _ = create_bridge()

    ok = bridge.publish_request_image("not-a-data-url")

    assert ok is False
    assert publisher.messages == []
    assert logger.warnings == ["Invalid measure image_data_url payload"]


def test_on_result_image_updates_runtime_state() -> None:
    bridge, logger, _, runtime_state = create_bridge()
    msg = CompressedImage()
    msg.format = "png"
    msg.data = b"\x01\x02"

    bridge.on_result_image(msg, now_ms=1234)

    snapshot = runtime_state.get_measure_result_snapshot()
    assert snapshot["image_data_url"] == "data:image/png;base64,AQI="
    assert snapshot["updated_at_ms"] == 1234
    assert logger.infos == [
        "Received measure result image: topic=/petanque/measure/result_image/compressed format=png bytes=2"
    ]


def test_on_result_vectors_updates_runtime_state() -> None:
    bridge, logger, _, runtime_state = create_bridge()
    msg = String()
    msg.data = '{"source":"opencv","distances_cm":[12.3]}'

    bridge.on_result_vectors(msg, now_ms=2222)

    snapshot = runtime_state.get_measure_result_snapshot()
    assert snapshot["vectors_json"] == '{"source":"opencv","distances_cm":[12.3]}'
    assert snapshot["updated_at_ms"] == 2222
    assert logger.infos == [
        "Received measure vectors: topic=/petanque/measure/result_vectors chars=41"
    ]
