from __future__ import annotations

from tablet_interface.camera_bridge import CameraBridge


class FakeLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class FakePublisherCache:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, bytes]] = []

    def publish_compressed_image(
        self,
        topic: str,
        *,
        image_format: str,
        image_bytes: bytes,
    ) -> bool:
        self.calls.append((topic, image_format, image_bytes))
        return True


def test_publish_frame_decodes_and_republishes_image_data_url() -> None:
    logger = FakeLogger()
    publishers = FakePublisherCache()
    bridge = CameraBridge(logger=logger, publishers=publishers)

    assert (
        bridge.publish_frame(
            topic="/tablet/camera/front/compressed",
            image_data_url="data:image/png;base64,AA==",
        )
        is True
    )

    assert publishers.calls == [
        ("/tablet/camera/front/compressed", "png", b"\x00")
    ]
    assert logger.infos == [
        "Published camera frame: topic=/tablet/camera/front/compressed format=png bytes=1"
    ]


def test_publish_frame_rejects_invalid_payload() -> None:
    logger = FakeLogger()
    publishers = FakePublisherCache()
    bridge = CameraBridge(logger=logger, publishers=publishers)

    assert bridge.publish_frame(topic="/tablet/camera/front/compressed", image_data_url="oops") is False

    assert publishers.calls == []
    assert logger.warnings == ["Invalid camera frame image_data_url payload"]


def test_publish_frame_rejects_empty_topic() -> None:
    logger = FakeLogger()
    publishers = FakePublisherCache()
    bridge = CameraBridge(logger=logger, publishers=publishers)

    assert bridge.publish_frame(topic="   ", image_data_url="data:image/png;base64,AA==") is False

    assert publishers.calls == []
    assert logger.warnings == ["Camera frame topic is empty"]
