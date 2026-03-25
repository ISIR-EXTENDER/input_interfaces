from __future__ import annotations

from typing import Protocol

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from tablet_interface.measure_codec import (
    decode_image_data_url,
    encode_compressed_image_data_url,
)
from tablet_interface.runtime_state import TabletRuntimeState


class LoggerLike(Protocol):
    def info(self, message: str) -> None:
        ...

    def warning(self, message: str) -> None:
        ...


class CompressedImagePublisherLike(Protocol):
    def publish(self, msg: CompressedImage) -> None:
        ...


class MeasureBridge:
    def __init__(
        self,
        *,
        logger: LoggerLike,
        request_image_publisher: CompressedImagePublisherLike,
        runtime_state: TabletRuntimeState,
        request_image_topic: str,
        result_image_topic: str,
        result_vectors_topic: str,
    ) -> None:
        self._logger = logger
        self._request_image_publisher = request_image_publisher
        self._runtime_state = runtime_state
        self._request_image_topic = request_image_topic
        self._result_image_topic = result_image_topic
        self._result_vectors_topic = result_vectors_topic

    def publish_request_image(self, image_data_url: str) -> bool:
        decoded = decode_image_data_url(image_data_url)
        if decoded is None:
            self._logger.warning("Invalid measure image_data_url payload")
            return False

        image_format, image_bytes = decoded
        msg = CompressedImage()
        msg.format = image_format
        msg.data = image_bytes
        self._request_image_publisher.publish(msg)
        self._logger.info(
            "Published measure request image: topic={0} format={1} bytes={2}".format(
                self._request_image_topic,
                image_format,
                len(image_bytes),
            )
        )
        return True

    def get_result_snapshot(self) -> dict[str, object]:
        return self._runtime_state.get_measure_result_snapshot()

    def on_result_image(self, msg: CompressedImage, *, now_ms: int) -> None:
        image_data_url = encode_compressed_image_data_url(msg)
        if not image_data_url:
            self._logger.warning("Received empty measure result image")
            return

        self._runtime_state.update_measure_result_image(
            image_data_url,
            now_ms=now_ms,
        )

        self._logger.info(
            "Received measure result image: topic={0} format={1} bytes={2}".format(
                self._result_image_topic,
                msg.format or "jpeg",
                len(msg.data),
            )
        )

    def on_result_vectors(self, msg: String, *, now_ms: int) -> None:
        self._runtime_state.update_measure_result_vectors(
            msg.data,
            now_ms=now_ms,
        )

        self._logger.info(
            "Received measure vectors: topic={0} chars={1}".format(
                self._result_vectors_topic,
                len(msg.data),
            )
        )

