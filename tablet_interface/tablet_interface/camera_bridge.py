from __future__ import annotations

from typing import Protocol

from tablet_interface.measure_codec import decode_image_data_url


class LoggerLike(Protocol):
    def info(self, message: str) -> None:
        ...

    def warning(self, message: str) -> None:
        ...


class CompressedImagePublisherCacheLike(Protocol):
    def publish_compressed_image(
        self,
        topic: str,
        *,
        image_format: str,
        image_bytes: bytes,
    ) -> bool:
        ...


class CameraBridge:
    def __init__(
        self,
        *,
        logger: LoggerLike,
        publishers: CompressedImagePublisherCacheLike,
    ) -> None:
        self._logger = logger
        self._publishers = publishers

    def publish_frame(self, *, topic: str, image_data_url: str) -> bool:
        normalized_topic = topic.strip()
        if not normalized_topic:
            self._logger.warning("Camera frame topic is empty")
            return False

        decoded = decode_image_data_url(image_data_url)
        if decoded is None:
            self._logger.warning("Invalid camera frame image_data_url payload")
            return False

        image_format, image_bytes = decoded
        ok = self._publishers.publish_compressed_image(
            normalized_topic,
            image_format=image_format,
            image_bytes=image_bytes,
        )
        if not ok:
            return False

        self._logger.info(
            "Published camera frame: topic={0} format={1} bytes={2}".format(
                normalized_topic,
                image_format,
                len(image_bytes),
            )
        )
        return True

