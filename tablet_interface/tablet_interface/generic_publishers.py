from __future__ import annotations

from typing import Dict

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float64, String


class GenericPublisherCache:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._bool_publishers: Dict[str, object] = {}
        self._string_publishers: Dict[str, object] = {}
        self._float_publishers: Dict[str, object] = {}
        self._compressed_image_publishers: Dict[str, object] = {}

    def publish_bool(self, topic: str, value: bool) -> bool:
        publisher = self.ensure_bool_publisher(topic)
        if publisher is None:
            return False

        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)
        return True

    def ensure_bool_publisher(self, topic: str):
        normalized_topic = topic.strip()
        if not normalized_topic:
            self._node.get_logger().warning("Bool publisher topic is empty")
            return None

        publisher = self._bool_publishers.get(normalized_topic)
        if publisher is None:
            publisher = self._node.create_publisher(Bool, normalized_topic, 10)
            self._bool_publishers[normalized_topic] = publisher
        return publisher

    def publish_string(self, topic: str, payload: str) -> bool:
        publisher = self.ensure_string_publisher(topic)
        if publisher is None:
            return False

        msg = String()
        msg.data = payload
        publisher.publish(msg)
        return True

    def publish_float(self, topic: str, value: float) -> bool:
        publisher = self.ensure_float_publisher(topic)
        if publisher is None:
            return False

        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)
        return True

    def ensure_string_publisher(self, topic: str):
        normalized_topic = topic.strip()
        if not normalized_topic:
            self._node.get_logger().warning("String publisher topic is empty")
            return None

        publisher = self._string_publishers.get(normalized_topic)
        if publisher is None:
            publisher = self._node.create_publisher(String, normalized_topic, 10)
            self._string_publishers[normalized_topic] = publisher
        return publisher

    def ensure_float_publisher(self, topic: str):
        normalized_topic = topic.strip()
        if not normalized_topic:
            self._node.get_logger().warning("Float publisher topic is empty")
            return None

        publisher = self._float_publishers.get(normalized_topic)
        if publisher is None:
            publisher = self._node.create_publisher(Float64, normalized_topic, 10)
            self._float_publishers[normalized_topic] = publisher
        return publisher

    def publish_compressed_image(
        self,
        topic: str,
        *,
        image_format: str,
        image_bytes: bytes,
    ) -> bool:
        normalized_topic = topic.strip()
        if not normalized_topic:
            self._node.get_logger().warning("CompressedImage publisher topic is empty")
            return False

        publisher = self._compressed_image_publishers.get(normalized_topic)
        if publisher is None:
            publisher = self._node.create_publisher(CompressedImage, normalized_topic, 10)
            self._compressed_image_publishers[normalized_topic] = publisher

        msg = CompressedImage()
        msg.format = image_format
        msg.data = image_bytes
        publisher.publish(msg)
        return True
