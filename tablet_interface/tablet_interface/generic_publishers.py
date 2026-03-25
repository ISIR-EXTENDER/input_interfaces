from __future__ import annotations

from typing import Dict

from rclpy.node import Node
from std_msgs.msg import Float64, String


class GenericPublisherCache:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._string_publishers: Dict[str, object] = {}
        self._float_publishers: Dict[str, object] = {}

    def publish_string(self, topic: str, payload: str) -> bool:
        normalized_topic = topic.strip()
        if not normalized_topic:
            self._node.get_logger().warning("String publisher topic is empty")
            return False

        publisher = self._string_publishers.get(normalized_topic)
        if publisher is None:
            publisher = self._node.create_publisher(String, normalized_topic, 10)
            self._string_publishers[normalized_topic] = publisher

        msg = String()
        msg.data = payload
        publisher.publish(msg)
        return True

    def publish_float(self, topic: str, value: float) -> bool:
        normalized_topic = topic.strip()
        if not normalized_topic:
            self._node.get_logger().warning("Float publisher topic is empty")
            return False

        publisher = self._float_publishers.get(normalized_topic)
        if publisher is None:
            publisher = self._node.create_publisher(Float64, normalized_topic, 10)
            self._float_publishers[normalized_topic] = publisher

        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)
        return True
