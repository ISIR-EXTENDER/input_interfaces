from __future__ import annotations

from typing import Dict, Tuple

import yaml
from rclpy.node import Node
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message


def parse_message_payload_text(payload_text: str) -> object:
    trimmed_payload = payload_text.strip()
    if not trimmed_payload:
        raise ValueError("Typed message payload is empty")

    try:
        parsed_payload = yaml.safe_load(trimmed_payload)
    except yaml.YAMLError as exc:
        raise ValueError(f"Invalid typed message payload: {exc}") from exc

    if parsed_payload is None:
        raise ValueError("Typed message payload is empty")
    return parsed_payload


def build_typed_message(message_type: str, payload_text: str):
    try:
        message_cls = get_message(message_type.strip())
    except (AttributeError, ModuleNotFoundError, ValueError) as exc:
        raise ValueError(f"Unsupported ROS message type: {message_type}") from exc

    parsed_payload = parse_message_payload_text(payload_text)
    field_types = message_cls.get_fields_and_field_types()
    if isinstance(parsed_payload, dict):
        message_fields = parsed_payload
    elif len(field_types) == 1:
        message_fields = {next(iter(field_types)): parsed_payload}
    else:
        raise ValueError(
            "Typed message payload must be a mapping for multi-field ROS messages"
        )

    message = message_cls()
    try:
        set_message_fields(message, message_fields)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError(
            f"Invalid payload for message type {message_type}: {exc}"
        ) from exc
    return message


class TypedMessagePublisherCache:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._publishers: Dict[Tuple[str, str], object] = {}
        self._message_classes: Dict[str, type] = {}

    def _get_message_class(self, message_type: str):
        normalized_message_type = message_type.strip()
        if not normalized_message_type:
            raise ValueError("ROS message type is empty")

        message_cls = self._message_classes.get(normalized_message_type)
        if message_cls is None:
            try:
                message_cls = get_message(normalized_message_type)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                raise ValueError(
                    f"Unsupported ROS message type: {normalized_message_type}"
                ) from exc
            self._message_classes[normalized_message_type] = message_cls
        return message_cls

    def ensure_publisher(self, topic: str, message_type: str):
        normalized_topic = topic.strip()
        if not normalized_topic:
            raise ValueError("Typed message publisher topic is empty")

        normalized_message_type = message_type.strip()
        message_cls = self._get_message_class(normalized_message_type)
        cache_key = (normalized_topic, normalized_message_type)
        publisher = self._publishers.get(cache_key)
        if publisher is None:
            publisher = self._node.create_publisher(message_cls, normalized_topic, 10)
            self._publishers[cache_key] = publisher
        return publisher

    def publish(self, topic: str, message_type: str, payload_text: str) -> bool:
        publisher = self.ensure_publisher(topic, message_type)
        message = build_typed_message(message_type, payload_text)
        publisher.publish(message)
        return True
