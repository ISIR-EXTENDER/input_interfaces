from __future__ import annotations

import threading
from collections.abc import Callable, Mapping, Sequence
from typing import Any

from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


TopicMonitorKey = tuple[str, str]
BLOCKED_TOPIC_MONITOR_MESSAGE_TYPES = {
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
}


def parse_topic_monitor_spec(raw_spec: str) -> tuple[str, str]:
    parts = raw_spec.split("|", 1)
    if len(parts) != 2:
        raise ValueError(
            "Topic monitor specs must use '<topic>|<message_type>' format"
        )

    topic = parts[0].strip()
    message_type = parts[1].strip()
    if not topic:
        raise ValueError("Topic monitor spec topic is empty")
    if not message_type:
        raise ValueError("Topic monitor spec message type is empty")
    return topic, message_type


def to_jsonable(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [to_jsonable(item) for item in value]
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    return value


def is_blocked_topic_monitor_message_type(message_type: str) -> bool:
    return message_type.strip() in BLOCKED_TOPIC_MONITOR_MESSAGE_TYPES


class TopicMonitorBridge:
    def __init__(self, *, node: Node, now_ms: Callable[[], int]) -> None:
        self._node = node
        self._now_ms = now_ms
        self._lock = threading.Lock()
        self._subscriptions: dict[TopicMonitorKey, object] = {}
        self._snapshots: dict[TopicMonitorKey, dict[str, object]] = {}
        self._revisions: dict[TopicMonitorKey, int] = {}

    def ensure_subscription(self, topic: str, message_type: str) -> tuple[bool, str]:
        normalized_topic = topic.strip()
        normalized_message_type = message_type.strip()
        if not normalized_topic:
            return False, "topic is empty"
        if not normalized_message_type:
            return False, "message_type is empty"
        if is_blocked_topic_monitor_message_type(normalized_message_type):
            return (
                False,
                (
                    "image/video topics are not supported by topic_monitor; "
                    "use the camera stream or camera_frame path instead"
                ),
            )

        key = (normalized_topic, normalized_message_type)
        with self._lock:
            if key in self._subscriptions:
                return True, "already subscribed"

        try:
            message_cls = get_message(normalized_message_type)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            return False, f"unsupported ROS message type: {normalized_message_type}: {exc}"

        def callback(message: object) -> None:
            self._record_message(
                key=key,
                topic=normalized_topic,
                message_type=normalized_message_type,
                message=message,
            )

        try:
            subscription = self._node.create_subscription(
                message_cls,
                normalized_topic,
                callback,
                10,
            )
        except Exception as exc:
            return False, f"failed to subscribe to {normalized_topic}: {exc}"

        with self._lock:
            self._subscriptions[key] = subscription
            self._snapshots.setdefault(
                key,
                {
                    "topic": normalized_topic,
                    "message_type": normalized_message_type,
                    "updated_at_ms": None,
                    "revision": 0,
                    "data": None,
                    "error": None,
                },
            )
            self._revisions.setdefault(key, 0)

        self._node.get_logger().info(
            "Topic monitor subscribed: topic={0} message_type={1}".format(
                normalized_topic,
                normalized_message_type,
            )
        )
        return True, "subscribed"

    def ensure_spec(self, raw_spec: str) -> tuple[bool, str]:
        try:
            topic, message_type = parse_topic_monitor_spec(raw_spec)
        except ValueError as exc:
            return False, str(exc)
        return self.ensure_subscription(topic, message_type)

    def get_snapshots(self) -> list[dict[str, object]]:
        with self._lock:
            return [
                dict(snapshot)
                for _, snapshot in sorted(
                    self._snapshots.items(),
                    key=lambda item: item[0][0],
                )
            ]

    def _record_message(
        self,
        *,
        key: TopicMonitorKey,
        topic: str,
        message_type: str,
        message: object,
    ) -> None:
        try:
            data = to_jsonable(message_to_ordereddict(message))
            error = None
        except Exception as exc:
            data = None
            error = f"failed to serialize message: {exc}"

        with self._lock:
            revision = self._revisions.get(key, 0) + 1
            self._revisions[key] = revision
            self._snapshots[key] = {
                "topic": topic,
                "message_type": message_type,
                "updated_at_ms": self._now_ms(),
                "revision": revision,
                "data": data,
                "error": error,
            }


__all__ = [
    "BLOCKED_TOPIC_MONITOR_MESSAGE_TYPES",
    "TopicMonitorBridge",
    "is_blocked_topic_monitor_message_type",
    "parse_topic_monitor_spec",
    "to_jsonable",
]
