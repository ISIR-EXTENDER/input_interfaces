from __future__ import annotations

import pytest
from std_msgs.msg import String

from tablet_interface.topic_monitor_bridge import (
    TopicMonitorBridge,
    parse_topic_monitor_spec,
    to_jsonable,
)


class FakeLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class FakeNode:
    def __init__(self) -> None:
        self.logger = FakeLogger()
        self.subscriptions: list[tuple[type, str, object, int]] = []

    def get_logger(self) -> FakeLogger:
        return self.logger

    def create_subscription(
        self,
        message_cls: type,
        topic: str,
        callback: object,
        qos: int,
    ) -> object:
        subscription = object()
        self.subscriptions.append((message_cls, topic, callback, qos))
        return subscription


def test_parse_topic_monitor_spec_valid() -> None:
    assert parse_topic_monitor_spec(" /topic | std_msgs/msg/String ") == (
        "/topic",
        "std_msgs/msg/String",
    )


@pytest.mark.parametrize("raw_spec", ["", "/topic", "|std_msgs/msg/String", "/topic|"])
def test_parse_topic_monitor_spec_invalid(raw_spec: str) -> None:
    with pytest.raises(ValueError):
        parse_topic_monitor_spec(raw_spec)


def test_topic_monitor_bridge_subscribes_and_records_json_snapshot() -> None:
    node = FakeNode()
    now_values = iter([1000, 1200])
    bridge = TopicMonitorBridge(node=node, now_ms=lambda: next(now_values))

    ok, detail = bridge.ensure_subscription("/debug/text", "std_msgs/msg/String")

    assert ok is True
    assert detail == "subscribed"
    assert len(node.subscriptions) == 1
    message_cls, topic, callback, qos = node.subscriptions[0]
    assert message_cls is String
    assert topic == "/debug/text"
    assert qos == 10

    msg = String()
    msg.data = "hello"
    callback(msg)

    assert bridge.get_snapshots() == [
        {
            "topic": "/debug/text",
            "message_type": "std_msgs/msg/String",
            "updated_at_ms": 1000,
            "revision": 1,
            "data": {"data": "hello"},
            "error": None,
        }
    ]


def test_topic_monitor_bridge_reuses_existing_subscription() -> None:
    node = FakeNode()
    bridge = TopicMonitorBridge(node=node, now_ms=lambda: 0)

    assert bridge.ensure_subscription("/debug/text", "std_msgs/msg/String")[0] is True
    assert bridge.ensure_subscription("/debug/text", "std_msgs/msg/String") == (
        True,
        "already subscribed",
    )
    assert len(node.subscriptions) == 1


def test_topic_monitor_bridge_rejects_unknown_message_type() -> None:
    node = FakeNode()
    bridge = TopicMonitorBridge(node=node, now_ms=lambda: 0)

    ok, detail = bridge.ensure_subscription("/debug/text", "missing/msg/Type")

    assert ok is False
    assert "unsupported ROS message type" in detail
    assert node.subscriptions == []


@pytest.mark.parametrize(
    "message_type",
    ["sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"],
)
def test_topic_monitor_bridge_rejects_image_message_types(message_type: str) -> None:
    node = FakeNode()
    bridge = TopicMonitorBridge(node=node, now_ms=lambda: 0)

    ok, detail = bridge.ensure_subscription("/camera/image_raw", message_type)

    assert ok is False
    assert "image/video topics are not supported by topic_monitor" in detail
    assert node.subscriptions == []


def test_to_jsonable_converts_binary_and_nested_sequences() -> None:
    assert to_jsonable({"payload": b"\x01\x02", "items": ({"x": 1},)}) == {
        "payload": [1, 2],
        "items": [{"x": 1}],
    }
