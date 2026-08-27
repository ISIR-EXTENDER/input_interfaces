from __future__ import annotations

from queue import SimpleQueue
from threading import Lock
from typing import Dict, List, Optional, Set, Tuple

from pynput import keyboard
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class KeyboardInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_interface_node")

        self.declare_parameter("key_topic_mappings", ["space=/keyboard/space"])
        self.declare_parameter("publish_qos_depth", 10)
        self.declare_parameter("publish_period_sec", 0.01)

        raw_mappings = list(self.get_parameter("key_topic_mappings").value)
        publish_qos_depth = int(self.get_parameter("publish_qos_depth").value)
        publish_period_sec = float(self.get_parameter("publish_period_sec").value)

        self._configured_topics = self._parse_key_topic_mappings(raw_mappings)
        self._bool_publishers = {
            key_name: self.create_publisher(Bool, topic_name, publish_qos_depth)
            for key_name, topic_name in self._configured_topics.items()
        }
        self._pending_presses: SimpleQueue[str] = SimpleQueue()
        self._active_keys: Set[str] = set()
        self._active_keys_lock = Lock()
        self._listener: Optional[keyboard.Listener] = None
        self._publish_timer = self.create_timer(publish_period_sec, self._flush_pending_presses)

        self.get_logger().info(
            f"Configured {len(self._configured_topics)} keyboard mappings."
        )

    def start_listener(self) -> None:
        if self._listener is not None:
            return

        self._listener = keyboard.Listener(
            on_press=self._handle_key_press,
            on_release=self._handle_key_release,
        )
        self._listener.start()

    def stop_listener(self) -> None:
        if self._listener is None:
            return

        self._listener.stop()
        self._listener = None

    def _handle_key_press(self, key: keyboard.Key | keyboard.KeyCode) -> None:
        key_name = self._normalize_key(key)
        if key_name is None or key_name not in self._bool_publishers:
            return

        with self._active_keys_lock:
            if key_name in self._active_keys:
                return
            self._active_keys.add(key_name)

        self._pending_presses.put(key_name)

    def _handle_key_release(self, key: keyboard.Key | keyboard.KeyCode) -> None:
        key_name = self._normalize_key(key)
        if key_name is None:
            return

        with self._active_keys_lock:
            self._active_keys.discard(key_name)

    def _flush_pending_presses(self) -> None:
        while not self._pending_presses.empty():
            key_name = self._pending_presses.get()
            publisher = self._bool_publishers.get(key_name)
            if publisher is None:
                continue

            publisher.publish(Bool(data=True))

    def _parse_key_topic_mappings(self, raw_mappings: List[str]) -> Dict[str, str]:
        mappings: Dict[str, str] = {}

        for raw_mapping in raw_mappings:
            key_name, topic_name = self._split_mapping(raw_mapping)
            normalized_key = self._normalize_config_key(key_name)

            if normalized_key in mappings:
                raise ValueError(f"Duplicate key mapping for '{normalized_key}'.")

            mappings[normalized_key] = topic_name

        if not mappings:
            raise ValueError("At least one key/topic mapping must be configured.")

        return mappings

    def _split_mapping(self, raw_mapping: str) -> Tuple[str, str]:
        if "=" not in raw_mapping:
            raise ValueError(
                "Invalid key_topic_mappings entry "
                f"'{raw_mapping}'. Expected format 'key=/topic_name'."
            )

        key_name, topic_name = raw_mapping.split("=", 1)
        key_name = key_name.strip()
        topic_name = topic_name.strip()

        if not key_name:
            raise ValueError(f"Invalid mapping '{raw_mapping}': key cannot be empty.")
        if not topic_name:
            raise ValueError(f"Invalid mapping '{raw_mapping}': topic cannot be empty.")

        return key_name, topic_name

    def _normalize_config_key(self, key_name: str) -> str:
        return key_name.strip().lower()

    def _normalize_key(self, key: keyboard.Key | keyboard.KeyCode) -> Optional[str]:
        if isinstance(key, keyboard.KeyCode):
            if key.char is None:
                return None
            return key.char.lower()

        return key.name.lower() if key.name is not None else None