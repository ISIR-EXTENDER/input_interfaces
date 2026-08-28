from __future__ import annotations

from queue import SimpleQueue
from threading import Lock
from typing import Optional

from keyboard_interface.mappings import parse_key_topic_mappings
from pynput import keyboard
from rclpy.node import Node
from std_msgs.msg import Bool


class KeyboardInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_interface_node')

        self.declare_parameter('key_topic_mappings', ['space=/keyboard/space'])
        self.declare_parameter('publish_qos_depth', 10)
        self.declare_parameter('publish_period_sec', 0.01)

        raw_mappings = list(self.get_parameter('key_topic_mappings').value)
        publish_qos_depth = int(self.get_parameter('publish_qos_depth').value)
        publish_period_sec = float(self.get_parameter('publish_period_sec').value)

        self._configured_topics = parse_key_topic_mappings(raw_mappings)
        self._bool_publishers = {
            key_name: self.create_publisher(Bool, topic_name, publish_qos_depth)
            for key_name, topic_name in self._configured_topics.items()
        }
        self._pending_key_states: SimpleQueue[tuple[str, bool]] = SimpleQueue()
        self._active_keys: set[str] = set()
        self._active_keys_lock = Lock()
        self._listener: Optional[keyboard.Listener] = None
        self._publish_timer = self.create_timer(
            publish_period_sec,
            self._flush_pending_key_states,
        )

        self.get_logger().info(
            f'Configured {len(self._configured_topics)} keyboard mappings.'
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

        self._pending_key_states.put((key_name, True))

    def _handle_key_release(self, key: keyboard.Key | keyboard.KeyCode) -> None:
        key_name = self._normalize_key(key)
        if key_name is None:
            return

        with self._active_keys_lock:
            was_active = key_name in self._active_keys
            self._active_keys.discard(key_name)

        if was_active:
            self._pending_key_states.put((key_name, False))

    def _flush_pending_key_states(self) -> None:
        while not self._pending_key_states.empty():
            key_name, is_pressed = self._pending_key_states.get()
            publisher = self._bool_publishers.get(key_name)
            if publisher is None:
                continue

            publisher.publish(Bool(data=is_pressed))

    def _normalize_key(self, key: keyboard.Key | keyboard.KeyCode) -> Optional[str]:
        if isinstance(key, keyboard.KeyCode):
            if key.char is None:
                return None
            return key.char.lower()

        return key.name.lower() if key.name is not None else None
