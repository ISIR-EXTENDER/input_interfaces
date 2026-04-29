from __future__ import annotations

import threading
from typing import Dict, List, Optional

from tablet_interface.measure_codec import is_legacy_fake_measure_vectors


class TabletRuntimeState:
    def __init__(
        self,
        *,
        default_mode: int,
        publish_rate_hz: float,
        gripper_open_position: float,
        gripper_close_position: float,
        measure_demo_vectors_json: str,
        measure_demo_image_data_url: str | None,
    ) -> None:
        self._lock = threading.Lock()
        self._connected: bool = False
        self._current_mode: int = int(default_mode)
        self._last_cmd_received_ms: Optional[int] = None
        self._last_seq: int = 0
        self._last_events: List[str] = []
        self._gripper_state: str = "unknown"
        self._gripper_open_position = float(gripper_open_position)
        self._gripper_close_position = float(gripper_close_position)
        self._measure_result_image_data_url: str | None = None
        self._measure_result_vectors_json: str | None = None
        self._measure_result_updated_at_ms: int | None = None
        self._measure_result_revision: int = 0
        self._measure_demo_vectors_json = measure_demo_vectors_json
        self._measure_demo_image_data_url = measure_demo_image_data_url
        self._ee_pose: Dict[str, float] | None = None
        self._tcp_speed_mps: float | None = None
        self._joint_positions: List[float] | None = None
        self._publish_rate_hz = float(publish_rate_hz)

    def update_command_meta(
        self,
        *,
        mode: int,
        seq: int,
        received_ms: int,
    ) -> None:
        with self._lock:
            self._current_mode = int(mode)
            self._last_cmd_received_ms = int(received_ms)
            self._last_seq = int(seq)

    def set_connected(self, connected: bool) -> None:
        with self._lock:
            self._connected = bool(connected)

    def set_gripper_action(self, action: str) -> None:
        with self._lock:
            self._gripper_state = action

    def update_gripper_position(self, position: float) -> None:
        open_distance = abs(float(position) - self._gripper_open_position)
        close_distance = abs(float(position) - self._gripper_close_position)
        state = "open" if open_distance <= close_distance else "close"
        with self._lock:
            self._gripper_state = state

    def update_measure_result_image(self, image_data_url: str, *, now_ms: int) -> None:
        with self._lock:
            self._measure_result_image_data_url = image_data_url
            self._measure_result_updated_at_ms = int(now_ms)
            self._measure_result_revision += 1

    def update_measure_result_vectors(self, vectors_json: str, *, now_ms: int) -> None:
        with self._lock:
            self._measure_result_vectors_json = vectors_json
            self._measure_result_updated_at_ms = int(now_ms)
            self._measure_result_revision += 1

    def get_measure_result_snapshot(self) -> Dict[str, object]:
        with self._lock:
            image_data_url = self._measure_result_image_data_url
            vectors_json = self._measure_result_vectors_json
            updated_at_ms = self._measure_result_updated_at_ms
            if (
                is_legacy_fake_measure_vectors(vectors_json)
                and self._measure_demo_image_data_url is not None
            ):
                image_data_url = self._measure_demo_image_data_url
                vectors_json = self._measure_demo_vectors_json
                updated_at_ms = None
            return {
                "revision": int(self._measure_result_revision),
                "image_data_url": image_data_url,
                "vectors_json": vectors_json,
                "updated_at_ms": updated_at_ms,
            }

    def update_ee_pose(self, *, x: float, y: float, z: float) -> None:
        with self._lock:
            self._ee_pose = {
                "x": float(x),
                "y": float(y),
                "z": float(z),
            }

    def update_tcp_speed(self, speed_mps: float) -> None:
        with self._lock:
            self._tcp_speed_mps = float(speed_mps)

    def update_joint_positions(self, positions: List[float]) -> None:
        with self._lock:
            self._joint_positions = [float(value) for value in positions]

    def get_state(self, *, now_ms: int) -> Dict[str, object]:
        with self._lock:
            cmd_age_ms = None
            if self._last_cmd_received_ms is not None:
                cmd_age_ms = int(now_ms) - int(self._last_cmd_received_ms)

            return {
                "connected": self._connected,
                "cmd_age_ms": cmd_age_ms,
                "watchdog_timeout_ms": 0,
                "last_seq": self._last_seq,
                "publishing_rate_hz": self._publish_rate_hz,
                "current_mode": int(self._current_mode),
                "gripper_state": self._gripper_state,
                "ee_pose": dict(self._ee_pose) if self._ee_pose is not None else None,
                "tcp_speed_mps": self._tcp_speed_mps,
                "joint_positions": (
                    list(self._joint_positions)
                    if self._joint_positions is not None
                    else None
                ),
                "events": list(self._last_events),
            }

    def get_current_mode(self) -> int:
        with self._lock:
            return int(self._current_mode)

    def clear_events(self) -> None:
        with self._lock:
            self._last_events = []
