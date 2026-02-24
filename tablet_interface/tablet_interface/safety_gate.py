from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

try:
    from geometry_msgs.msg import Twist  # type: ignore
except Exception:  # pragma: no cover - fallback for test environments without ROS
    from dataclasses import dataclass as _dataclass

    @_dataclass
    class _Vec3:
        x: float = 0.0
        y: float = 0.0
        z: float = 0.0

    @_dataclass
    class Twist:  # minimal stand-in for geometry_msgs.msg.Twist
        linear: _Vec3 = _Vec3()
        angular: _Vec3 = _Vec3()


@dataclass
class TeleopCmd:
    twist: Twist
    mode: int
    seq: int
    received_ms: int


class SafetyGate:
    def __init__(
        self,
        *,
        watchdog_timeout_ms: int,
        max_linear_mps: float,
        max_linear_z_mps: float,
        max_angular_rps: float,
        default_mode: int = 0,
    ) -> None:
        self.watchdog_timeout_ms = int(watchdog_timeout_ms)
        self.max_linear_mps = float(max_linear_mps)
        self.max_linear_z_mps = float(max_linear_z_mps)
        self.max_angular_rps = float(max_angular_rps)
        self.default_mode = int(default_mode)

        self._last_cmd: Optional[TeleopCmd] = None
        self._last_mode: int = self.default_mode

    def update_cmd(self, cmd: TeleopCmd) -> None:
        self._last_cmd = cmd
        self._last_mode = int(cmd.mode)

    def process(self, now_ms: int) -> Tuple[Twist, int, List[str]]:
        events: List[str] = []
        mode = self._last_mode

        if self._last_cmd is None:
            return Twist(), mode, events

        age_ms = int(now_ms) - int(self._last_cmd.received_ms)
        if age_ms > self.watchdog_timeout_ms:
            events.append("watchdog_timeout")
            return Twist(), mode, events

        return self._saturate_twist(self._last_cmd.twist), mode, events

    def _saturate_twist(self, twist: Twist) -> Twist:
        out = Twist()
        out.linear.x = self._clamp(twist.linear.x, self.max_linear_mps)
        out.linear.y = self._clamp(twist.linear.y, self.max_linear_mps)
        out.linear.z = self._clamp(twist.linear.z, self.max_linear_z_mps)
        out.angular.x = self._clamp(twist.angular.x, self.max_angular_rps)
        out.angular.y = self._clamp(twist.angular.y, self.max_angular_rps)
        out.angular.z = self._clamp(twist.angular.z, self.max_angular_rps)
        return out

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value
