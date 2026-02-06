from __future__ import annotations

import threading
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from extender_msgs.msg import TeleopCommand

from tablet_interface.safety_gate import SafetyGate, TeleopCmd, Twist


class TabletInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("tablet_interface_node")

        self.declare_parameter("teleop_cmd_topic", "/teleop_cmd")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("watchdog_timeout_ms", 200)
        self.declare_parameter("max_linear_mps", 0.2)
        self.declare_parameter("max_linear_z_mps", 0.2)
        self.declare_parameter("max_angular_rps", 0.5)
        self.declare_parameter("linear_scale", 0.2)
        self.declare_parameter("angular_scale", 0.5)
        self.declare_parameter("z_scale", 0.2)
        self.declare_parameter("default_mode", 0)
        self.declare_parameter("accept_mode_from_client", True)
        self.declare_parameter("state_publish_hz", 5.0)
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("bind_port", 8765)
        self.declare_parameter("ws_path", "/ws/control")

        self.teleop_cmd_topic = self.get_parameter("teleop_cmd_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.watchdog_timeout_ms = int(self.get_parameter("watchdog_timeout_ms").value)
        self.max_linear_mps = float(self.get_parameter("max_linear_mps").value)
        self.max_linear_z_mps = float(self.get_parameter("max_linear_z_mps").value)
        self.max_angular_rps = float(self.get_parameter("max_angular_rps").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.z_scale = float(self.get_parameter("z_scale").value)
        self.default_mode = int(self.get_parameter("default_mode").value)
        self.accept_mode_from_client = bool(self.get_parameter("accept_mode_from_client").value)
        self.state_publish_hz = float(self.get_parameter("state_publish_hz").value)
        self.bind_host = str(self.get_parameter("bind_host").value)
        self.bind_port = int(self.get_parameter("bind_port").value)
        self.ws_path = str(self.get_parameter("ws_path").value)

        self._gate = SafetyGate(
            watchdog_timeout_ms=self.watchdog_timeout_ms,
            max_linear_mps=self.max_linear_mps,
            max_linear_z_mps=self.max_linear_z_mps,
            max_angular_rps=self.max_angular_rps,
            default_mode=self.default_mode,
        )

        self._lock = threading.Lock()
        self._last_cmd_received_ms: Optional[int] = None
        self._last_seq: int = 0
        self._connected: bool = False
        self._last_events: List[str] = []

        self._publisher = self.create_publisher(TeleopCommand, self.teleop_cmd_topic, 10)
        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info("Tablet interface node initialized")
        self.get_logger().info(
            "Safety params: watchdog_timeout_ms={0} max_linear_mps={1:.3f} "
            "max_linear_z_mps={2:.3f} max_angular_rps={3:.3f} default_mode={4}".format(
                self.watchdog_timeout_ms,
                self.max_linear_mps,
                self.max_linear_z_mps,
                self.max_angular_rps,
                self.default_mode,
            )
        )
        self.get_logger().info(
            "WS params: bind_host={0} bind_port={1} ws_path={2} state_publish_hz={3:.1f}".format(
                self.bind_host,
                self.bind_port,
                self.ws_path,
                self.state_publish_hz,
            )
        )
        self.get_logger().info(
            "Scale params: linear_scale={0:.3f} z_scale={1:.3f} angular_scale={2:.3f}".format(
                self.linear_scale,
                self.z_scale,
                self.angular_scale,
            )
        )
        self.get_logger().info(
            "Teleop params: topic={0} publish_rate_hz={1:.1f} accept_mode_from_client={2}".format(
                self.teleop_cmd_topic,
                self.publish_rate_hz,
                str(self.accept_mode_from_client).lower(),
            )
        )

    def update_latest_cmd(
        self,
        *,
        twist: Twist,
        mode: int,
        seq: int,
        received_ms: Optional[int] = None,
    ) -> None:
        now_ms = self._now_ms()
        if received_ms is None:
            received_ms = now_ms

        if not self.accept_mode_from_client:
            mode = self.default_mode

        cmd = TeleopCmd(
            twist=twist,
            mode=int(mode),
            seq=int(seq),
            received_ms=int(received_ms),
        )

        with self._lock:
            self._gate.update_cmd(cmd)
            self._last_cmd_received_ms = int(received_ms)
            self._last_seq = int(seq)

    def set_connected(self, connected: bool) -> None:
        with self._lock:
            self._connected = bool(connected)

    def get_state(self) -> Dict[str, object]:
        with self._lock:
            now_ms = self._now_ms()
            cmd_age_ms = None
            if self._last_cmd_received_ms is not None:
                cmd_age_ms = int(now_ms) - int(self._last_cmd_received_ms)

            return {
                "connected": self._connected,
                "cmd_age_ms": cmd_age_ms,
                "watchdog_timeout_ms": self.watchdog_timeout_ms,
                "last_seq": self._last_seq,
                "publishing_rate_hz": float(self.publish_rate_hz),
                "current_mode": int(self._gate._last_mode),
                "events": list(self._last_events),
            }

    def _on_timer(self) -> None:
        now_ms = self._now_ms()
        with self._lock:
            twist, mode, events = self._gate.process(now_ms=now_ms)
            self._last_events = list(events)

        msg = TeleopCommand()
        msg.twist = twist
        msg.mode = int(mode)
        self._publisher.publish(msg)

    def _now_ms(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1_000_000)


__all__ = ["TabletInterfaceNode"]
