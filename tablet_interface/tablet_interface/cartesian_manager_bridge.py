"""Publish tablet commands in the ``cartesian_manager`` contract.

``cartesian_manager`` replaces the old ``sandbox_controller`` path. It expects:

- ``geometry_msgs/msg/TwistStamped`` Cartesian velocity on an input topic, by
  default ``/joystick_cartesian_command``;
- ``std_msgs/msg/String`` mode requests on ``/mode_request``.

The frame matters. ``cartesian_manager`` performs no TF conversion: a command
whose ``header.frame_id`` is neither empty nor the configured
``default_input_frame_id`` is dropped, and the robot silently stops. This bridge
therefore stamps every command with a configured frame and logs it at startup.
"""

from __future__ import annotations

from typing import Callable, Optional, Protocol, Tuple

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String

from tablet_interface.mode_request import (
    is_one_shot_mode_request,
    validate_mode_request,
)
from tablet_interface.runtime_state import TabletRuntimeState


class _Publisher(Protocol):
    def publish(self, msg: object) -> None:
        ...


class _Logger(Protocol):
    def info(self, message: str) -> None:
        ...

    def warning(self, message: str) -> None:
        ...


class CartesianManagerBridge:
    """Adapt tablet teleop and UI mode actions to the ``cartesian_manager`` topics."""

    def __init__(
        self,
        *,
        logger: _Logger,
        command_publisher: _Publisher,
        mode_request_publisher: _Publisher,
        runtime_state: TabletRuntimeState,
        now_msg: Callable[[], Time],
        command_topic: str,
        mode_request_topic: str,
        command_frame_id: str,
    ) -> None:
        self._logger = logger
        self._command_publisher = command_publisher
        self._mode_request_publisher = mode_request_publisher
        self._runtime_state = runtime_state
        self._now_msg = now_msg
        self._command_topic = command_topic
        self._mode_request_topic = mode_request_topic
        self._command_frame_id = command_frame_id.strip()

    @property
    def command_frame_id(self) -> str:
        return self._command_frame_id

    def build_command(self, twist: Twist) -> TwistStamped:
        """Wrap a mapped twist in a stamped, framed command."""
        msg = TwistStamped()
        msg.header.stamp = self._now_msg()
        msg.header.frame_id = self._command_frame_id
        msg.twist.linear.x = float(twist.linear.x)
        msg.twist.linear.y = float(twist.linear.y)
        msg.twist.linear.z = float(twist.linear.z)
        msg.twist.angular.x = float(twist.angular.x)
        msg.twist.angular.y = float(twist.angular.y)
        msg.twist.angular.z = float(twist.angular.z)
        return msg

    def publish_command(self, twist: Twist) -> None:
        self._command_publisher.publish(self.build_command(twist))

    def request_mode(self, raw_mode: str) -> Tuple[bool, str]:
        """Validate and publish a mode request.

        Returns ``(ok, detail)``. An invalid request is rejected here rather than
        published, because ``cartesian_manager`` drops unparseable modes without
        any feedback the tablet could surface to the operator.
        """
        ok, normalized, detail = validate_mode_request(raw_mode)
        if not ok:
            self._logger.warning(
                "Rejected mode request '{0}': {1}".format(raw_mode, detail)
            )
            return False, detail

        msg = String()
        msg.data = normalized
        self._mode_request_publisher.publish(msg)

        if is_one_shot_mode_request(normalized):
            # The manager dispatches the joint target once and returns to
            # passthrough by itself, so no sticky state is recorded here.
            self._runtime_state.set_last_mode_request(None)
        else:
            self._runtime_state.set_last_mode_request(normalized)

        self._logger.info(
            "Published mode request: topic={0} mode={1} ({2})".format(
                self._mode_request_topic,
                normalized,
                detail,
            )
        )
        return True, normalized

    def describe(self) -> str:
        return (
            "cartesian_manager bridge: command_topic={0} frame_id={1} "
            "mode_request_topic={2}".format(
                self._command_topic,
                self._command_frame_id or "<empty, manager default>",
                self._mode_request_topic,
            )
        )


def resolve_mode_request_topic_alias(
    topic: str,
    *,
    mode_request_topic: str,
) -> Optional[str]:
    """Return the mode-request topic when ``topic`` addresses it.

    UI widgets reach ROS through generic ``ui_typed``/``ui_button`` messages that
    carry a topic name. Routing those through the bridge, instead of a raw
    publisher, keeps mode validation in one place.
    """
    normalized = topic.strip()
    if not normalized:
        return None
    return mode_request_topic if normalized == mode_request_topic.strip() else None


__all__ = ["CartesianManagerBridge", "resolve_mode_request_topic_alias"]
