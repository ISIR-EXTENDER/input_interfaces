from __future__ import annotations

from typing import Protocol

from std_msgs.msg import Float32MultiArray, Float64MultiArray

from tablet_interface.runtime_state import TabletRuntimeState


class LoggerLike(Protocol):
    def info(self, message: str) -> None:
        ...

    def warning(self, message: str) -> None:
        ...


class Float64MultiArrayPublisherLike(Protocol):
    def publish(self, msg: Float64MultiArray) -> None:
        ...


class Float32MultiArrayPublisherLike(Protocol):
    def publish(self, msg: Float32MultiArray) -> None:
        ...


class ActuatorBridge:
    def __init__(
        self,
        *,
        logger: LoggerLike,
        gripper_publisher: Float64MultiArrayPublisherLike,
        hub_digital_output_publisher: Float32MultiArrayPublisherLike,
        runtime_state: TabletRuntimeState,
        gripper_topic: str,
        gripper_open_position: float,
        gripper_close_position: float,
        hub_digital_output_topic: str,
        hub_electromagnet_channel: float,
    ) -> None:
        self._logger = logger
        self._gripper_publisher = gripper_publisher
        self._hub_digital_output_publisher = hub_digital_output_publisher
        self._runtime_state = runtime_state
        self._gripper_topic = gripper_topic
        self._gripper_open_position = float(gripper_open_position)
        self._gripper_close_position = float(gripper_close_position)
        self._hub_digital_output_topic = hub_digital_output_topic
        self._hub_electromagnet_channel = float(hub_electromagnet_channel)

    def set_gripper(self, action: str) -> bool:
        normalized = action.strip().lower()
        if normalized not in {"open", "close"}:
            self._logger.warning(f"Invalid gripper action: {action}")
            return False

        position = (
            self._gripper_open_position
            if normalized == "open"
            else self._gripper_close_position
        )
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self._gripper_publisher.publish(msg)
        self._runtime_state.set_gripper_action(normalized)
        self._logger.info(
            "Published gripper command: action={0} topic={1} value={2:.3f}".format(
                normalized,
                self._gripper_topic,
                position,
            )
        )
        return True

    def set_electromagnet(self, enabled: bool) -> bool:
        msg = Float32MultiArray()
        msg.data = [
            self._hub_electromagnet_channel,
            0.0 if enabled else 1.0,
        ]
        self._hub_digital_output_publisher.publish(msg)
        self._logger.info(
            "Published hub digital output: topic={0} channel={1:.1f} value={2:.1f}".format(
                self._hub_digital_output_topic,
                self._hub_electromagnet_channel,
                msg.data[1],
            )
        )
        return True

    def on_gripper_command(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        self._runtime_state.update_gripper_position(float(msg.data[0]))

