from __future__ import annotations

import threading
from typing import Protocol

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String


class LoggerLike(Protocol):
    def info(self, message: str) -> None:
        ...

    def warning(self, message: str) -> None:
        ...


class StringPublisherLike(Protocol):
    def publish(self, msg: String) -> None:
        ...


class SetParametersClientLike(Protocol):
    def wait_for_service(self, timeout_sec: float) -> bool:
        ...

    def call_async(self, req: SetParameters.Request):  # pragma: no cover - protocol
        ...


class PetanqueBridge:
    _VALID_STATE_COMMANDS = {
        "teleop",
        "activate_throw",
        "go_to_start",
        "throw",
        "pick_up",
        "stop",
        "test_loop",
    }

    def __init__(
        self,
        *,
        logger: LoggerLike,
        state_cmd_publisher: StringPublisherLike,
        param_client: SetParametersClientLike,
        param_service: str,
        total_duration_param: str,
        angle_between_start_and_finish_param: str,
        alpha_param: str,
        param_call_timeout_sec: float,
    ) -> None:
        self._logger = logger
        self._state_cmd_publisher = state_cmd_publisher
        self._param_client = param_client
        self._param_service = param_service
        self._total_duration_param = total_duration_param
        self._angle_between_start_and_finish_param = (
            angle_between_start_and_finish_param
        )
        self._alpha_param = alpha_param
        self._param_call_timeout_sec = param_call_timeout_sec

    def send_state_command(self, command: str) -> bool:
        normalized = command.strip().lower()
        if normalized not in self._VALID_STATE_COMMANDS:
            self._logger.warning(f"Invalid state machine command: {command}")
            return False

        msg = String()
        msg.data = normalized
        self._state_cmd_publisher.publish(msg)
        self._logger.info(f"Published state machine command: {normalized}")
        return True

    def set_total_duration(self, total_duration: float) -> bool:
        if total_duration <= 0.0:
            self._logger.warning(
                f"Invalid total_duration={total_duration:.3f}; expected > 0"
            )
            return False

        return self._set_double_parameter(
            parameter_name=self._total_duration_param,
            value=float(total_duration),
        )

    def set_angle_between_start_and_finish(self, angle: float) -> bool:
        return self._set_double_parameter(
            parameter_name=self._angle_between_start_and_finish_param,
            value=float(angle),
        )

    def set_alpha(self, alpha: float) -> bool:
        if alpha < 0.0 or alpha > 40.0:
            self._logger.warning(
                f"Invalid alpha={alpha:.3f}; expected in [0, 40]"
            )
            return False

        return self._set_double_parameter(
            parameter_name=self._alpha_param,
            value=float(alpha),
        )

    def _set_double_parameter(self, *, parameter_name: str, value: float) -> bool:
        if not parameter_name:
            self._logger.warning("Petanque parameter name is empty")
            return False

        if not self._param_client.wait_for_service(
            timeout_sec=self._param_call_timeout_sec
        ):
            self._logger.warning(f"Service unavailable: {self._param_service}")
            return False

        param = Parameter(
            name=parameter_name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value),
            ),
        )
        req = SetParameters.Request(parameters=[param])
        future = self._param_client.call_async(req)

        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self._param_call_timeout_sec):
            self._logger.warning(
                f"Timeout while setting parameter {parameter_name}"
            )
            return False

        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover
            self._logger.warning(f"SetParameters call failed: {exc}")
            return False

        if not result or not result.results:
            self._logger.warning("SetParameters returned empty result")
            return False

        if not result.results[0].successful:
            reason = result.results[0].reason or "unknown error"
            self._logger.warning(f"Failed to set {parameter_name}: {reason}")
            return False

        self._logger.info(f"Updated {parameter_name}={value:.3f}")
        return True

