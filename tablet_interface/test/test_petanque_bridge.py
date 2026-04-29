from __future__ import annotations

from types import SimpleNamespace

from rcl_interfaces.msg import SetParametersResult

from tablet_interface.petanque_bridge import PetanqueBridge


class FakeLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def publish(self, msg) -> None:
        self.messages.append(msg.data)


class FakeFuture:
    def __init__(self, result) -> None:
        self._result = result

    def add_done_callback(self, callback) -> None:
        callback(self)

    def result(self):
        return self._result


class FakeParamClient:
    def __init__(self, *, service_available: bool = True, call_result=None) -> None:
        self.service_available = service_available
        self.call_result = call_result
        self.requests = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self.service_available

    def call_async(self, req):
        self.requests.append(req)
        return FakeFuture(self.call_result)


def create_bridge(
    *,
    service_available: bool = True,
    call_result=None,
) -> tuple[PetanqueBridge, FakeLogger, FakePublisher, FakeParamClient]:
    logger = FakeLogger()
    publisher = FakePublisher()
    client = FakeParamClient(
        service_available=service_available,
        call_result=call_result,
    )
    bridge = PetanqueBridge(
        logger=logger,
        state_cmd_publisher=publisher,
        param_client=client,
        param_service="/petanque_throw/set_parameters",
        total_duration_param="total_duration",
        angle_between_start_and_finish_param="angle_between_start_and_finish",
        alpha_param="alpha",
        param_call_timeout_sec=0.01,
    )
    return bridge, logger, publisher, client


def test_send_state_command_normalizes_and_publishes() -> None:
    bridge, logger, publisher, _ = create_bridge()

    assert bridge.send_state_command(" Throw ") is True
    assert publisher.messages == ["throw"]
    assert logger.infos == ["Published state machine command: throw"]


def test_send_state_command_rejects_unknown_values() -> None:
    bridge, logger, publisher, _ = create_bridge()

    assert bridge.send_state_command("dance") is False
    assert publisher.messages == []
    assert logger.warnings == ["Invalid state machine command: dance"]


def test_set_total_duration_validates_positive_values() -> None:
    bridge, logger, _, client = create_bridge()

    assert bridge.set_total_duration(0.0) is False
    assert client.requests == []
    assert logger.warnings == ["Invalid total_duration=0.000; expected > 0"]


def test_set_alpha_validates_expected_range() -> None:
    bridge, logger, _, client = create_bridge()

    assert bridge.set_alpha(41.0) is False
    assert client.requests == []
    assert logger.warnings == ["Invalid alpha=41.000; expected in [0, 40]"]


def test_set_angle_between_start_and_finish_calls_parameter_service() -> None:
    result = SimpleNamespace(
        results=[SetParametersResult(successful=True, reason="")]
    )
    bridge, logger, _, client = create_bridge(call_result=result)

    assert bridge.set_angle_between_start_and_finish(17.5) is True
    assert len(client.requests) == 1
    request = client.requests[0]
    assert request.parameters[0].name == "angle_between_start_and_finish"
    assert request.parameters[0].value.double_value == 17.5
    assert logger.infos == ["Updated angle_between_start_and_finish=17.500"]


def test_set_parameter_reports_service_unavailability() -> None:
    bridge, logger, _, client = create_bridge(service_available=False)

    assert bridge.set_angle_between_start_and_finish(17.5) is False
    assert client.requests == []
    assert logger.warnings == ["Service unavailable: /petanque_throw/set_parameters"]

