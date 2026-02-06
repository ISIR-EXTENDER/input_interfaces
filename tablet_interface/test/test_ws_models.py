import pytest
from pydantic import ValidationError

from tablet_interface.ws_models import CmdMessage, EventMessage, StateMessage


def test_cmd_message_valid() -> None:
    payload = {
        "type": "teleop_cmd",
        "seq": 12,
        "mode": 2,
        "linear": {"x": 0.1, "y": 0.2, "z": 0.0},
        "angular": {"x": -0.1, "y": 0.0, "z": 0.3},
    }
    msg = CmdMessage.model_validate(payload)
    assert msg.mode == 2
    assert msg.linear.x == 0.1


@pytest.mark.parametrize(
    "payload",
    [
        {  # invalid mode
            "type": "teleop_cmd",
            "seq": 1,
            "mode": 4,
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        {  # missing linear field
            "type": "teleop_cmd",
            "seq": 1,
            "mode": 0,
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        {  # wrong types
            "type": "teleop_cmd",
            "seq": "1",
            "mode": 0,
            "linear": {"x": "0", "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
    ],
)
def test_cmd_message_invalid(payload: dict) -> None:
    with pytest.raises(ValidationError):
        CmdMessage.model_validate(payload)


def test_state_message_valid() -> None:
    payload = {
        "type": "state",
        "connected": True,
        "cmd_age_ms": 20,
        "watchdog_timeout_ms": 200,
        "last_seq": 10,
        "publishing_rate_hz": 30.0,
        "current_mode": 1,
    }
    msg = StateMessage.model_validate(payload)
    assert msg.current_mode == 1


@pytest.mark.parametrize(
    "payload",
    [
        {  # invalid mode
            "type": "state",
            "connected": True,
            "cmd_age_ms": 20,
            "watchdog_timeout_ms": 200,
            "last_seq": 10,
            "publishing_rate_hz": 30.0,
            "current_mode": 9,
        },
        {  # invalid publishing_rate_hz type
            "type": "state",
            "connected": True,
            "cmd_age_ms": 20,
            "watchdog_timeout_ms": 200,
            "last_seq": 10,
            "publishing_rate_hz": "30.0",
            "current_mode": 1,
        },
    ],
)
def test_state_message_invalid(payload: dict) -> None:
    with pytest.raises(ValidationError):
        StateMessage.model_validate(payload)


def test_event_message_valid() -> None:
    payload = {
        "type": "event",
        "severity": "warning",
        "code": "W001",
        "message": "Low battery",
    }
    msg = EventMessage.model_validate(payload)
    assert msg.severity == "warning"


@pytest.mark.parametrize(
    "payload",
    [
        {"type": "event", "severity": "debug", "code": "W001", "message": "x"},
        {"type": "event", "severity": "info", "code": "", "message": "x"},
        {"type": "event", "severity": "info", "code": "W001", "message": ""},
    ],
)
def test_event_message_invalid(payload: dict) -> None:
    with pytest.raises(ValidationError):
        EventMessage.model_validate(payload)
