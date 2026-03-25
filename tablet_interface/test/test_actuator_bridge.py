from __future__ import annotations

from std_msgs.msg import Float64MultiArray

from tablet_interface.actuator_bridge import ActuatorBridge
from tablet_interface.runtime_state import TabletRuntimeState


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
        self.messages: list[object] = []

    def publish(self, msg: object) -> None:
        self.messages.append(msg)


def create_runtime_state() -> TabletRuntimeState:
    return TabletRuntimeState(
        default_mode=2,
        publish_rate_hz=60.0,
        gripper_open_position=0.2,
        gripper_close_position=1.1,
        measure_demo_vectors_json='{"source":"demo"}',
        measure_demo_image_data_url="data:image/png;base64,demo",
    )


def create_bridge() -> tuple[ActuatorBridge, FakeLogger, FakePublisher, FakePublisher, TabletRuntimeState]:
    logger = FakeLogger()
    gripper_publisher = FakePublisher()
    hub_publisher = FakePublisher()
    runtime_state = create_runtime_state()
    bridge = ActuatorBridge(
        logger=logger,
        gripper_publisher=gripper_publisher,
        hub_digital_output_publisher=hub_publisher,
        runtime_state=runtime_state,
        gripper_topic="/gripper_controller/commands",
        gripper_open_position=0.2,
        gripper_close_position=1.1,
        hub_digital_output_topic="/hub/digital_output",
        hub_electromagnet_channel=2.0,
    )
    return bridge, logger, gripper_publisher, hub_publisher, runtime_state


def test_set_gripper_publishes_expected_position_and_updates_state() -> None:
    bridge, logger, gripper_publisher, _, runtime_state = create_bridge()

    assert bridge.set_gripper("open") is True

    assert len(gripper_publisher.messages) == 1
    assert list(gripper_publisher.messages[0].data) == [0.2]
    assert runtime_state.get_state(now_ms=0)["gripper_state"] == "open"
    assert logger.infos == [
        "Published gripper command: action=open topic=/gripper_controller/commands value=0.200"
    ]


def test_set_gripper_rejects_invalid_action() -> None:
    bridge, logger, gripper_publisher, _, _ = create_bridge()

    assert bridge.set_gripper("toggle") is False

    assert gripper_publisher.messages == []
    assert logger.warnings == ["Invalid gripper action: toggle"]


def test_set_electromagnet_uses_active_low_wiring() -> None:
    bridge, logger, _, hub_publisher, _ = create_bridge()

    assert bridge.set_electromagnet(True) is True

    assert len(hub_publisher.messages) == 1
    assert list(hub_publisher.messages[0].data) == [2.0, 0.0]
    assert logger.infos == [
        "Published hub digital output: topic=/hub/digital_output channel=2.0 value=0.0"
    ]


def test_on_gripper_command_updates_runtime_state_from_feedback_position() -> None:
    bridge, _, _, _, runtime_state = create_bridge()
    msg = Float64MultiArray()
    msg.data = [1.1]

    bridge.on_gripper_command(msg)

    assert runtime_state.get_state(now_ms=0)["gripper_state"] == "close"
