from __future__ import annotations

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist

from tablet_interface.cartesian_manager_bridge import CartesianManagerBridge
from tablet_interface.runtime_state import TabletRuntimeState


class RecordingPublisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, msg: object) -> None:
        self.messages.append(msg)


class RecordingLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)


def create_runtime_state() -> TabletRuntimeState:
    return TabletRuntimeState(
        default_mode=0,
        publish_rate_hz=30.0,
        gripper_open_position=0.2,
        gripper_close_position=1.1,
        measure_demo_vectors_json='{"source":"demo"}',
        measure_demo_image_data_url="data:image/png;base64,demo",
    )


def create_bridge(
    *,
    command_frame_id: str = "base_link",
) -> tuple[CartesianManagerBridge, RecordingPublisher, RecordingPublisher, RecordingLogger, TabletRuntimeState]:
    command_publisher = RecordingPublisher()
    mode_publisher = RecordingPublisher()
    logger = RecordingLogger()
    runtime_state = create_runtime_state()
    bridge = CartesianManagerBridge(
        logger=logger,
        command_publisher=command_publisher,
        mode_request_publisher=mode_publisher,
        runtime_state=runtime_state,
        now_msg=lambda: Time(sec=7, nanosec=8),
        command_topic="/joystick_cartesian_command",
        mode_request_topic="/mode_request",
        command_frame_id=command_frame_id,
    )
    return bridge, command_publisher, mode_publisher, logger, runtime_state


def make_twist() -> Twist:
    twist = Twist()
    twist.linear.x = 0.1
    twist.linear.y = -0.2
    twist.linear.z = 0.3
    twist.angular.x = 0.4
    twist.angular.y = -0.5
    twist.angular.z = 0.6
    return twist


def test_publish_command_stamps_configured_frame() -> None:
    bridge, command_publisher, _mode, _logger, _state = create_bridge()

    bridge.publish_command(make_twist())

    assert len(command_publisher.messages) == 1
    msg = command_publisher.messages[0]
    # A mismatched frame makes cartesian_manager drop the command silently,
    # so the stamped frame is the single most important field here.
    assert msg.header.frame_id == "base_link"
    assert msg.header.stamp.sec == 7
    assert msg.twist.linear.x == 0.1
    assert msg.twist.linear.y == -0.2
    assert msg.twist.linear.z == 0.3
    assert msg.twist.angular.x == 0.4
    assert msg.twist.angular.y == -0.5
    assert msg.twist.angular.z == 0.6


def test_publish_command_supports_empty_frame() -> None:
    # An empty frame_id is treated by the manager as its default input frame.
    bridge, command_publisher, _mode, _logger, _state = create_bridge(
        command_frame_id="  "
    )

    bridge.publish_command(make_twist())

    assert command_publisher.messages[0].header.frame_id == ""


def test_request_mode_publishes_normalized_string() -> None:
    bridge, _cmd, mode_publisher, _logger, runtime_state = create_bridge()

    ok, detail = bridge.request_mode("GEOMETRIC/Snake")

    assert ok is True
    assert detail == "geometric/snake"
    assert [msg.data for msg in mode_publisher.messages] == ["geometric/snake"]
    assert runtime_state.get_state(now_ms=0)["mode_request"] == "geometric/snake"


def test_request_mode_rejects_invalid_mode_without_publishing() -> None:
    bridge, _cmd, mode_publisher, logger, runtime_state = create_bridge()

    ok, detail = bridge.request_mode("geometric/spiral")

    assert ok is False
    assert "unknown geometric mode" in detail
    assert mode_publisher.messages == []
    assert logger.warnings
    assert runtime_state.get_state(now_ms=0)["mode_request"] is None


def test_request_mode_does_not_latch_one_shot_joint_targets() -> None:
    bridge, _cmd, mode_publisher, _logger, runtime_state = create_bridge()

    bridge.request_mode("geometric/snake")
    ok, _detail = bridge.request_mode("behaviour/joint_target/home")

    assert ok is True
    assert [msg.data for msg in mode_publisher.messages] == [
        "geometric/snake",
        "behaviour/joint_target/home",
    ]
    # The manager returns to passthrough on its own after dispatching the target,
    # so the tablet must not keep showing a joint-target mode.
    assert runtime_state.get_state(now_ms=0)["mode_request"] is None
