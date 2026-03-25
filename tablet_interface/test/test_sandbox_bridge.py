from __future__ import annotations

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64MultiArray

from tablet_interface.runtime_state import TabletRuntimeState
from tablet_interface.sandbox_bridge import SandboxBridge


def create_runtime_state() -> TabletRuntimeState:
    return TabletRuntimeState(
        default_mode=2,
        publish_rate_hz=60.0,
        gripper_open_position=0.2,
        gripper_close_position=1.1,
        measure_demo_vectors_json='{"source":"demo"}',
        measure_demo_image_data_url="data:image/png;base64,demo",
    )


def test_sandbox_bridge_updates_ee_pose() -> None:
    runtime_state = create_runtime_state()
    bridge = SandboxBridge(runtime_state=runtime_state)
    msg = PoseStamped()
    msg.pose.position.x = 0.1
    msg.pose.position.y = -0.2
    msg.pose.position.z = 0.3

    bridge.on_ee_pose(msg)

    snapshot = runtime_state.get_state(now_ms=0)
    assert snapshot["ee_pose"] == {"x": 0.1, "y": -0.2, "z": 0.3}


def test_sandbox_bridge_computes_tcp_speed_from_velocity_command() -> None:
    runtime_state = create_runtime_state()
    bridge = SandboxBridge(runtime_state=runtime_state)
    msg = TwistStamped()
    msg.twist.linear.x = 3.0
    msg.twist.linear.y = 4.0
    msg.twist.linear.z = 12.0

    bridge.on_velocity_command(msg)

    snapshot = runtime_state.get_state(now_ms=0)
    assert snapshot["tcp_speed_mps"] == 13.0


def test_sandbox_bridge_updates_joint_positions() -> None:
    runtime_state = create_runtime_state()
    bridge = SandboxBridge(runtime_state=runtime_state)
    msg = Float64MultiArray()
    msg.data = [1.0, 2.5, -3.0]

    bridge.on_joint_pose(msg)

    snapshot = runtime_state.get_state(now_ms=0)
    assert snapshot["joint_positions"] == [1.0, 2.5, -3.0]
