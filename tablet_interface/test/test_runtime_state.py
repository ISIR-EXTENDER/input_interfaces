from tablet_interface.runtime_state import TabletRuntimeState


def create_runtime_state() -> TabletRuntimeState:
    return TabletRuntimeState(
        default_mode=2,
        publish_rate_hz=60.0,
        gripper_open_position=0.2,
        gripper_close_position=1.1,
        measure_demo_vectors_json='{"source":"demo"}',
        measure_demo_image_data_url="data:image/png;base64,demo",
    )


def test_runtime_state_tracks_command_and_connection_metadata() -> None:
    state = create_runtime_state()

    state.set_connected(True)
    state.update_command_meta(mode=1, seq=7, received_ms=1200)

    snapshot = state.get_state(now_ms=1350)
    assert snapshot["connected"] is True
    assert snapshot["cmd_age_ms"] == 150
    assert snapshot["last_seq"] == 7
    assert snapshot["current_mode"] == 1
    assert snapshot["publishing_rate_hz"] == 60.0


def test_runtime_state_maps_gripper_position_and_sandbox_feedback() -> None:
    state = create_runtime_state()

    state.set_gripper_action("open")
    state.update_gripper_position(1.0)
    state.update_ee_pose(x=0.1, y=0.2, z=0.3)
    state.update_tcp_speed(0.42)
    state.update_joint_positions([1.0, 2.0, 3.0])

    snapshot = state.get_state(now_ms=0)
    assert snapshot["gripper_state"] == "close"
    assert snapshot["ee_pose"] == {"x": 0.1, "y": 0.2, "z": 0.3}
    assert snapshot["tcp_speed_mps"] == 0.42
    assert snapshot["joint_positions"] == [1.0, 2.0, 3.0]


def test_runtime_state_uses_demo_measure_snapshot_for_legacy_vectors() -> None:
    state = create_runtime_state()

    state.update_measure_result_image("data:image/png;base64,real", now_ms=100)
    state.update_measure_result_vectors(
        '{"source":"fake_opencv_demo","distances_cm":[27.9]}',
        now_ms=120,
    )

    snapshot = state.get_measure_result_snapshot()
    assert snapshot["image_data_url"] == "data:image/png;base64,demo"
    assert snapshot["vectors_json"] == '{"source":"demo"}'
    assert snapshot["updated_at_ms"] is None
