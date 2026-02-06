from tablet_interface.safety_gate import SafetyGate, TeleopCmd, Twist


def _twist(lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0) -> Twist:
    t = Twist()
    t.linear.x = lx
    t.linear.y = ly
    t.linear.z = lz
    t.angular.x = ax
    t.angular.y = ay
    t.angular.z = az
    return t


def test_watchdog_timeout_zeroes_twist() -> None:
    gate = SafetyGate(
        watchdog_timeout_ms=100,
        max_linear_mps=1.0,
        max_linear_z_mps=1.0,
        max_angular_rps=1.0,
    )
    cmd = TeleopCmd(
        twist=_twist(lx=0.5),
        mode=2,
        seq=1,
        received_ms=0,
    )
    gate.update_cmd(cmd)

    twist, mode, events = gate.process(now_ms=200)
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0
    assert mode == 2
    assert "watchdog_timeout" in events

def test_saturation_limits_components() -> None:
    gate = SafetyGate(
        watchdog_timeout_ms=1000,
        max_linear_mps=1.0,
        max_linear_z_mps=0.2,
        max_angular_rps=1.5,
    )
    cmd = TeleopCmd(
        twist=_twist(lx=2.0, ly=-2.0, lz=1.0, ax=3.0, ay=-3.0, az=0.5),
        mode=0,
        seq=3,
        received_ms=0,
    )
    gate.update_cmd(cmd)

    twist, mode, events = gate.process(now_ms=10)
    assert twist.linear.x == 1.0
    assert twist.linear.y == -1.0
    assert twist.linear.z == 0.2
    assert twist.angular.x == 1.5
    assert twist.angular.y == -1.5
    assert twist.angular.z == 0.5
    assert mode == 0
    assert events == []
