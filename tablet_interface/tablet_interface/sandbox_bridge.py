from __future__ import annotations

from math import sqrt
from typing import Protocol

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64MultiArray

from tablet_interface.runtime_state import TabletRuntimeState


class SandboxBridge:
    def __init__(self, *, runtime_state: TabletRuntimeState) -> None:
        self._runtime_state = runtime_state

    def on_ee_pose(self, msg: PoseStamped) -> None:
        self._runtime_state.update_ee_pose(
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            z=float(msg.pose.position.z),
        )

    def on_velocity_command(self, msg: TwistStamped) -> None:
        linear = msg.twist.linear
        speed_mps = sqrt(
            float(linear.x) ** 2
            + float(linear.y) ** 2
            + float(linear.z) ** 2
        )
        self._runtime_state.update_tcp_speed(speed_mps)

    def on_joint_pose(self, msg: Float64MultiArray) -> None:
        self._runtime_state.update_joint_positions(
            [float(value) for value in msg.data]
        )

