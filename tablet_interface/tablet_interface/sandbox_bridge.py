from __future__ import annotations

from math import sqrt
from typing import Protocol

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from tablet_interface.runtime_state import TabletRuntimeState


#: Message types accepted on the joint feedback topic.
#:
#: ``sandbox_controller`` published a bare ``Float64MultiArray`` on
#: ``/sandbox_controller/joint_pose``. The cartesian_manager stack reads
#: ``/joint_states``, which is a ``sensor_msgs/msg/JointState``. Both are
#: supported so the tablet can run against either stack.
JOINT_POSE_MESSAGE_TYPES = (
    "sensor_msgs/msg/JointState",
    "std_msgs/msg/Float64MultiArray",
)

DEFAULT_JOINT_POSE_MESSAGE_TYPE = "sensor_msgs/msg/JointState"


def resolve_joint_pose_message_class(message_type: str):
    """Map a joint feedback message type name to its class.

    Raises ``ValueError`` for an unsupported type so the node can report a
    configuration error instead of subscribing with the wrong class.
    """
    normalized = message_type.strip()
    if normalized == "sensor_msgs/msg/JointState":
        return JointState
    if normalized == "std_msgs/msg/Float64MultiArray":
        return Float64MultiArray
    raise ValueError(
        "unsupported joint pose message type '{0}', expected one of {1}".format(
            message_type,
            ", ".join(JOINT_POSE_MESSAGE_TYPES),
        )
    )


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

    def on_joint_pose(self, msg: Float64MultiArray | JointState) -> None:
        # JointState carries positions in `position`; Float64MultiArray in `data`.
        positions = getattr(msg, "position", None)
        if positions is None:
            positions = msg.data
        self._runtime_state.update_joint_positions(
            [float(value) for value in positions]
        )

