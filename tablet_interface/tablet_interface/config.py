from __future__ import annotations

from dataclasses import dataclass

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node


#: Publish tablet teleop as ``cartesian_manager`` inputs (TwistStamped + mode
#: requests). This is the current control stack.
COMMAND_BACKEND_CARTESIAN_MANAGER = "cartesian_manager"

#: Publish the legacy ``extender_msgs/TeleopCommand`` consumed by
#: ``sandbox_controller`` and ``controllers/cartesian_velocity``. Kept as a
#: rollback path while the manager stack is being validated on hardware.
COMMAND_BACKEND_TELEOP_COMMAND = "teleop_command"

COMMAND_BACKENDS = (
    COMMAND_BACKEND_CARTESIAN_MANAGER,
    COMMAND_BACKEND_TELEOP_COMMAND,
)


@dataclass(frozen=True)
class TabletInterfaceConfig:
    command_backend: str
    cartesian_command_topic: str
    mode_request_topic: str
    command_frame_id: str
    teleop_cmd_topic: str
    publish_rate_hz: float
    linear_scale: float
    angular_scale: float
    swap_xy: bool
    linear_axes: list[int]
    linear_signs: list[float]
    angular_axes: list[int]
    angular_signs: list[float]
    default_mode: int
    accept_mode_from_client: bool
    state_publish_hz: float
    bind_host: str
    bind_port: int
    ws_path: str
    state_machine_topic: str
    gripper_topic: str
    gripper_open_position: float
    gripper_close_position: float
    hub_digital_output_topic: str
    hub_electromagnet_channel: float
    petanque_param_service: str
    petanque_total_duration_param: str
    petanque_angle_between_start_and_finish_param: str
    petanque_alpha_param: str
    measure_request_image_topic: str
    measure_result_image_topic: str
    measure_result_vectors_topic: str
    sandbox_ee_pose_topic: str
    sandbox_velocity_command_topic: str
    sandbox_joint_pose_topic: str
    sandbox_joint_pose_message_type: str
    sandbox_toggle_output_topic: str
    sandbox_toggle_output_message_type: str
    sandbox_toggle_output_mode: str
    eager_typed_publishers: list[str]
    topic_snapshot_hz: float
    topic_monitor_specs: list[str]
    param_call_timeout_sec: float


PARAMETER_DEFAULTS = {
    "command_backend": COMMAND_BACKEND_CARTESIAN_MANAGER,
    "cartesian_command_topic": "/joystick_cartesian_command",
    "mode_request_topic": "/mode_request",
    # cartesian_manager does no TF conversion. This must match its
    # default_input_frame_id, or every command is dropped with a warning.
    "command_frame_id": "base_link",
    "teleop_cmd_topic": "/teleop_cmd",
    "publish_rate_hz": 30.0,
    "linear_scale": 0.2,
    "angular_scale": 0.5,
    "swap_xy": False,
    "linear_axes": [0, 1, 2],
    "linear_signs": [1.0, 1.0, 1.0],
    "angular_axes": [0, 1, 2],
    "angular_signs": [1.0, 1.0, 1.0],
    "default_mode": 0,
    "accept_mode_from_client": True,
    "state_publish_hz": 5.0,
    "bind_host": "0.0.0.0",
    "bind_port": 8765,
    "ws_path": "/ws/control",
    "state_machine_topic": "/petanque_state_machine/change_state",
    "gripper_topic": "/gripper_controller/commands",
    "gripper_open_position": 0.2,
    "gripper_close_position": 1.1,
    "hub_digital_output_topic": "/hub/digital_output",
    "hub_electromagnet_channel": 2.0,
    "petanque_param_service": "/petanque_throw/set_parameters",
    "petanque_total_duration_param": "total_duration",
    "petanque_angle_between_start_and_finish_param": "angle_between_start_and_finish",
    "petanque_alpha_param": "alpha",
    "measure_request_image_topic": "/petanque/measure/request_image/compressed",
    "measure_result_image_topic": "/petanque/measure/result_image/compressed",
    "measure_result_vectors_topic": "/petanque/measure/result_vectors",
    # Feedback now comes from qontrol_controller through cartesian_manager
    # instead of sandbox_controller.
    "sandbox_ee_pose_topic": "/ee_pose",
    "sandbox_velocity_command_topic": "/ee_velocity",
    "sandbox_joint_pose_topic": "/joint_states",
    "sandbox_joint_pose_message_type": "sensor_msgs/msg/JointState",
    "sandbox_toggle_output_topic": "/sandbox/digital_output",
    "sandbox_toggle_output_message_type": "std_msgs/msg/Float64",
    "sandbox_toggle_output_mode": "numeric",
    "eager_typed_publishers": [],
    "topic_snapshot_hz": 10.0,
    "topic_monitor_specs": [
        "/tag_detections|extender_msgs/msg/SharedControlGoalArray",
        "/visual_servoing/velocity_command|geometry_msgs/msg/TwistStamped",
        "/visual_servoing/error_TAGtoTAGd|geometry_msgs/msg/TwistStamped",
    ],
    "param_call_timeout_sec": 1.5,
}


def read_string_array_parameter(node: Node, name: str) -> list[str]:
    """Read a string-array parameter, tolerating an empty declaration.

    rclpy cannot infer a type for an empty list, so an empty default is declared
    through a descriptor and stays uninitialized until something sets it.
    ``get_parameter`` raises in that state, so an empty list is returned instead.
    """
    try:
        value = node.get_parameter(name).value
    except ParameterUninitializedException:
        return []
    if value is None:
        return []
    return [str(item) for item in value]


def normalize_command_backend(value: str, *, logger=None) -> str:
    """Resolve the configured command backend, falling back to the current stack.

    An unknown value is a configuration mistake that would otherwise be noticed
    only when the robot does not move, so it is reported and coerced rather than
    left to fail silently.
    """
    normalized = value.strip().lower().replace("-", "_")
    if normalized in COMMAND_BACKENDS:
        return normalized

    if logger is not None:
        logger.warning(
            "Unknown command_backend '{0}', falling back to '{1}'. Expected one of {2}.".format(
                value,
                COMMAND_BACKEND_CARTESIAN_MANAGER,
                ", ".join(COMMAND_BACKENDS),
            )
        )
    return COMMAND_BACKEND_CARTESIAN_MANAGER


def declare_tablet_interface_parameters(node: Node) -> None:
    for name, default in PARAMETER_DEFAULTS.items():
        if isinstance(default, list) and not default:
            # rclpy cannot infer the element type of an empty list, so an empty
            # default needs an explicit descriptor.
            node.declare_parameter(
                name,
                default,
                ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
            )
            continue
        node.declare_parameter(name, default)


def load_tablet_interface_config(node: Node) -> TabletInterfaceConfig:
    return TabletInterfaceConfig(
        command_backend=normalize_command_backend(
            str(node.get_parameter("command_backend").value),
            logger=node.get_logger(),
        ),
        cartesian_command_topic=str(
            node.get_parameter("cartesian_command_topic").value
        ),
        mode_request_topic=str(node.get_parameter("mode_request_topic").value),
        command_frame_id=str(node.get_parameter("command_frame_id").value),
        teleop_cmd_topic=str(node.get_parameter("teleop_cmd_topic").value),
        publish_rate_hz=float(node.get_parameter("publish_rate_hz").value),
        linear_scale=float(node.get_parameter("linear_scale").value),
        angular_scale=float(node.get_parameter("angular_scale").value),
        swap_xy=bool(node.get_parameter("swap_xy").value),
        linear_axes=list(node.get_parameter("linear_axes").value),
        linear_signs=list(node.get_parameter("linear_signs").value),
        angular_axes=list(node.get_parameter("angular_axes").value),
        angular_signs=list(node.get_parameter("angular_signs").value),
        default_mode=int(node.get_parameter("default_mode").value),
        accept_mode_from_client=bool(node.get_parameter("accept_mode_from_client").value),
        state_publish_hz=float(node.get_parameter("state_publish_hz").value),
        bind_host=str(node.get_parameter("bind_host").value),
        bind_port=int(node.get_parameter("bind_port").value),
        ws_path=str(node.get_parameter("ws_path").value),
        state_machine_topic=str(node.get_parameter("state_machine_topic").value),
        gripper_topic=str(node.get_parameter("gripper_topic").value),
        gripper_open_position=float(node.get_parameter("gripper_open_position").value),
        gripper_close_position=float(node.get_parameter("gripper_close_position").value),
        hub_digital_output_topic=str(node.get_parameter("hub_digital_output_topic").value),
        hub_electromagnet_channel=float(
            node.get_parameter("hub_electromagnet_channel").value
        ),
        petanque_param_service=str(node.get_parameter("petanque_param_service").value),
        petanque_total_duration_param=str(
            node.get_parameter("petanque_total_duration_param").value
        ),
        petanque_angle_between_start_and_finish_param=str(
            node.get_parameter("petanque_angle_between_start_and_finish_param").value
        ),
        petanque_alpha_param=str(node.get_parameter("petanque_alpha_param").value),
        measure_request_image_topic=str(
            node.get_parameter("measure_request_image_topic").value
        ),
        measure_result_image_topic=str(
            node.get_parameter("measure_result_image_topic").value
        ),
        measure_result_vectors_topic=str(
            node.get_parameter("measure_result_vectors_topic").value
        ),
        sandbox_ee_pose_topic=str(node.get_parameter("sandbox_ee_pose_topic").value),
        sandbox_velocity_command_topic=str(
            node.get_parameter("sandbox_velocity_command_topic").value
        ),
        sandbox_joint_pose_topic=str(
            node.get_parameter("sandbox_joint_pose_topic").value
        ),
        sandbox_joint_pose_message_type=str(
            node.get_parameter("sandbox_joint_pose_message_type").value
        ),
        sandbox_toggle_output_topic=str(
            node.get_parameter("sandbox_toggle_output_topic").value
        ),
        sandbox_toggle_output_message_type=str(
            node.get_parameter("sandbox_toggle_output_message_type").value
        ),
        sandbox_toggle_output_mode=str(
            node.get_parameter("sandbox_toggle_output_mode").value
        ),
        eager_typed_publishers=read_string_array_parameter(
            node, "eager_typed_publishers"
        ),
        topic_snapshot_hz=float(node.get_parameter("topic_snapshot_hz").value),
        topic_monitor_specs=read_string_array_parameter(
            node, "topic_monitor_specs"
        ),
        param_call_timeout_sec=float(
            node.get_parameter("param_call_timeout_sec").value
        ),
    )
