from __future__ import annotations

from typing import TYPE_CHECKING, Any, Protocol

from tablet_interface.ws_models import (
    CameraFrameMessage,
    CmdMessage,
    EventMessage,
    GripperCmdMessage,
    MeasureRefreshMessage,
    MeasureRequestMessage,
    MeasureResultMessage,
    PetanqueConfigMessage,
    StateCmdMessage,
    StateMessage,
    UiButtonMessage,
    UiScalarMessage,
)

if TYPE_CHECKING:
    from tablet_interface.ros_teleop_publisher import TabletInterfaceNode


class JsonSender(Protocol):
    async def send_json(self, data: Any) -> None:
        ...


async def send_event(
    sender: JsonSender,
    code: str,
    severity: str,
    message: str,
) -> None:
    event = EventMessage(type="event", severity=severity, code=code, message=message)
    await sender.send_json(event.model_dump())


def build_state_message(state: dict[str, object]) -> dict[str, object]:
    message = StateMessage(
        type="state",
        connected=bool(state["connected"]),
        cmd_age_ms=(
            int(state["cmd_age_ms"])
            if isinstance(state.get("cmd_age_ms"), int)
            else None
        ),
        watchdog_timeout_ms=int(state["watchdog_timeout_ms"]),
        last_seq=int(state["last_seq"]),
        publishing_rate_hz=float(state["publishing_rate_hz"]),
        current_mode=int(state["current_mode"]),
        gripper_state=state.get("gripper_state"),
        ee_pose=state.get("ee_pose"),
        tcp_speed_mps=state.get("tcp_speed_mps"),
        joint_positions=state.get("joint_positions"),
    )
    return message.model_dump()


async def send_measure_result(
    sender: JsonSender,
    *,
    image_data_url: str | None,
    vectors_json: str | None,
    updated_at_ms: int | None,
) -> None:
    result = MeasureResultMessage(
        type="measure_result",
        image_data_url=image_data_url,
        vectors_json=vectors_json,
        updated_at_ms=updated_at_ms,
    )
    await sender.send_json(result.model_dump())


def _handle_ui_button_topic(
    node: "TabletInterfaceNode",
    button: UiButtonMessage,
) -> tuple[bool, str, str, str]:
    if button.topic == node.state_machine_topic:
        ok = node.send_state_command(button.payload)
        if ok:
            return True, "STATE_CMD_OK", "info", f"ui_button payload={button.payload}"

    if button.topic == node.hub_digital_output_topic:
        normalized = button.payload.strip().lower()
        enable_values = {
            "electromagnet_on",
            "on",
            "1",
            "true",
            "activate",
        }
        disable_values = {
            "electromagnet_off",
            "off",
            "0",
            "false",
            "deactivate",
        }
        if normalized in enable_values | disable_values:
            enabled = normalized in enable_values
            ok = node.set_electromagnet(enabled)
            return (
                ok,
                "HUB_DIGITAL_OUTPUT_OK" if ok else "HUB_DIGITAL_OUTPUT_FAILED",
                "info" if ok else "warning",
                f"electromagnet={'on' if enabled else 'off'}",
            )

    ok = node.publish_ui_button(button.topic, button.payload)
    if ok:
        return (
            True,
            "UI_BUTTON_OK",
            "info",
            f"ui_button topic={button.topic} payload={button.payload}",
        )

    return (
        False,
        "UI_BUTTON_IGNORED",
        "warning",
        f"unsupported ui_button topic={button.topic}",
    )


async def handle_ws_payload(
    node: "TabletInterfaceNode",
    sender: JsonSender,
    payload: object,
) -> None:
    msg_type = payload.get("type") if isinstance(payload, dict) else None

    if msg_type == "teleop_cmd":
        cmd = CmdMessage.model_validate(payload)
        twist = node.map_and_scale_cmd(
            linear_values=(cmd.linear.x, cmd.linear.y, cmd.linear.z),
            angular_values=(cmd.angular.x, cmd.angular.y, cmd.angular.z),
        )
        node.update_latest_cmd(
            twist=twist,
            mode=int(cmd.mode),
            seq=int(cmd.seq),
            received_ms=node._now_ms(),
        )
        node.get_logger().debug(f"WS teleop_cmd accepted seq={cmd.seq} mode={cmd.mode}")
        return

    if msg_type == "state_cmd":
        state_cmd = StateCmdMessage.model_validate(payload)
        ok = node.send_state_command(state_cmd.command)
        await send_event(
            sender,
            code="STATE_CMD_OK" if ok else "STATE_CMD_FAILED",
            severity="info" if ok else "warning",
            message=f"state_cmd={state_cmd.command}",
        )
        return

    if msg_type == "gripper_cmd":
        gripper_cmd = GripperCmdMessage.model_validate(payload)
        ok = node.set_gripper(gripper_cmd.action)
        await send_event(
            sender,
            code="GRIPPER_CMD_OK" if ok else "GRIPPER_CMD_FAILED",
            severity="info" if ok else "warning",
            message=f"gripper_cmd={gripper_cmd.action}",
        )
        return

    if msg_type == "petanque_cfg":
        cfg = PetanqueConfigMessage.model_validate(payload)
        ok = True
        updated_fields: list[str] = []
        if cfg.total_duration is not None:
            ok = node.set_petanque_total_duration(cfg.total_duration) and ok
            updated_fields.append(f"total_duration={cfg.total_duration:.3f}")
        if cfg.angle_between_start_and_finish is not None:
            ok = (
                node.set_petanque_angle_between_start_and_finish(
                    cfg.angle_between_start_and_finish
                )
                and ok
            )
            updated_fields.append(
                "angle_between_start_and_finish="
                f"{cfg.angle_between_start_and_finish:.3f}"
            )
        if cfg.alpha is not None:
            ok = node.set_petanque_alpha(cfg.alpha) and ok
            updated_fields.append(f"alpha={cfg.alpha:.3f}")
        if updated_fields:
            node.get_logger().info("Applied petanque_cfg: " + ", ".join(updated_fields))
        await send_event(
            sender,
            code="PETANQUE_CFG_OK" if ok else "PETANQUE_CFG_FAILED",
            severity="info" if ok else "warning",
            message=", ".join(updated_fields),
        )
        return

    if msg_type == "measure_request":
        request = MeasureRequestMessage.model_validate(payload)
        ok = node.publish_measure_request_image(request.image_data_url)
        await send_event(
            sender,
            code="MEASURE_REQUEST_OK" if ok else "MEASURE_REQUEST_FAILED",
            severity="info" if ok else "warning",
            message="measure image request sent" if ok else "invalid measure image",
        )
        return

    if msg_type == "measure_refresh":
        MeasureRefreshMessage.model_validate(payload)
        snapshot = node.get_measure_result_snapshot()
        image_data_url = snapshot.get("image_data_url")
        vectors_json = snapshot.get("vectors_json")
        if image_data_url is None and vectors_json is None:
            await send_event(
                sender,
                code="MEASURE_REFRESH_EMPTY",
                severity="warning",
                message="no cached measure result",
            )
            return
        await send_measure_result(
            sender,
            image_data_url=image_data_url if isinstance(image_data_url, str) else None,
            vectors_json=vectors_json if isinstance(vectors_json, str) else None,
            updated_at_ms=(
                int(snapshot["updated_at_ms"])
                if isinstance(snapshot.get("updated_at_ms"), int)
                else None
            ),
        )
        await send_event(
            sender,
            code="MEASURE_REFRESH_OK",
            severity="info",
            message="sent cached measure result",
        )
        return

    if msg_type == "ui_button":
        button = UiButtonMessage.model_validate(payload)
        ok, code, severity, message = _handle_ui_button_topic(node, button)
        await send_event(sender, code=code, severity=severity, message=message)
        return

    if msg_type == "ui_scalar":
        scalar = UiScalarMessage.model_validate(payload)
        ok = node.publish_ui_scalar(scalar.topic, scalar.value)
        await send_event(
            sender,
            code="UI_SCALAR_OK" if ok else "UI_SCALAR_FAILED",
            severity="info" if ok else "warning",
            message=f"ui_scalar topic={scalar.topic} value={scalar.value:.3f}",
        )
        return

    if msg_type == "camera_frame":
        frame = CameraFrameMessage.model_validate(payload)
        ok = node.publish_camera_frame(
            topic=frame.topic,
            image_data_url=frame.image_data_url,
        )
        await send_event(
            sender,
            code="CAMERA_FRAME_OK" if ok else "CAMERA_FRAME_FAILED",
            severity="info" if ok else "warning",
            message=f"camera_frame topic={frame.topic}",
        )
        return

    await send_event(
        sender,
        code="CMD_INVALID_TYPE",
        severity="warning",
        message=f"unsupported type={msg_type}",
    )
