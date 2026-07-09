from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field, ValidationError, confloat, conint, model_validator


class Vector3Model(BaseModel):
    # UI teleop gains can amplify normalized joystick values above 1.0.
    # Keep a finite range to reject clearly invalid payloads.
    x: confloat(ge=-20.0, le=20.0)
    y: confloat(ge=-20.0, le=20.0)
    z: confloat(ge=-20.0, le=20.0)


class CmdMessage(BaseModel):
    type: Literal["teleop_cmd"]
    seq: conint(strict=True, ge=0)
    mode: conint(strict=True, ge=0, le=4)
    linear: Vector3Model
    angular: Vector3Model


class StateCmdMessage(BaseModel):
    type: Literal["state_cmd"]
    command: Literal[
        "teleop",
        "activate_throw",
        "go_to_start",
        "throw",
        "pick_up",
        "stop",
        "test_loop",
    ]


class GripperCmdMessage(BaseModel):
    type: Literal["gripper_cmd"]
    action: Literal["open", "close"]
    speed: confloat(ge=0.0, le=1.0) | None = None
    force: confloat(ge=0.0, le=1.0) | None = None


class PetanqueConfigMessage(BaseModel):
    type: Literal["petanque_cfg"]
    total_duration: confloat(gt=0) | None = None
    angle_between_start_and_finish: float | None = None
    alpha: confloat(ge=0.0, le=40.0) | None = None

    @model_validator(mode="after")
    def _validate_has_payload(self) -> "PetanqueConfigMessage":
        if (
            self.total_duration is None
            and self.angle_between_start_and_finish is None
            and self.alpha is None
        ):
            raise ValueError(
                "petanque_cfg requires at least one field: total_duration or "
                "angle_between_start_and_finish or alpha"
            )
        return self


class UiButtonMessage(BaseModel):
    type: Literal["ui_button"]
    topic: str = Field(min_length=1)
    payload: str = Field(min_length=1)
    widget_id: str | None = None


class UiScalarMessage(BaseModel):
    type: Literal["ui_scalar"]
    topic: str = Field(min_length=1)
    value: confloat(allow_inf_nan=False)
    widget_id: str | None = None


class UiBoolMessage(BaseModel):
    type: Literal["ui_bool"]
    topic: str = Field(min_length=1)
    value: bool
    widget_id: str | None = None


class UiTypedMessage(BaseModel):
    type: Literal["ui_typed"]
    topic: str = Field(min_length=1)
    message_type: str = Field(min_length=1)
    payload_text: str = Field(min_length=1)
    widget_id: str | None = None


class CameraFrameMessage(BaseModel):
    type: Literal["camera_frame"]
    topic: str = Field(min_length=1)
    image_data_url: str = Field(min_length=32)
    widget_id: str | None = None

    @model_validator(mode="after")
    def _validate_image_data_url(self) -> "CameraFrameMessage":
        if not self.image_data_url.startswith("data:image/"):
            raise ValueError("camera_frame image_data_url must start with data:image/")
        return self


class TopicMonitorSpec(BaseModel):
    topic: str = Field(min_length=1)
    message_type: str = Field(min_length=1)


class TopicSubscribeMessage(BaseModel):
    type: Literal["topic_subscribe"]
    topics: list[TopicMonitorSpec] = Field(min_length=1)


class MeasureRequestMessage(BaseModel):
    type: Literal["measure_request"]
    image_data_url: str = Field(min_length=32)

    @model_validator(mode="after")
    def _validate_image_data_url(self) -> "MeasureRequestMessage":
        if not self.image_data_url.startswith("data:image/"):
            raise ValueError("measure_request image_data_url must start with data:image/")
        return self


class MeasureRefreshMessage(BaseModel):
    type: Literal["measure_refresh"]


class MeasureResultMessage(BaseModel):
    type: Literal["measure_result"]
    image_data_url: str | None = None
    vectors_json: str | None = None
    updated_at_ms: conint(strict=True, ge=0) | None = None


class TopicSnapshotMessage(BaseModel):
    type: Literal["topic_snapshot"]
    topic: str = Field(min_length=1)
    message_type: str = Field(min_length=1)
    updated_at_ms: conint(strict=True, ge=0) | None = None
    revision: conint(strict=True, ge=0)
    data: Any | None = None
    error: str | None = None


class PositionMessage(BaseModel):
    x: confloat(allow_inf_nan=False)
    y: confloat(allow_inf_nan=False)
    z: confloat(allow_inf_nan=False)


class StateMessage(BaseModel):
    type: Literal["state"]
    connected: bool
    cmd_age_ms: conint(strict=True, ge=0) | None = None
    watchdog_timeout_ms: conint(strict=True, ge=0)
    last_seq: conint(strict=True, ge=0)
    publishing_rate_hz: confloat(strict=True, ge=0)
    current_mode: conint(strict=True, ge=0, le=3)
    gripper_state: Literal["open", "close", "unknown"] | None = None
    ee_pose: PositionMessage | None = None
    tcp_speed_mps: confloat(ge=0.0, allow_inf_nan=False) | None = None
    joint_positions: list[confloat(allow_inf_nan=False)] | None = None


class EventMessage(BaseModel):
    type: Literal["event"]
    severity: Literal["info", "warning", "error"]
    code: str = Field(min_length=1)
    message: str = Field(min_length=1)


__all__ = [
    "CmdMessage",
    "StateCmdMessage",
    "GripperCmdMessage",
    "PetanqueConfigMessage",
    "UiButtonMessage",
    "UiScalarMessage",
    "UiBoolMessage",
    "UiTypedMessage",
    "CameraFrameMessage",
    "TopicMonitorSpec",
    "TopicSubscribeMessage",
    "MeasureRequestMessage",
    "MeasureRefreshMessage",
    "MeasureResultMessage",
    "TopicSnapshotMessage",
    "PositionMessage",
    "StateMessage",
    "EventMessage",
    "Vector3Model",
    "ValidationError",
]
