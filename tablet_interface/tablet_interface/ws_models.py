from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field, ValidationError, confloat, conint


class Vector3Model(BaseModel):
    x: confloat(ge=-1.0, le=1.0)
    y: confloat(ge=-1.0, le=1.0)
    z: confloat(ge=-1.0, le=1.0)


class CmdMessage(BaseModel):
    type: Literal["teleop_cmd"]
    seq: conint(strict=True, ge=0)
    mode: conint(strict=True, ge=0, le=3)
    linear: Vector3Model
    angular: Vector3Model


class StateCmdMessage(BaseModel):
    type: Literal["state_cmd"]
    command: Literal["teleop", "activate_throw", "go_to_start", "throw", "stop"]


class PetanqueConfigMessage(BaseModel):
    type: Literal["petanque_cfg"]
    total_duration: confloat(gt=0)


class UiButtonMessage(BaseModel):
    type: Literal["ui_button"]
    topic: str = Field(min_length=1)
    payload: str = Field(min_length=1)
    widget_id: str | None = None


class StateMessage(BaseModel):
    type: Literal["state"]
    connected: bool
    cmd_age_ms: conint(strict=True, ge=0)
    watchdog_timeout_ms: conint(strict=True, ge=0)
    last_seq: conint(strict=True, ge=0)
    publishing_rate_hz: confloat(strict=True, ge=0)
    current_mode: conint(strict=True, ge=0, le=3)


class EventMessage(BaseModel):
    type: Literal["event"]
    severity: Literal["info", "warning", "error"]
    code: str = Field(min_length=1)
    message: str = Field(min_length=1)


__all__ = [
    "CmdMessage",
    "StateCmdMessage",
    "PetanqueConfigMessage",
    "UiButtonMessage",
    "StateMessage",
    "EventMessage",
    "Vector3Model",
    "ValidationError",
]
