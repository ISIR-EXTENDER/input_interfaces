from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field, ValidationError, conint, confloat


class Vector3Model(BaseModel):
    x: confloat(strict=True)
    y: confloat(strict=True)
    z: confloat(strict=True)


class CmdMessage(BaseModel):
    type: Literal["teleop_cmd"]
    seq: conint(strict=True, ge=0)
    mode: conint(strict=True, ge=0, le=3)
    linear: Vector3Model
    angular: Vector3Model


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
    "StateMessage",
    "EventMessage",
    "Vector3Model",
    "ValidationError",
]
