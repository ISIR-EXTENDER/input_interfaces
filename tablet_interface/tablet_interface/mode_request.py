"""Mode-request grammar shared with ``cartesian_manager``.

``cartesian_manager`` selects its shapers from a ``std_msgs/msg/String`` published
on ``/mode_request``. The node normalizes the incoming string before parsing it
(``normalizeParameterName`` in ``src/ros/parameter_parsing.cpp``) and then splits
it on ``/`` (``splitModeRequest`` in ``src/core/manager.cpp``).

This module mirrors both steps so the tablet backend can reject an invalid mode
locally, with a useful message, instead of publishing a string the manager will
silently drop.
"""

from __future__ import annotations

from typing import Tuple

import yaml

GEOMETRIC_PREFIX = "geometric"
BEHAVIOUR_PREFIX = "behaviour"

#: Geometric shapers accepted by ``Manager::setMode``.
#:
#: The target architecture selects between ``translation``, ``orientation``,
#: ``snake`` and ``both``; ``jaco`` is the current transitional name. When the
#: manager gains ``translation``/``orientation``, add them here and the tablet's
#: own TRANSLATION/ROTATION modes can become real mode requests instead of
#: local axis masks.
GEOMETRIC_MODES = ("both", "jaco", "snake")

#: ``geometric/both`` applies no shaping, so it is the neutral geometric state.
DEFAULT_GEOMETRIC_MODE = f"{GEOMETRIC_PREFIX}/both"
SNAKE_MODE = f"{GEOMETRIC_PREFIX}/snake"
JACO_MODE = f"{GEOMETRIC_PREFIX}/jaco"
PASSTHROUGH_MODE = f"{BEHAVIOUR_PREFIX}/passthrough"
JOINT_TARGET_PREFIX = f"{BEHAVIOUR_PREFIX}/joint_target"


def normalize_mode_request(raw: str) -> str:
    """Normalize a mode request the way ``cartesian_manager`` does.

    Lowercases the request and turns ``-`` into ``_``. Surrounding whitespace is
    stripped first because it never survives a YAML/JSON round trip intact.
    """
    return raw.strip().lower().replace("-", "_")


def validate_mode_request(raw: str) -> Tuple[bool, str, str]:
    """Validate a mode request against the manager grammar.

    Returns ``(ok, normalized, detail)``. ``detail`` explains the rejection when
    ``ok`` is ``False``, and echoes the parsed shape when it is ``True``.

    Joint-target names are *not* validated here: the set of valid names lives in
    the manager's ``behaviours.joint_targets.target_names`` parameter, which the
    tablet backend does not read. The manager rejects an unknown name itself.
    """
    normalized = normalize_mode_request(raw)
    if not normalized:
        return False, normalized, "mode request is empty"

    parts = normalized.split("/")
    if any(not part for part in parts):
        return False, normalized, "mode request has an empty path segment"
    if len(parts) < 2:
        return (
            False,
            normalized,
            "mode request needs at least two segments, such as geometric/both",
        )

    if parts[0] == GEOMETRIC_PREFIX:
        if len(parts) != 2:
            return False, normalized, "geometric mode request takes exactly one name"
        if parts[1] not in GEOMETRIC_MODES:
            return (
                False,
                normalized,
                "unknown geometric mode '{0}', expected one of {1}".format(
                    parts[1],
                    ", ".join(GEOMETRIC_MODES),
                ),
            )
        return True, normalized, f"geometric mode {parts[1]}"

    if parts[0] == BEHAVIOUR_PREFIX:
        if parts[1] == "passthrough":
            if len(parts) != 2:
                return (
                    False,
                    normalized,
                    "behaviour/passthrough takes no extra segment",
                )
            return True, normalized, "behaviour passthrough"

        if parts[1] == "joint_target":
            if len(parts) != 3:
                return (
                    False,
                    normalized,
                    "behaviour/joint_target needs a target name, "
                    "such as behaviour/joint_target/home",
                )
            return True, normalized, f"behaviour joint target {parts[2]}"

        return (
            False,
            normalized,
            f"unknown behaviour '{parts[1]}', expected passthrough or joint_target",
        )

    return (
        False,
        normalized,
        "unknown mode family '{0}', expected {1} or {2}".format(
            parts[0],
            GEOMETRIC_PREFIX,
            BEHAVIOUR_PREFIX,
        ),
    )


def extract_mode_from_payload_text(payload_text: str) -> str:
    """Pull the mode string out of a typed-widget payload.

    UI widgets send ROS payloads as YAML text, so a String mode arrives as
    ``{data: geometric/snake}``. A bare ``geometric/snake`` is accepted too,
    because that is what an operator types first.

    Returns an empty string when nothing usable is present; the caller reports
    the rejection through the normal validation path.
    """
    trimmed = payload_text.strip()
    if not trimmed:
        return ""

    try:
        parsed = yaml.safe_load(trimmed)
    except yaml.YAMLError:
        return trimmed

    if isinstance(parsed, dict):
        value = parsed.get("data")
        return str(value).strip() if value is not None else ""
    if parsed is None:
        return ""
    return str(parsed).strip()


def is_one_shot_mode_request(raw: str) -> bool:
    """Report whether the manager returns to passthrough right after the request.

    Joint targets are dispatched once and then the manager immediately falls back
    to passthrough, so the tablet backend must not treat them as sticky state.
    """
    normalized = normalize_mode_request(raw)
    return normalized.startswith(f"{JOINT_TARGET_PREFIX}/")


__all__ = [
    "BEHAVIOUR_PREFIX",
    "DEFAULT_GEOMETRIC_MODE",
    "GEOMETRIC_MODES",
    "GEOMETRIC_PREFIX",
    "JACO_MODE",
    "JOINT_TARGET_PREFIX",
    "PASSTHROUGH_MODE",
    "SNAKE_MODE",
    "extract_mode_from_payload_text",
    "is_one_shot_mode_request",
    "normalize_mode_request",
    "validate_mode_request",
]
