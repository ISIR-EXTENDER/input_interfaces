from __future__ import annotations

import pytest

from tablet_interface.mode_request import (
    DEFAULT_GEOMETRIC_MODE,
    JACO_MODE,
    PASSTHROUGH_MODE,
    SNAKE_MODE,
    extract_mode_from_payload_text,
    is_one_shot_mode_request,
    normalize_mode_request,
    validate_mode_request,
)


@pytest.mark.parametrize(
    "raw,expected",
    [
        ("geometric/snake", "geometric/snake"),
        ("  geometric/snake  ", "geometric/snake"),
        ("GEOMETRIC/SNAKE", "geometric/snake"),
        ("behaviour/joint-target/home", "behaviour/joint_target/home"),
    ],
)
def test_normalize_mode_request_matches_manager_rules(raw: str, expected: str) -> None:
    assert normalize_mode_request(raw) == expected


@pytest.mark.parametrize(
    "raw,expected_normalized",
    [
        (DEFAULT_GEOMETRIC_MODE, "geometric/both"),
        (JACO_MODE, "geometric/jaco"),
        (SNAKE_MODE, "geometric/snake"),
        (PASSTHROUGH_MODE, "behaviour/passthrough"),
        ("behaviour/joint_target/home", "behaviour/joint_target/home"),
        ("Behaviour/Joint-Target/Home", "behaviour/joint_target/home"),
    ],
)
def test_validate_accepts_manager_grammar(raw: str, expected_normalized: str) -> None:
    ok, normalized, _detail = validate_mode_request(raw)

    assert ok is True
    assert normalized == expected_normalized


@pytest.mark.parametrize(
    "raw",
    [
        "",
        "   ",
        "geometric",
        "geometric/",
        "geometric/spiral",
        "geometric/jaco/extra",
        "behaviour/passthrough/extra",
        "behaviour/joint_target",
        "behaviour/unknown",
        "kinematic/jaco",
        "//",
    ],
)
def test_validate_rejects_invalid_requests(raw: str) -> None:
    ok, _normalized, detail = validate_mode_request(raw)

    assert ok is False
    assert detail


def test_validate_does_not_check_joint_target_names() -> None:
    # The manager owns the list of target names, so an unknown name is accepted
    # here and rejected downstream.
    ok, normalized, _detail = validate_mode_request("behaviour/joint_target/anything")

    assert ok is True
    assert normalized == "behaviour/joint_target/anything"


@pytest.mark.parametrize(
    "raw,expected",
    [
        ("behaviour/joint_target/home", True),
        ("behaviour/passthrough", False),
        ("geometric/snake", False),
    ],
)
def test_is_one_shot_mode_request(raw: str, expected: bool) -> None:
    assert is_one_shot_mode_request(raw) is expected


@pytest.mark.parametrize(
    "payload_text,expected",
    [
        ("{data: geometric/snake}", "geometric/snake"),
        ('{data: "behaviour/joint_target/home"}', "behaviour/joint_target/home"),
        ("geometric/both", "geometric/both"),
        ("  geometric/both  ", "geometric/both"),
        ("", ""),
        ("{}", ""),
        ("{other: geometric/snake}", ""),
    ],
)
def test_extract_mode_from_payload_text(payload_text: str, expected: str) -> None:
    assert extract_mode_from_payload_text(payload_text) == expected
