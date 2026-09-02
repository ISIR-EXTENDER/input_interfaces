#!/usr/bin/env python3
# Copyright 2026 Etienne Moullet
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for mouse joystick output configuration and Joy axis mapping."""

import math

from builtin_interfaces.msg import Time
from mouse_joystick_server import _build_joy_axes
from mouse_joystick_server import _build_joy_message
from mouse_joystick_server import _build_teleop_command
from mouse_joystick_server import _validate_joy_mapping
from mouse_joystick_server import _validate_output_type
import pytest


@pytest.mark.parametrize('output_type', ['twist', 'joy'])
def test_supported_output_types(output_type):
    _validate_output_type(output_type)


@pytest.mark.parametrize('output_type', ['', 'teleop_command', 'Joy', 'both'])
def test_unsupported_output_type_is_rejected(output_type):
    with pytest.raises(ValueError, match='Invalid output_type'):
        _validate_output_type(output_type)


def test_joy_axes_use_configured_indexes_and_zero_fill_unused_axes():
    axes = _build_joy_axes(
        x=0.25,
        y=-0.5,
        axis_count=4,
        x_axis_index=2,
        x_axis_scale=-2.0,
        y_axis_index=0,
        y_axis_scale=0.5,
    )

    assert axes == [-0.25, 0.0, -0.5, 0.0]


def test_joy_axes_are_clamped_after_scaling():
    axes = _build_joy_axes(
        x=0.75,
        y=-0.75,
        axis_count=2,
        x_axis_index=0,
        x_axis_scale=2.0,
        y_axis_index=1,
        y_axis_scale=2.0,
    )

    assert axes == [1.0, -1.0]


def test_neutral_joy_axes_are_all_zero():
    axes = _build_joy_axes(0.0, 0.0, 3, 2, -1.0, 0, 1.0)

    assert axes == [0.0, 0.0, 0.0]


def test_teleop_command_preserves_existing_translation_mapping():
    message = _build_teleop_command(0.25, -0.75)

    if hasattr(message, 'TRANSLATION'):
        assert message.mode == message.TRANSLATION
    assert message.twist.linear.x == 0.25
    assert message.twist.linear.y == -0.75
    assert message.twist.linear.z == 0.0
    assert message.twist.angular.x == 0.0
    assert message.twist.angular.y == 0.0
    assert message.twist.angular.z == 0.0


def test_joy_message_contains_timestamp_mapping_and_no_buttons():
    stamp = Time(sec=123, nanosec=456)
    message = _build_joy_message(0.5, -0.25, stamp, 3, 2, -1.0, 0, 2.0)

    assert message.header.stamp == stamp
    assert list(message.axes) == [-0.5, 0.0, -0.5]
    assert list(message.buttons) == []


@pytest.mark.parametrize(
    'axis_count,x_index,x_scale,y_index,y_scale,error',
    [
        (0, 0, 1.0, 1, 1.0, 'joy_axis_count'),
        (2, -1, 1.0, 1, 1.0, 'joy_axes.x.index'),
        (2, 0, 1.0, 2, 1.0, 'joy_axes.y.index'),
        (2, 0, 1.0, 0, 1.0, 'must be different'),
        (2, 0, math.inf, 1, 1.0, 'joy_axes.x.scale'),
        (2, 0, 1.0, 1, math.nan, 'joy_axes.y.scale'),
    ],
)
def test_invalid_joy_mapping_is_rejected(
    axis_count,
    x_index,
    x_scale,
    y_index,
    y_scale,
    error,
):
    with pytest.raises(ValueError, match=error):
        _validate_joy_mapping(axis_count, x_index, x_scale, y_index, y_scale)


def test_valid_joy_mapping_is_accepted():
    _validate_joy_mapping(4, 3, -1.0, 1, 0.5)
