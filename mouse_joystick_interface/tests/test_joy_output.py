# Copyright 2026 Etienne Moullet
# SPDX-License-Identifier: Apache-2.0
"""Tests for Joy-only output and HTTP configuration."""

import io
import json
from unittest.mock import Mock

from sensor_msgs.msg import Joy

from mouse_joystick_server import MouseJoystickInterface, _TeleopHandler, _build_runtime_config


def test_runtime_config_always_includes_button_settings():
    types = ['press'] * 12
    labels = [str(index) for index in range(12)]
    config = _build_runtime_config(types, labels, 120, 80, 600, 'mouseover')
    assert config['joy_button_types'] == types
    assert config['joy_button_labels'] == labels
    assert config['joy_button_width_px'] == 120
    assert 'output_type' not in config


def test_http_command_always_validates_and_passes_buttons():
    node = Mock()
    handler = object.__new__(_TeleopHandler)
    handler.path = '/teleop'
    handler.node = node
    handler._send_json = Mock()
    data = json.dumps({'x': 0.25, 'y': -0.5, 'buttons': [1] + [0] * 11}).encode()
    handler.rfile = io.BytesIO(data)
    handler.headers = {'Content-Length': str(len(data))}
    handler.do_POST()
    node.publish_teleop_command.assert_called_once_with(0.25, -0.5, [1] + [0] * 11)
    handler._send_json.assert_called_once_with({'ok': True})


def test_publish_uses_joy_axes_and_buttons():
    node = Mock()
    node.joy_axis_count_ = 2
    node.joy_x_axis_index_ = 0
    node.joy_x_axis_scale_ = 1.0
    node.joy_y_axis_index_ = 1
    node.joy_y_axis_scale_ = 1.0
    from builtin_interfaces.msg import Time
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
    MouseJoystickInterface.publish_teleop_command(node, 0.25, -0.5, [1] + [0] * 11)
    msg = node.joy_pub_.publish.call_args.args[0]
    assert isinstance(msg, Joy)
    assert list(msg.axes) == [0.25, -0.5]
    assert list(msg.buttons) == [1] + [0] * 11
