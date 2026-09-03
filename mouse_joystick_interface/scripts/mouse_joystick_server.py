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

"""Browser-based mouse joystick interface for ROS 2 teleoperation commands."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import subprocess
import threading
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from extender_msgs.msg import TeleopCommand
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


_OUTPUT_TYPE_TWIST = 'twist'
_OUTPUT_TYPE_JOY = 'joy'
_SUPPORTED_OUTPUT_TYPES = (_OUTPUT_TYPE_TWIST, _OUTPUT_TYPE_JOY)
_JOY_BUTTON_COUNT = 12
_JOY_BUTTON_TYPE_PRESS = 'press'
_JOY_BUTTON_TYPE_TOGGLE = 'toggle'
_SUPPORTED_JOY_BUTTON_TYPES = (_JOY_BUTTON_TYPE_PRESS, _JOY_BUTTON_TYPE_TOGGLE)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp a floating-point value to an inclusive range."""
    return max(minimum, min(maximum, value))


def _validate_output_type(output_type: str) -> None:
    """Reject unsupported output message selections."""
    if output_type not in _SUPPORTED_OUTPUT_TYPES:
        supported = ', '.join(_SUPPORTED_OUTPUT_TYPES)
        raise ValueError(
            f'Invalid output_type {output_type!r}; expected one of: {supported}'
        )


def _validate_joy_mapping(
    axis_count: int,
    x_axis_index: int,
    x_axis_scale: float,
    y_axis_index: int,
    y_axis_scale: float,
) -> None:
    """Validate the configured mapping from browser coordinates to Joy axes."""
    if axis_count <= 0:
        raise ValueError('joy_axis_count must be greater than zero')

    for axis_name, axis_index in (
        ('joy_axes.x.index', x_axis_index),
        ('joy_axes.y.index', y_axis_index),
    ):
        if axis_index < 0 or axis_index >= axis_count:
            raise ValueError(
                f'{axis_name} must be in [0, {axis_count - 1}], got {axis_index}'
            )

    if x_axis_index == y_axis_index:
        raise ValueError('joy_axes.x.index and joy_axes.y.index must be different')

    for scale_name, scale in (
        ('joy_axes.x.scale', x_axis_scale),
        ('joy_axes.y.scale', y_axis_scale),
    ):
        if not math.isfinite(scale):
            raise ValueError(f'{scale_name} must be finite, got {scale}')


def _build_joy_axes(
    x: float,
    y: float,
    axis_count: int,
    x_axis_index: int,
    x_axis_scale: float,
    y_axis_index: int,
    y_axis_scale: float,
) -> list[float]:
    """Build a zero-filled Joy axis array from normalized browser coordinates."""
    axes = [0.0] * axis_count
    axes[x_axis_index] = _clamp(float(x) * x_axis_scale, -1.0, 1.0)
    axes[y_axis_index] = _clamp(float(y) * y_axis_scale, -1.0, 1.0)
    return axes


def _validate_joy_buttons(buttons: object) -> list[int]:
    """Return a validated 12-entry Joy button array."""
    if not isinstance(buttons, list) or len(buttons) != _JOY_BUTTON_COUNT:
        raise ValueError(f'buttons must be a list of exactly {_JOY_BUTTON_COUNT} entries')

    if any(type(value) is not int or value not in (0, 1) for value in buttons):
        raise ValueError('buttons entries must be integers containing only 0 or 1')

    return list(buttons)


def _validate_joy_button_types(button_types: object) -> list[str]:
    """Return a validated, index-ordered list of Joy button behaviors."""
    if not isinstance(button_types, list) or len(button_types) != _JOY_BUTTON_COUNT:
        raise ValueError(
            f'joy button types must be a list of exactly {_JOY_BUTTON_COUNT} entries'
        )

    validated_types = []
    for index, button_type in enumerate(button_types):
        if type(button_type) is not str or button_type not in _SUPPORTED_JOY_BUTTON_TYPES:
            supported = ', '.join(_SUPPORTED_JOY_BUTTON_TYPES)
            raise ValueError(
                f'joy_buttons.button_{index}.type must be one of: {supported}'
            )
        validated_types.append(button_type)

    return validated_types


def _build_runtime_config(output_type: str, joy_button_types: list[str]) -> dict:
    """Build the browser-visible configuration for the selected output type."""
    config = {'output_type': output_type}
    if output_type == _OUTPUT_TYPE_JOY:
        config['joy_button_types'] = _validate_joy_button_types(joy_button_types)
    return config


def _build_teleop_command(x: float, y: float) -> TeleopCommand:
    """Build the existing translation TeleopCommand output."""
    teleop_msg = TeleopCommand()
    if hasattr(TeleopCommand, 'TRANSLATION'):
        # Older generated message bindings may not expose constants; keep
        # publishing useful twist data even then.
        teleop_msg.mode = TeleopCommand.TRANSLATION

    teleop_msg.twist.linear.x = float(x)
    teleop_msg.twist.linear.y = float(y)
    teleop_msg.twist.linear.z = 0.0
    teleop_msg.twist.angular.x = 0.0
    teleop_msg.twist.angular.y = 0.0
    teleop_msg.twist.angular.z = 0.0
    return teleop_msg


def _build_joy_message(
    x: float,
    y: float,
    stamp,
    axis_count: int,
    x_axis_index: int,
    x_axis_scale: float,
    y_axis_index: int,
    y_axis_scale: float,
    buttons: Optional[list[int]] = None,
) -> Joy:
    """Build a timestamped Joy output using the configured axis mapping."""
    joy_msg = Joy()
    joy_msg.header.stamp = stamp
    joy_msg.axes = _build_joy_axes(
        x,
        y,
        axis_count,
        x_axis_index,
        x_axis_scale,
        y_axis_index,
        y_axis_scale,
    )
    joy_msg.buttons = (
        [0] * _JOY_BUTTON_COUNT if buttons is None else _validate_joy_buttons(buttons)
    )
    return joy_msg


class _TeleopHandler(BaseHTTPRequestHandler):
    """HTTP handler serving the joystick UI and receiving teleop updates."""

    node: Optional['MouseJoystickInterface'] = None
    web_root: Optional[Path] = None

    def _send_json(self, payload: dict, status: int = 200) -> None:
        """Send a JSON response to the browser."""
        data = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_html(self, html: bytes) -> None:
        """Send the joystick page HTML."""
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def do_GET(self) -> None:
        """Serve the web joystick page or its runtime output configuration."""
        if self.path == '/config':
            if self.node is None:
                self._send_json({'ok': False, 'error': 'server_not_ready'}, status=503)
                return

            self._send_json(
                _build_runtime_config(
                    self.node.output_type_,
                    self.node.joy_button_types_,
                )
            )
            return

        if self.path not in ['/', '/index.html']:
            self.send_error(404)
            return

        if self.web_root is None:
            self.send_error(500)
            return

        index_path = self.web_root / 'index.html'
        if not index_path.exists():
            self.send_error(404)
            return

        self._send_html(index_path.read_bytes())

    def do_POST(self) -> None:
        """Accept a joystick command payload and publish it through ROS."""
        if self.path != '/teleop':
            self.send_error(404)
            return

        if self.node is None:
            self._send_json({'ok': False, 'error': 'server_not_ready'}, status=503)
            return

        try:
            content_length = int(self.headers.get('Content-Length', '0'))
            body = self.rfile.read(content_length) if content_length > 0 else b'{}'
            payload = json.loads(body.decode('utf-8'))
            if not isinstance(payload, dict):
                raise ValueError('payload must be a JSON object')
            x = float(payload.get('x', 0.0))
            y = float(payload.get('y', 0.0))
            buttons = None
            if self.node.output_type_ == _OUTPUT_TYPE_JOY:
                buttons = (
                    [0] * _JOY_BUTTON_COUNT
                    if 'buttons' not in payload
                    else _validate_joy_buttons(payload['buttons'])
                )
        except (ValueError, TypeError, json.JSONDecodeError):
            self._send_json({'ok': False, 'error': 'invalid_payload'}, status=400)
            return

        x = max(-1.0, min(1.0, x))
        y = max(-1.0, min(1.0, y))
        self.node.publish_teleop_command(x, y, buttons)
        self._send_json({'ok': True})

    def log_message(self, fmt: str, *args: object) -> None:
        """Suppress the default per-request HTTP access log."""
        return


class MouseJoystickInterface(Node):
    """ROS 2 node hosting a web joystick and publishing configured commands."""

    def __init__(self) -> None:
        """Create the publisher and start the local HTTP server."""
        super().__init__('mouse_joystick_interface')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8765)
        self.declare_parameter('auto_open_browser', True)
        self.declare_parameter('output_type', _OUTPUT_TYPE_TWIST)
        self.declare_parameter('teleop_topic', 'teleop_cmd')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('joy_axis_count', 2)
        self.declare_parameter('joy_axes.x.index', 0)
        self.declare_parameter('joy_axes.x.scale', 1.0)
        self.declare_parameter('joy_axes.y.index', 1)
        self.declare_parameter('joy_axes.y.scale', 1.0)
        for button_index in range(_JOY_BUTTON_COUNT):
            self.declare_parameter(
                f'joy_buttons.button_{button_index}.type',
                _JOY_BUTTON_TYPE_PRESS,
            )

        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        auto_open_browser = (
            self.get_parameter('auto_open_browser').get_parameter_value().bool_value
        )
        self.output_type_ = self.get_parameter('output_type').value
        teleop_topic = self.get_parameter('teleop_topic').get_parameter_value().string_value
        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.joy_axis_count_ = self.get_parameter('joy_axis_count').value
        self.joy_x_axis_index_ = self.get_parameter('joy_axes.x.index').value
        self.joy_x_axis_scale_ = self.get_parameter('joy_axes.x.scale').value
        self.joy_y_axis_index_ = self.get_parameter('joy_axes.y.index').value
        self.joy_y_axis_scale_ = self.get_parameter('joy_axes.y.scale').value
        self.joy_button_types_ = [
            self.get_parameter(f'joy_buttons.button_{button_index}.type').value
            for button_index in range(_JOY_BUTTON_COUNT)
        ]

        _validate_output_type(self.output_type_)

        self.teleop_pub_ = None
        self.joy_pub_ = None
        if self.output_type_ == _OUTPUT_TYPE_TWIST:
            self.teleop_pub_ = self.create_publisher(TeleopCommand, teleop_topic, 10)
        else:
            _validate_joy_mapping(
                self.joy_axis_count_,
                self.joy_x_axis_index_,
                self.joy_x_axis_scale_,
                self.joy_y_axis_index_,
                self.joy_y_axis_scale_,
            )
            self.joy_button_types_ = _validate_joy_button_types(self.joy_button_types_)
            self.joy_pub_ = self.create_publisher(Joy, joy_topic, 10)

        self.web_root_ = self._resolve_web_root()
        # BaseHTTPRequestHandler instances are constructed by the HTTP server,
        # so shared ROS/web context is attached to the handler class.
        _TeleopHandler.node = self
        _TeleopHandler.web_root = self.web_root_

        self.server_ = ThreadingHTTPServer((host, int(port)), _TeleopHandler)
        self.server_thread_ = threading.Thread(target=self.server_.serve_forever, daemon=True)
        self.server_thread_.start()

        self.url_ = f'http://{host}:{port}/'
        self.get_logger().info(f'Mouse joystick UI available at: {self.url_}')

        if auto_open_browser:
            self._open_browser(self.url_)

    def _resolve_web_root(self) -> Path:
        """Return the directory containing the installed browser UI assets."""
        try:
            share_path = Path(get_package_share_directory('mouse_joystick_interface'))
            return share_path / 'web'
        except Exception:
            return Path(__file__).resolve().parent.parent / 'web'

    def _open_browser(self, url: str) -> None:
        """Open the joystick UI in the desktop browser when possible."""
        if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
            self.get_logger().info(f'No graphical display detected. Open manually: {url}')
            return

        try:
            subprocess.Popen(
                ['xdg-open', url],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as ex:
            self.get_logger().warn(f'Failed to auto-open browser: {ex}')

    def publish_teleop_command(
        self,
        x: float,
        y: float,
        buttons: Optional[list[int]] = None,
    ) -> None:
        """Publish normalized browser coordinates using the selected output type."""
        if self.output_type_ == _OUTPUT_TYPE_JOY:
            joy_msg = _build_joy_message(
                x,
                y,
                self.get_clock().now().to_msg(),
                self.joy_axis_count_,
                self.joy_x_axis_index_,
                self.joy_x_axis_scale_,
                self.joy_y_axis_index_,
                self.joy_y_axis_scale_,
                buttons,
            )
            self.joy_pub_.publish(joy_msg)
            return

        self.teleop_pub_.publish(_build_teleop_command(x, y))

    def shutdown_server(self) -> None:
        """Stop the HTTP server and release its listening socket."""
        if hasattr(self, 'server_') and self.server_ is not None:
            self.server_.shutdown()
            self.server_.server_close()


def main(args=None) -> None:
    """Run the mouse joystick interface node."""
    rclpy.init(args=args)
    node = MouseJoystickInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_server()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
