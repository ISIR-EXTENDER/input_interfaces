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
import os
from pathlib import Path
import subprocess
import threading
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from extender_msgs.msg import TeleopCommand
import rclpy
from rclpy.node import Node


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
        """Serve the web joystick page for `/` and `/index.html` requests."""
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
            x = float(payload.get('x', 0.0))
            y = float(payload.get('y', 0.0))
        except (ValueError, TypeError, json.JSONDecodeError):
            self._send_json({'ok': False, 'error': 'invalid_payload'}, status=400)
            return

        x = max(-1.0, min(1.0, x))
        y = max(-1.0, min(1.0, y))
        self.node.publish_teleop_command(x, y)
        self._send_json({'ok': True})

    def log_message(self, fmt: str, *args: object) -> None:
        """Suppress the default per-request HTTP access log."""
        return


class MouseJoystickInterface(Node):
    """ROS 2 node hosting a web joystick and publishing TeleopCommand messages."""

    def __init__(self) -> None:
        """Create the publisher and start the local HTTP server."""
        super().__init__('mouse_joystick_interface')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8765)
        self.declare_parameter('auto_open_browser', True)
        self.declare_parameter('teleop_topic', 'teleop_cmd')

        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        auto_open_browser = (
            self.get_parameter('auto_open_browser').get_parameter_value().bool_value
        )
        teleop_topic = self.get_parameter('teleop_topic').get_parameter_value().string_value

        self.teleop_pub_ = self.create_publisher(TeleopCommand, teleop_topic, 10)

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

    def publish_teleop_command(self, x: float, y: float) -> None:
        """Publish a normalized translation command from joystick axes."""
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

        self.teleop_pub_.publish(teleop_msg)

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
