# Copyright 2026 ISIR-EXTENDER
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Exercise installed hand and mapper nodes with synthetic landmark clouds."""

from pathlib import Path
import signal
import subprocess
import time
import uuid

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)

from geometry_msgs.msg import Point32, TwistStamped

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import Joy, PointCloud

from visualization_msgs.msg import Marker

import yaml


class Pipeline:
    """Observe all stages in a private topic namespace."""

    def __init__(self, node, publisher, processes, executor):
        """Keep observer state and child processes together."""
        self.node = node
        self.executor = executor
        self.publisher = publisher
        self.processes = processes
        self.joys = []
        self.commands = []
        self.markers = []
        self.subscriptions = []

    def wait(self, predicate, timeout=5.0, on_tick=None):
        """Spin until a condition succeeds or a child fails."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert all(process.poll() is None for process in self.processes)
            if on_tick is not None:
                on_tick()
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError('Timed out waiting for hand pipeline output')

    def pump(self, seconds):
        """Receive callbacks for a bounded interval."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def cloud(self, xyz, count=21):
        """Put displacement only in the configured wrist landmark."""
        message = PointCloud()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.points = [Point32() for _ in range(count)]
        if count:
            message.points[0] = Point32(x=float(xyz[0]), y=float(xyz[1]),
                                        z=float(xyz[2]))
        return message

    def send(self, xyz):
        """Publish once and wait for Joy, mapped command, and marker."""
        self.pump(0.03)
        counts = (len(self.joys), len(self.commands), len(self.markers))
        self.publisher.publish(self.cloud(xyz))
        self.wait(lambda: all(
            len(messages) > count
            for messages, count in zip(
                (self.joys, self.commands, self.markers), counts)))
        return self.joys[-1], self.commands[-1], self.markers[-1]


@pytest.fixture(params=['axis', 'vector'])
def pipeline(request, tmp_path):
    """Run fresh installed executables without camera or robot dependencies."""
    namespace = '/hand_test_' + uuid.uuid4().hex
    share = Path(get_package_share_directory('hand_joystick_interfaces'))
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('observer', namespace=namespace, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publisher = node.create_publisher(
        PointCloud, namespace + '/hand_landmarks', 10)
    mapper_config = tmp_path / 'mapper.yaml'
    document = yaml.safe_load(
        (share / 'config/hand_joystick_mapper.yaml').read_text())
    mapper_config.write_text(yaml.safe_dump({
        namespace + '/joystick_mapper': document['joystick_mapper'],
    }))
    processes = []
    logs = []
    try:
        for package, executable, arguments in (
            ('hand_joystick_interfaces', 'hand_joystick_node', [
                '-p', 'landmark_index:=0',
                '-p', 'dead_zone:=0.05', '-p', 'saturation_zone:=0.25',
                '-p', 'normalization_mode:=' + request.param,
                '-p', 'lost_hand_cooldown_sec:=0.3',
                '-p', 'joy_output_topic:=' + namespace + '/joy',
                '-p', 'cartesian_command_topic:=' + namespace + '/command',
                '-p', 'cartesian_command_marker_topic:=' + namespace + '/marker',
            ]),
            ('joystick_mapper', 'joystick_mapper_node', [
                '--params-file', str(mapper_config),
                '-p', 'joy_topic:=' + namespace + '/joy',
                '-p', 'output_topic:=' + namespace + '/command',
                '-p', 'mode_request_topic:=' + namespace + '/mode_request',
                '-p', 'gripper_command_topic:=' + namespace + '/gripper',
            ]),
        ):
            binary = (Path(get_package_prefix(package)) / 'lib' / package
                      / executable)
            log = (tmp_path / (executable + '.log')).open('w+')
            logs.append(log)
            processes.append(subprocess.Popen(
                [str(binary), '--ros-args', '-r', '__ns:=' + namespace,
                 *arguments], stdout=log, stderr=subprocess.STDOUT))
        observer = Pipeline(node, publisher, processes, executor)
        for message_type, topic, output in (
            (Joy, '/joy', observer.joys),
            (TwistStamped, '/command', observer.commands),
            (Marker, '/marker', observer.markers),
        ):
            observer.subscriptions.append(node.create_subscription(
                message_type, namespace + topic, output.append, 10))
        observer.wait(lambda: (
            publisher.get_subscription_count() == 1
            and node.count_subscribers(namespace + '/joy') >= 2
            and node.count_subscribers(namespace + '/command') >= 2
            and node.count_publishers(namespace + '/marker') == 1))
        # Graph discovery can precede DDS reader/writer matching. Exchange
        # neutral samples until every stage has actually received data.
        observer.wait(
            lambda: observer.joys and observer.commands and observer.markers,
            on_tick=lambda: publisher.publish(observer.cloud((0.0, 0.0, 0.0))))
        observer.pump(0.05)
        yield request.param, observer
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        for log in logs:
            log.seek(0)
            print(log.read())
            log.close()


def _assert_output(joy, command, marker, expected):
    """Check transport shape, Explorer mapping, and mapped visualization."""
    assert list(joy.axes) == pytest.approx(expected, abs=1e-6)
    assert list(joy.buttons) == []
    assert joy.header.stamp.sec > 0
    linear = command.twist.linear
    mapped = [expected[0], expected[2], -expected[1]]
    assert [linear.x, linear.y, linear.z] == pytest.approx(
        mapped, abs=1e-6)
    angular = command.twist.angular
    assert [angular.x, angular.y, angular.z] == [0.0, 0.0, 0.0]
    assert command.header.frame_id == 'base_link'
    assert command.header.stamp.sec > 0
    assert marker.header.frame_id == 'base_link'
    assert marker.ns == 'cartesian_command'
    assert len(marker.points) == 2
    endpoint = marker.points[1]
    assert [endpoint.x, endpoint.y, endpoint.z] == pytest.approx(
        mapped, abs=1e-6)


def test_normalization_mapping_and_zero_commands(pipeline):
    """Preserve normalization and signs without a second dead zone."""
    mode, observer = pipeline
    cases = [
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((0.05, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((0.07, 0.0, 0.0), (0.1, 0.0, 0.0)),
        ((0.15, 0.0, 0.0), (0.5, 0.0, 0.0)),
        ((0.25, 0.0, 0.0), (1.0, 0.0, 0.0)),
    ]
    if mode == 'axis':
        cases.extend([
            ((0.15, -0.30, 0.05), (0.5, -1.0, 0.0)),
            ((0.0, 0.0, -0.15), (0.0, 0.0, -0.5)),
            ((0.04, 0.04, 0.04), (0.0, 0.0, 0.0)),
        ])
    else:
        cases.extend([
            ((0.12, 0.16, 0.0), (0.45, 0.6, 0.0)),
            ((0.6, 0.8, 0.0), (0.6, 0.8, 0.0)),
            ((0.0, 0.0, -0.15), (0.0, 0.0, -0.5)),
        ])
    for displacement, expected in cases:
        _assert_output(*observer.send(displacement), expected)


def test_invalid_clouds_watchdog_and_recovery(pipeline):
    """Bad clouds must not renew freshness; one zero stops stale motion."""
    _, observer = pipeline
    _assert_output(*observer.send((0.15, 0.0, 0.0)), (0.5, 0.0, 0.0))
    start = len(observer.joys)
    # Repeated malformed/non-finite input must still trigger the watchdog.
    for _ in range(8):
        observer.publisher.publish(observer.cloud((0.15, 0.0, 0.0), count=20))
        observer.publisher.publish(observer.cloud((float('nan'), 0.0, 0.0)))
        observer.publisher.publish(observer.cloud((float('inf'), 0.0, 0.0)))
        observer.pump(0.06)
    observer.wait(lambda: len(observer.joys) > start
                  and observer.commands[-1].twist.linear.x == 0.0
                  and observer.markers[-1].points[1].x == 0.0)
    assert len(observer.joys) == start + 1
    _assert_output(observer.joys[-1], observer.commands[-1],
                   observer.markers[-1], (0.0, 0.0, 0.0))
    observer.pump(0.4)
    assert len(observer.joys) == start + 1
    _assert_output(*observer.send((-0.15, 0.0, 0.0)), (-0.5, 0.0, 0.0))
    observer.wait(lambda: observer.joys[-1].axes[0] == 0.0, timeout=1.0)
