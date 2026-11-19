# Copyright 2026 Etienne Moullet
# SPDX-License-Identifier: Apache-2.0
"""Test mapper topic selection and exposing the observed mode."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock

from launch import LaunchContext
import pytest
import yaml

from mouse_joystick_server import MouseJoystickInterface, _TeleopHandler


PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('mouse_launch', PACKAGE / 'launch/mouse_joystick_launch.py')
mouse_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mouse_launch)


@pytest.mark.parametrize('name,topic', [
    ('', '/joystick_mapper/active_mode'),
    ('mouse', '/joystick_mapper/mouse/active_mode'),
    ('hand_2', '/joystick_mapper/hand_2/active_mode'),
])
def test_mapper_name_selects_topic(tmp_path, name, topic):
    config = tmp_path / 'mapper.yaml'
    config.write_text(yaml.safe_dump({'joystick_mapper': {'ros__parameters': {'mapper_name': name}}}))
    assert mouse_launch._mapper_button_overrides(str(config))['active_mode_topic'] == topic


def test_invalid_name_rejected(tmp_path):
    config = tmp_path / 'mapper.yaml'
    config.write_text(yaml.safe_dump({'joystick_mapper': {'ros__parameters': {'mapper_name': 'bad/name'}}}))
    with pytest.raises(ValueError, match='mapper_name'):
        mouse_launch._mapper_button_overrides(str(config))


def test_external_mapper_keeps_interface_configuration(tmp_path, monkeypatch):
    context = LaunchContext()
    context.launch_configurations.update({
        'config_file': 'interface.yaml', 'mapper_config_file': str(tmp_path / 'missing'),
        'start_mapper': 'false',
    })
    monkeypatch.setattr(mouse_launch, 'Node', lambda **kwargs: kwargs)
    assert len(mouse_launch._interface_node(context)[0]['parameters']) == 1


def test_active_mode_updates_and_is_exposed():
    node = Mock()
    node.active_mode_ = None
    handler = object.__new__(_TeleopHandler)
    handler.path = '/active_mode'
    handler.node = node
    handler._send_json = Mock()
    handler.do_GET()
    handler._send_json.assert_called_with({'active_mode': None})
    for name in ('b1', 'custom_mode', ''):
        MouseJoystickInterface._active_mode_callback(node, Mock(data=name))
        handler.do_GET()
        handler._send_json.assert_called_with({'active_mode': name or None})


def test_robot_position_updates_and_is_exposed():
    node = Mock()
    node.robot_position = None
    handler = object.__new__(_TeleopHandler)
    handler.path = '/robot_position'
    handler.node = node
    handler._send_json = Mock()

    handler.do_GET()
    handler._send_json.assert_called_with({'robot_position': None})

    msg = Mock()
    msg.name = ['joint_2', 'joint_1']
    msg.position = [1.0, 0.5]
    MouseJoystickInterface._robot_position_callback(node, msg)
    handler.do_GET()
    handler._send_json.assert_called_with({
        'robot_position': {'joint_2': 1.0, 'joint_1': 0.5},
    })
