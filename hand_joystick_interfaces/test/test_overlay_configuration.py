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


"""Check consumer-to-producer control-overlay mirroring."""

from pathlib import Path
import runpy

import yaml


_PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_launch_module(launch_name):
    """Load one launch file as a regular Python module."""
    path = _PACKAGE_ROOT / 'launch' / launch_name
    return runpy.run_path(str(path))


def test_shared_yaml_keeps_control_metrics_in_consumer_section_only():
    """Producer sections should contain policy, not copied control metrics."""
    cases = (
        (
            'explorer_hand_parameters.yaml',
            'hand_landmarks_node',
        ),
        (
            'explorer_3d_hand_parameters.yaml',
            'oak_hand_landmarks_node',
        ),
    )
    for config_name, producer_name in cases:
        document = yaml.safe_load(
            (_PACKAGE_ROOT / 'config' / config_name).read_text(
                encoding='utf-8'
            )
        )
        consumer = document['hand_joystick_interface']['ros__parameters']
        producer = document[producer_name]['ros__parameters']
        assert isinstance(producer['show_control_overlay'], bool)
        assert 'dead_zone' in consumer
        assert 'saturation_zone' in consumer
        assert 'normalization_mode' in consumer
        assert 'landmark_index' in consumer
        assert 'overlay_dead_zone' not in producer
        assert 'overlay_saturation_zone' not in producer
        assert 'overlay_normalization_mode' not in producer
        assert 'tracked_landmark_index' not in producer


def test_combined_launches_translate_consumer_controls_to_overlay():
    """Combined launches should derive overlay values from consumer YAML."""
    cases = (
        (
            'usb_cam_hand_joystick_launch.py',
            'explorer_hand_parameters.yaml',
        ),
        (
            'test_video_2d_hand_joystick_launch.py',
            'explorer_hand_parameters.yaml',
        ),
        (
            'oak_hand_joystick_launch.py',
            'explorer_3d_hand_parameters.yaml',
        ),
    )
    for launch_name, config_name in cases:
        config_path = _PACKAGE_ROOT / 'config' / config_name
        document = yaml.safe_load(config_path.read_text(encoding='utf-8'))
        consumer = document['hand_joystick_interface']['ros__parameters']
        launch_module = _load_launch_module(launch_name)

        assert launch_module['_load_consumer_overlay_parameters'](
            config_path
        ) == {
            'overlay_dead_zone': consumer['dead_zone'],
            'overlay_saturation_zone': consumer['saturation_zone'],
            'overlay_normalization_mode': consumer['normalization_mode'],
            'tracked_landmark_index': consumer['landmark_index'],
        }


def test_combined_launches_use_optional_terminal_overrides():
    """Combined launches should not replace YAML with launch-code defaults."""
    for launch_name in (
        'usb_cam_hand_joystick_launch.py',
        'oak_hand_joystick_launch.py',
        'test_video_2d_hand_joystick_launch.py',
    ):
        source = (_PACKAGE_ROOT / 'launch' / launch_name).read_text(
            encoding='utf-8'
        )
        assert "'show_control_overlay'" in source
        assert "'overlay_dead_zone'" in source
        assert "'overlay_saturation_zone'" in source
        assert "'overlay_normalization_mode'" in source
        assert 'default_value=USE_YAML_DEFAULT' in source
        assert "'dead_zone'" in source
        assert "'saturation_zone'" in source
        assert "'normalization_mode'" in source


def test_combined_launches_layer_consumer_yaml_after_producer_yaml():
    """Producer parameters should follow base, integration, terminal order."""
    cases = (
        (
            'usb_cam_hand_joystick_launch.py',
            "LaunchConfiguration('hand_landmarks_params_file')",
            '**producer_control_overrides',
        ),
        (
            'oak_hand_joystick_launch.py',
            "LaunchConfiguration('oak_config_file')",
            'oak_overrides',
        ),
        (
            'test_video_2d_hand_joystick_launch.py',
            '            hand_landmarks_config,',
            '**producer_control_overrides',
        ),
    )
    for launch_name, producer_yaml, terminal_overrides in cases:
        source = (_PACKAGE_ROOT / 'launch' / launch_name).read_text(
            encoding='utf-8'
        )
        producer_index = source.index(producer_yaml)
        integration_index = source.index(
            "LaunchConfiguration('config_file')",
            producer_index,
        )
        overrides_index = source.index(
            terminal_overrides,
            integration_index,
        )
        assert producer_index < integration_index < overrides_index
