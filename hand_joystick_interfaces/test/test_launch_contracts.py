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

"""Camera-free tests for combined hand-joystick launch contracts."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import yaml


_PACKAGE_NAME = 'hand_joystick_interfaces'


def _load_launch_module(launch_name):
    """Load one installed launch file as an isolated Python module."""
    launch_path = (
        Path(get_package_share_directory(_PACKAGE_NAME))
        / 'launch'
        / launch_name
    )
    spec = importlib.util.spec_from_file_location(
        f'_hand_joystick_{launch_path.stem}',
        launch_path,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _resolve_launch(launch_name, overrides=None):
    """Resolve launch factories and direct nodes without starting processes."""
    from launch import LaunchContext
    from launch.actions import (
        DeclareLaunchArgument, OpaqueFunction,
    )
    from launch_ros.actions import Node

    module = _load_launch_module(launch_name)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})

    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)

    nodes = []
    for entity in description.entities:
        if isinstance(entity, Node):
            if (entity.condition is None
                    or entity.condition.evaluate(context)):
                nodes.append(entity)
        elif isinstance(entity, OpaqueFunction):
            nodes.extend(entity.execute(context) or [])

    assert nodes
    assert all(isinstance(node, Node) for node in nodes)
    return context, nodes


def _evaluated_parameters(node, context):
    """Evaluate normalized launch parameters without executing the node."""
    from launch_ros.utilities import evaluate_parameters

    # Jazzy exposes no public pre-execution accessor for normalized parameters.
    normalized_parameters = vars(node)['_Node__parameters']
    return evaluate_parameters(context, normalized_parameters)


def _parameters_from_file(path, node_name):
    """Read YAML parameters applicable to one unnamespaced node."""
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    parameters = {}
    for selector in ('/**', node_name, f'/{node_name}'):
        selected = document.get(selector)
        if selected is not None:
            parameters.update(selected.get('ros__parameters', {}))
    return parameters


def _flatten_parameters(parameters, prefix=''):
    """Flatten nested ROS YAML into the dotted names used by nodes."""
    result = {}
    for name, value in parameters.items():
        full_name = prefix + name
        if isinstance(value, dict):
            result.update(_flatten_parameters(value, full_name + '.'))
        else:
            result[full_name] = value
    return result


def _effective_parameters(node, context):
    """Apply parameter files and dictionaries in launch precedence order."""
    effective = {}
    node_name = vars(node)['_Node__node_name']
    for parameter_source in _evaluated_parameters(node, context):
        if isinstance(parameter_source, Path):
            effective.update(_flatten_parameters(
                _parameters_from_file(parameter_source, node_name)))
        else:
            effective.update(parameter_source)
    return effective


def _expanded_remappings(node, context):
    """Evaluate normalized remappings without executing the node."""
    from launch.utilities import perform_substitutions

    normalized_remappings = vars(node)['_Node__remappings']
    return [
        (
            perform_substitutions(context, source),
            perform_substitutions(context, destination),
        )
        for source, destination in normalized_remappings
    ]


def _node_by_executable(nodes, executable):
    """Return the single node matching an executable name."""
    matching = [
        node for node in nodes if node.node_executable == executable]
    assert len(matching) == 1
    return matching[0]


def _assert_executables_resolve(nodes, context):
    """Resolve every launch node executable through the ament index."""
    from launch_ros.substitutions import ExecutableInPackage

    for node in nodes:
        executable_path = ExecutableInPackage(
            package=node.node_package,
            executable=node.node_executable,
        ).perform(context)
        assert Path(executable_path).is_file()


def _assert_control_mirror(producer, consumer):
    """Check consumer controls against producer overlay values."""
    assert producer['overlay_dead_zone'] == consumer['dead_zone']
    assert producer['overlay_saturation_zone'] == consumer['saturation_zone']
    assert (
        producer['overlay_normalization_mode']
        == consumer['normalization_mode']
    )
    assert producer['tracked_landmark_index'] == consumer['landmark_index']


def test_installed_combined_launch_assets_resolve():
    """Load every installed hand-joystick launch description."""
    package_share = Path(get_package_share_directory(_PACKAGE_NAME))
    for launch_name in (
        'hand_joystick_launch.py',
        'usb_cam_hand_joystick_launch.py',
        'test_video_2d_hand_joystick_launch.py',
        'oak_hand_joystick_launch.py',
    ):
        assert (package_share / 'launch' / launch_name).is_file()
        context, nodes = _resolve_launch(launch_name)
        _assert_executables_resolve(nodes, context)


def test_consumer_only_launch_resolves_default_configuration():
    """Resolve the consumer-only launch and its default YAML values."""
    context, nodes = _resolve_launch('hand_joystick_launch.py')
    consumer_node = _node_by_executable(nodes, 'hand_joystick_node')
    consumer = _effective_parameters(consumer_node, context)

    assert consumer_node.node_package == _PACKAGE_NAME
    assert consumer['dead_zone'] == 0.05
    assert consumer['saturation_zone'] == 0.4
    assert consumer['normalization_mode'] == 'vector'
    assert consumer['landmark_index'] == 0
    assert consumer['joy_output_topic'] == '/hand_joy'
    assert consumer['cartesian_command_topic'] == '/joystick_cartesian_command'


def test_usb_combined_launch_resolves_defaults_and_topic_wiring():
    """Resolve USB camera, producer, and consumer defaults in memory."""
    context, nodes = _resolve_launch('usb_cam_hand_joystick_launch.py')
    camera_node = _node_by_executable(nodes, 'usb_cam_node_exe')
    producer_node = _node_by_executable(nodes, 'hand_landmarks_node')
    consumer_node = _node_by_executable(nodes, 'hand_joystick_node')

    camera = _effective_parameters(camera_node, context)
    producer = _effective_parameters(producer_node, context)
    consumer = _effective_parameters(consumer_node, context)

    assert camera['video_device'] == '/dev/video0'
    assert camera['framerate'] == 30.0
    assert _expanded_remappings(camera_node, context) == [
        ('image_raw', '/camera/color/image_raw'),
        ('camera_info', '/camera/color/camera_info'),
    ]

    assert producer['image_topic'] == '/camera/color/image_raw'
    assert producer['selfie_mode'] is True
    assert producer['visualize'] is False
    assert producer['show_control_overlay'] is True
    _assert_control_mirror(producer, consumer)


def test_oak_combined_launch_resolves_defaults_and_control_mirror():
    """Resolve the metric producer and consumer defaults in memory."""
    context, nodes = _resolve_launch('oak_hand_joystick_launch.py')
    producer = _effective_parameters(
        _node_by_executable(nodes, 'oak_hand_landmarks_node'),
        context,
    )
    consumer = _effective_parameters(
        _node_by_executable(nodes, 'hand_joystick_node'),
        context,
    )

    assert producer['camera_frame_id'] == 'oak_rgb_camera_optical_frame'
    assert producer['fps'] == 50.0
    assert producer['visualize'] is True
    assert producer['show_control_overlay'] is True
    assert producer['auto_reference_on_first_detection'] is True
    _assert_control_mirror(producer, consumer)


def test_video_combined_launch_resolves_defaults_and_control_mirror():
    """Resolve offline video, producer, and consumer defaults in memory."""
    context, nodes = _resolve_launch(
        'test_video_2d_hand_joystick_launch.py'
    )
    video = _effective_parameters(
        _node_by_executable(nodes, 'video_publisher'),
        context,
    )
    producer = _effective_parameters(
        _node_by_executable(nodes, 'hand_landmarks_node'),
        context,
    )
    consumer = _effective_parameters(
        _node_by_executable(nodes, 'hand_joystick_node'),
        context,
    )

    assert video['folder_path'] == ''
    assert video['fps'] == 30
    assert producer['image_topic'] == '/camera/color/image_raw'
    assert producer['visualize'] is True
    assert producer['show_control_overlay'] is True
    _assert_control_mirror(producer, consumer)


def test_terminal_controls_override_consumer_and_overlay_together():
    """Check terminal precedence for all three combined pipelines."""
    overrides = {
        'show_control_overlay': 'false',
        'dead_zone': '0.08',
        'saturation_zone': '0.35',
        'normalization_mode': 'axis',
        'landmark_index': '8',
    }
    for launch_name, producer_executable in (
        ('usb_cam_hand_joystick_launch.py', 'hand_landmarks_node'),
        ('test_video_2d_hand_joystick_launch.py', 'hand_landmarks_node'),
        ('oak_hand_joystick_launch.py', 'oak_hand_landmarks_node'),
    ):
        context, nodes = _resolve_launch(launch_name, overrides)
        producer = _effective_parameters(
            _node_by_executable(nodes, producer_executable),
            context,
        )
        consumer = _effective_parameters(
            _node_by_executable(nodes, 'hand_joystick_node'),
            context,
        )

        assert producer['show_control_overlay'] is False
        assert consumer['dead_zone'] == 0.08
        assert consumer['saturation_zone'] == 0.35
        assert consumer['normalization_mode'] == 'axis'
        assert consumer['landmark_index'] == 8
        _assert_control_mirror(producer, consumer)


_PIPELINE_LAUNCHES = (
    'hand_joystick_launch.py',
    'usb_cam_hand_joystick_launch.py',
    'oak_hand_joystick_launch.py',
    'test_video_2d_hand_joystick_launch.py',
)


def test_hand_mapper_configuration_and_topics():
    """Every pipeline connects raw hand axes to the manager input."""
    for launch_name in _PIPELINE_LAUNCHES:
        context, nodes = _resolve_launch(launch_name)
        consumer = _effective_parameters(
            _node_by_executable(nodes, 'hand_joystick_node'), context)
        mapper = _effective_parameters(
            _node_by_executable(nodes, 'joystick_mapper_node'), context)
        assert mapper['joy_topic'] == consumer['joy_output_topic']
        assert mapper['output_topic'] == consumer['cartesian_command_topic']
        assert mapper['output_topic'] == '/joystick_cartesian_command'
        assert mapper['deadzone'] == 0.0
        assert mapper['modes.names'] == ['translation']
        frame = mapper['modes.translation.angular_output_frame_id']
        assert frame == 'base_link'
        for axis, index, scale in (
                ('x', 0, 1.0), ('y', 2, 1.0), ('z', 1, -1.0)):
            prefix = 'modes.translation.axes.linear_' + axis
            assert mapper[prefix + '.index'] == index
            assert mapper[prefix + '.scale'] == scale
            angular = 'modes.translation.axes.angular_' + axis + '.index'
            assert mapper[angular] == -1
        for button in ('local_mode', 'jaco', 'snake', 'home', 'gripper'):
            assert mapper[button + '_button_index'] == -1


def test_mapper_can_be_owned_by_robot_bringup():
    """Disable only the mapper while keeping each hand pipeline available."""
    for launch_name in _PIPELINE_LAUNCHES:
        context, nodes = _resolve_launch(
            launch_name, {'start_mapper': 'false'})
        assert not any(
            node.node_executable == 'joystick_mapper_node' for node in nodes)
        _node_by_executable(nodes, 'hand_joystick_node')


def test_mapper_file_override_is_resolved(tmp_path):
    """Explicit mapper configuration overrides the bundled default."""
    config = tmp_path / 'mapper.yaml'
    config.write_text(yaml.safe_dump({'joystick_mapper': {'ros__parameters': {
        'joy_topic': '/custom_hand_joy', 'output_topic': '/custom_command',
        'deadzone': 0.0,
    }}}))
    for launch_name in _PIPELINE_LAUNCHES:
        context, nodes = _resolve_launch(
            launch_name, {'mapper_config_file': str(config)})
        mapper = _effective_parameters(
            _node_by_executable(nodes, 'joystick_mapper_node'), context)
        assert mapper['joy_topic'] == '/custom_hand_joy'
        assert mapper['output_topic'] == '/custom_command'
