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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


# Keep these defaults aligned with JoystickMapper::declareButton.
_BUTTON_DEFAULT_MODES = {
    'local_mode': 'toggle',
    'jaco': 'toggle',
    'snake': 'toggle',
    'home': 'trigger',
    'gripper': 'toggle',
}
_BUTTON_COUNT = 12


def _mapper_button_overrides(config_path):
    """Derive interface button types and labels from the mapper parameters."""
    try:
        with open(config_path, encoding='utf-8') as config_file:
            config = yaml.safe_load(config_file)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'Cannot read mapper configuration {config_path}: {exc}') from exc
    try:
        params = config['joystick_mapper']['ros__parameters']
        if not isinstance(params, dict):
            raise TypeError('ros__parameters must be a mapping')
    except (KeyError, TypeError) as exc:
        raise ValueError(
            f'Mapper configuration {config_path} must contain '
            'joystick_mapper.ros__parameters'
        ) from exc

    assignments = {}
    for name, default_mode in _BUTTON_DEFAULT_MODES.items():
        index = params.get(f'{name}_button_index', -1)
        if type(index) is not int or index < -1 or index >= _BUTTON_COUNT:
            raise ValueError(f'Invalid {name}_button_index {index!r}; expected -1 or 0-11')
        if index == -1:
            continue
        mode = params.get(f'{name}_button_mode', default_mode)
        if not isinstance(mode, str):
            raise ValueError(f'{name}_button_mode must be a string')
        mode = mode.lower().replace('-', '_')
        if mode not in ('toggle', 'trigger', 'hold', 'momentary', 'pressed'):
            mode = default_mode
        button_type = 'toggle' if mode in ('hold', 'momentary', 'pressed') else 'press'
        if index in assignments and assignments[index][0] != button_type:
            raise ValueError(
                f'Button {index} has conflicting interface types for '
                f'{assignments[index][1]} and {name}'
            )
        if index in assignments:
            assignments[index][1].append(name)
        else:
            assignments[index] = (button_type, [name])
    overrides = {}
    for index, (button_type, names) in assignments.items():
        overrides[f'joy_buttons.button_{index}.type'] = button_type
        overrides[f'joy_buttons.button_{index}.label'] = ' / '.join(names)
    return overrides


def _interface_node(context):
    """Apply mapper-derived parameters only when this launch starts the mapper."""
    parameters = [LaunchConfiguration('config_file')]
    if IfCondition(LaunchConfiguration('start_mapper')).evaluate(context):
        config_path = LaunchConfiguration('mapper_config_file').perform(context)
        parameters.append(_mapper_button_overrides(config_path))
    return [Node(
        package='mouse_joystick_interface',
        executable='mouse_joystick_server.py',
        name='mouse_joystick_interface',
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for the mouse joystick web interface."""
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution(
            [
                FindPackageShare('mouse_joystick_interface'),
                'config',
                'mouse_joystick_params.yaml',
            ]
        ),
        description='Path to the mouse joystick interface parameters file',
    )
    mapper_config_file_arg = DeclareLaunchArgument(
        'mapper_config_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('mouse_joystick_interface'), 'config',
             'mouse_joystick_mapper.yaml']
        ),
        description='Mouse joystick mapper YAML parameter file',
    )
    start_mapper_arg = DeclareLaunchArgument(
        'start_mapper', default_value='true',
        description='Start the mouse mapper; disable when bringup owns it',
    )
    mapper_node = Node(
        package='joystick_mapper',
        executable='joystick_mapper_node',
        name='joystick_mapper',
        output='screen',
        parameters=[LaunchConfiguration('mapper_config_file')],
        condition=IfCondition(LaunchConfiguration('start_mapper')),
    )

    return LaunchDescription([
        config_file_arg, start_mapper_arg, mapper_config_file_arg,
        OpaqueFunction(function=_interface_node), mapper_node,
    ])
