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


"""Launch the hand joystick consumer and Cartesian mapper."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _mapper_arguments():
    """Declare optional mapper startup and its configuration."""
    return [
        DeclareLaunchArgument(
            'start_mapper', default_value='true',
            description='Start the hand mapper; disable when bringup owns it',
        ),
        DeclareLaunchArgument(
            'mapper_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hand_joystick_interfaces'),
                'config', 'hand_joystick_mapper.yaml',
            ]),
            description='Hand joystick mapper YAML parameter file',
        ),
    ]


def _mapper_node():
    """Map normalized hand Joy axes to Cartesian commands."""
    return Node(
        package='joystick_mapper', executable='joystick_mapper_node',
        name='joystick_mapper', output='screen',
        parameters=[LaunchConfiguration('mapper_config_file')],
        condition=IfCondition(LaunchConfiguration('start_mapper')),
    )


def generate_launch_description():
    """Create the hand input and optional mapper launch description."""
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('hand_joystick_interfaces'),
            'config',
            'explorer_hand_parameters.yaml'
        ]),
        description='Path to the configuration file (2D mode)'
    )

    # Hand joystick interface node
    hand_joystick_node = Node(
        package='hand_joystick_interfaces',
        executable='hand_joystick_node',
        name='hand_joystick_interface',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        *_mapper_arguments(),
        _mapper_node(),
        config_file_arg,
        hand_joystick_node
    ])
