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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    node = Node(
        package='mouse_joystick_interface',
        executable='mouse_joystick_server.py',
        name='mouse_joystick_interface',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([config_file_arg, node])
