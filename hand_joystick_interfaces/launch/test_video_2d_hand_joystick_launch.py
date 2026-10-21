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


"""Launch offline video, hand landmarks, and 2D hand joystick control."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml


USE_YAML_DEFAULT = '__use_yaml__'
CONTROL_OVERLAY_PARAMETERS = (
    ('dead_zone', 'overlay_dead_zone'),
    ('saturation_zone', 'overlay_saturation_zone'),
    ('normalization_mode', 'overlay_normalization_mode'),
    ('landmark_index', 'tracked_landmark_index'),
)


def _parse_bool(value):
    """Parse a resolved ROS launch boolean."""
    return value.lower() in ('1', 'true', 'yes', 'on')


def _load_consumer_overlay_parameters(config_path):
    """Translate available consumer controls to producer overlay parameters."""
    with open(config_path, encoding='utf-8') as config_file:
        document = yaml.safe_load(config_file) or {}

    node_config = document.get('hand_joystick_interface', {})
    consumer_parameters = node_config.get('ros__parameters', {})
    if not isinstance(consumer_parameters, dict):
        return {}

    return {
        producer_name: consumer_parameters[consumer_name]
        for consumer_name, producer_name in CONTROL_OVERLAY_PARAMETERS
        if consumer_name in consumer_parameters
    }


def _create_nodes(context, hand_landmarks_config, video_config):
    """Create video, producer, and consumer nodes with optional overrides."""
    config_path = LaunchConfiguration('config_file').perform(context)
    producer_control_overrides = _load_consumer_overlay_parameters(
        config_path
    )
    consumer_control_overrides = {}

    show_control_overlay = LaunchConfiguration(
        'show_control_overlay'
    ).perform(context)
    if show_control_overlay != USE_YAML_DEFAULT:
        producer_control_overrides['show_control_overlay'] = _parse_bool(
            show_control_overlay
        )

    for argument_name, producer_name, value_type in (
        ('dead_zone', 'overlay_dead_zone', float),
        ('saturation_zone', 'overlay_saturation_zone', float),
        ('normalization_mode', 'overlay_normalization_mode', str),
        ('landmark_index', 'tracked_landmark_index', int),
    ):
        value = LaunchConfiguration(argument_name).perform(context)
        if value == USE_YAML_DEFAULT:
            continue
        converted_value = value_type(value)
        producer_control_overrides[producer_name] = converted_value
        consumer_control_overrides[argument_name] = converted_value

    offline_video_node = Node(
        package='offline_media_publisher',
        executable='video_publisher',
        name='video_publisher',
        output='screen',
        parameters=[
            video_config,
            {
                'folder_path': LaunchConfiguration('folder_path'),
                'fps': LaunchConfiguration('fps'),
            },
        ],
    )

    hand_landmarks_node = Node(
        package='mediapipe_mocap',
        executable='hand_landmarks_node',
        name='hand_landmarks_node',
        output='screen',
        parameters=[
            hand_landmarks_config,
            LaunchConfiguration('config_file'),
            {
                **producer_control_overrides,
            },
        ],
    )

    hand_joystick_node = Node(
        package='hand_joystick_interfaces',
        executable='hand_joystick_node',
        name='hand_joystick_interface',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            consumer_control_overrides,
        ],
    )

    return [
        offline_video_node,
        hand_landmarks_node,
        hand_joystick_node,
    ]


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
    """Create the offline-video 2D hand joystick launch description."""
    hand_joystick_share_dir = get_package_share_directory(
        'hand_joystick_interfaces'
    )
    mediapipe_share_dir = get_package_share_directory('mediapipe_mocap')
    offline_media_share_dir = get_package_share_directory(
        'offline_media_publisher'
    )

    hand_joystick_config = os.path.join(
        hand_joystick_share_dir,
        'config',
        'explorer_hand_parameters.yaml',
    )
    hand_landmarks_config = os.path.join(
        mediapipe_share_dir,
        'config',
        'hand_landmarks_node.yaml',
    )
    video_config = os.path.join(
        offline_media_share_dir,
        'config',
        'video_publisher.yaml',
    )

    arguments = [
        DeclareLaunchArgument(
            'folder_path',
            default_value='',
            description='Path to folder containing videos (required)',
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30',
            description='Publishing rate in Hz (overrides native video FPS)',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=hand_joystick_config,
            description=(
                'Shared YAML with joystick and producer parameter sections'
            ),
        ),
        DeclareLaunchArgument(
            'dead_zone',
            default_value=USE_YAML_DEFAULT,
            description=(
                'Override the YAML joystick and overlay dead-zone radius'
            ),
        ),
        DeclareLaunchArgument(
            'saturation_zone',
            default_value=USE_YAML_DEFAULT,
            description=(
                'Override the YAML joystick and overlay saturation radius'
            ),
        ),
        DeclareLaunchArgument(
            'normalization_mode',
            default_value=USE_YAML_DEFAULT,
            description=(
                'Override the YAML axis or vector normalization policy'
            ),
        ),
        DeclareLaunchArgument(
            'show_control_overlay',
            default_value=USE_YAML_DEFAULT,
            description='Override the YAML control-overlay visibility',
        ),
        DeclareLaunchArgument(
            'landmark_index',
            default_value=USE_YAML_DEFAULT,
            description=(
                'Override the YAML tracked hand-landmark index (0-20)'
            ),
        ),
    ]

    return LaunchDescription([
        *_mapper_arguments(),
        _mapper_node(),
        *arguments,
        OpaqueFunction(
            function=_create_nodes,
            kwargs={
                'hand_landmarks_config': hand_landmarks_config,
                'video_config': video_config,
            },
        ),
    ])
