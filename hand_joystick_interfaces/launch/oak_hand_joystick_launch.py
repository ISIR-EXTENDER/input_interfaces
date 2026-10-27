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


"""Launch OAK hand landmarks and 3D hand joystick control."""

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


def _create_nodes(context):
    """Create producer and consumer nodes with explicit overrides."""
    config_path = LaunchConfiguration('config_file').perform(context)
    oak_overrides = _load_consumer_overlay_parameters(config_path)
    joystick_overrides = {}

    fps = LaunchConfiguration('fps').perform(context)
    if fps != USE_YAML_DEFAULT:
        oak_overrides['fps'] = float(fps)

    rgb_width = LaunchConfiguration('rgb_width').perform(context)
    rgb_height = LaunchConfiguration('rgb_height').perform(context)
    if rgb_width != USE_YAML_DEFAULT:
        oak_overrides['rgb_width'] = int(rgb_width)
    if rgb_height != USE_YAML_DEFAULT:
        oak_overrides['rgb_height'] = int(rgb_height)

    visualize = LaunchConfiguration('visualize').perform(context)
    if visualize != USE_YAML_DEFAULT:
        oak_overrides['visualize'] = _parse_bool(visualize)

    window_name = LaunchConfiguration('window_name').perform(context)
    if window_name != USE_YAML_DEFAULT:
        oak_overrides['window_name'] = window_name

    show_control_overlay = LaunchConfiguration(
        'show_control_overlay'
    ).perform(context)
    if show_control_overlay != USE_YAML_DEFAULT:
        oak_overrides['show_control_overlay'] = _parse_bool(
            show_control_overlay
        )

    dead_zone = LaunchConfiguration('dead_zone').perform(context)
    if dead_zone != USE_YAML_DEFAULT:
        dead_zone_value = float(dead_zone)
        oak_overrides['overlay_dead_zone'] = dead_zone_value
        joystick_overrides['dead_zone'] = dead_zone_value

    saturation_zone = LaunchConfiguration('saturation_zone').perform(context)
    if saturation_zone != USE_YAML_DEFAULT:
        saturation_zone_value = float(saturation_zone)
        oak_overrides['overlay_saturation_zone'] = saturation_zone_value
        joystick_overrides['saturation_zone'] = saturation_zone_value

    normalization_mode = LaunchConfiguration(
        'normalization_mode'
    ).perform(context)
    if normalization_mode != USE_YAML_DEFAULT:
        oak_overrides['overlay_normalization_mode'] = normalization_mode
        joystick_overrides['normalization_mode'] = normalization_mode

    landmark_index = LaunchConfiguration('landmark_index').perform(context)
    if landmark_index != USE_YAML_DEFAULT:
        landmark_index_value = int(landmark_index)
        oak_overrides['tracked_landmark_index'] = landmark_index_value
        joystick_overrides['landmark_index'] = landmark_index_value

    oak_hand_landmarks_node = Node(
        package='mediapipe_mocap',
        executable='oak_hand_landmarks_node',
        name='oak_hand_landmarks_node',
        output='screen',
        parameters=[
            LaunchConfiguration('oak_config_file'),
            LaunchConfiguration('config_file'),
            oak_overrides,
        ],
    )

    hand_joystick_node = Node(
        package='hand_joystick_interfaces',
        executable='hand_joystick_node',
        name='hand_joystick_interface',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            joystick_overrides,
        ],
    )

    return [oak_hand_landmarks_node, hand_joystick_node]


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
    """Create the OAK hand-joystick pipeline launch description."""
    hand_joystick_share_dir = get_package_share_directory(
        'hand_joystick_interfaces'
    )
    mediapipe_share_dir = get_package_share_directory('mediapipe_mocap')

    hand_joystick_config = os.path.join(
        hand_joystick_share_dir,
        'config',
        'explorer_3d_hand_parameters.yaml',
    )
    oak_hand_landmarks_config = os.path.join(
        mediapipe_share_dir,
        'config',
        'oak_hand_landmarks_node.yaml',
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value=USE_YAML_DEFAULT,
        description='OAK camera FPS (set to override YAML)',
    )

    rgb_width_arg = DeclareLaunchArgument(
        'rgb_width',
        default_value=USE_YAML_DEFAULT,
        description=(
            'OAK RGB/depth output width in pixels (set to override YAML)'
        ),
    )

    rgb_height_arg = DeclareLaunchArgument(
        'rgb_height',
        default_value=USE_YAML_DEFAULT,
        description=(
            'OAK RGB/depth output height in pixels (set to override YAML)'
        ),
    )

    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value=USE_YAML_DEFAULT,
        description=(
            'Show local OpenCV visualization window in '
            'oak_hand_landmarks_node (set to override YAML)'
        ),
    )

    window_name_arg = DeclareLaunchArgument(
        'window_name',
        default_value=USE_YAML_DEFAULT,
        description=(
            'OpenCV window title when visualize is enabled '
            '(set to override YAML)'
        ),
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=hand_joystick_config,
        description='Path to shared 3D hand config file',
    )

    oak_config_file_arg = DeclareLaunchArgument(
        'oak_config_file',
        default_value=oak_hand_landmarks_config,
        description='Path to OAK 3D hand landmarks config file',
    )

    show_control_overlay_arg = DeclareLaunchArgument(
        'show_control_overlay',
        default_value=USE_YAML_DEFAULT,
        description='Override the YAML control-overlay visibility',
    )

    dead_zone_arg = DeclareLaunchArgument(
        'dead_zone',
        default_value=USE_YAML_DEFAULT,
        description=(
            'Joystick dead-zone radius, mirrored to the optional overlay'
        ),
    )

    saturation_zone_arg = DeclareLaunchArgument(
        'saturation_zone',
        default_value=USE_YAML_DEFAULT,
        description=(
            'Joystick saturation radius, mirrored to the optional overlay'
        ),
    )

    normalization_mode_arg = DeclareLaunchArgument(
        'normalization_mode',
        default_value=USE_YAML_DEFAULT,
        description=(
            'Joystick normalization mode, mirrored to the optional overlay'
        ),
    )

    landmark_index_arg = DeclareLaunchArgument(
        'landmark_index',
        default_value=USE_YAML_DEFAULT,
        description=(
            'Tracked landmark index (0-20) for joystick and OAK feedback '
            'overlay (set to override YAML)'
        ),
    )

    return LaunchDescription([
        *_mapper_arguments(),
        _mapper_node(),
        fps_arg,
        rgb_width_arg,
        rgb_height_arg,
        visualize_arg,
        window_name_arg,
        config_file_arg,
        oak_config_file_arg,
        show_control_overlay_arg,
        dead_zone_arg,
        saturation_zone_arg,
        normalization_mode_arg,
        landmark_index_arg,
        OpaqueFunction(function=_create_nodes),
    ])
