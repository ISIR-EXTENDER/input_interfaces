from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    visual_servoing_config = PathJoinSubstitution([
        FindPackageShare("visual_servoing"),
        "config",
        "visual_servoing.yaml",
    ])
    saved_tag_goals_config = PathJoinSubstitution([
        FindPackageShare("visual_servoing"),
        "config",
        "saved_tag_goals.yaml",
    ])
    handeye_tf_config = PathJoinSubstitution([
        FindPackageShare("visual_servoing"),
        "config",
        "handeye_tf_kinovaCam.yaml",
    ])

    visual_servoing = Node(
        package="visual_servoing",
        executable="visual_servoing",
        name="visual_servoing",
        output="screen",
        parameters=[
            visual_servoing_config,
            {
                "yaml_path": saved_tag_goals_config,
                "yaml_path_transform_EEtoCAM": handeye_tf_config,
            },
        ],
    )

    return LaunchDescription([
        visual_servoing,
    ])
