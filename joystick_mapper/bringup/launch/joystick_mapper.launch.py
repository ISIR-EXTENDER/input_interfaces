from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    default_config_file = PathJoinSubstitution(
        [
            FindPackageShare("joystick_mapper"),
            "config",
            "joystick_3d.yaml",
        ]
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Path to the joystick mapper parameter file.",
            ),
            Node(
                package="joystick_mapper",
                executable="joystick_mapper_node",
                name="joystick_mapper",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
