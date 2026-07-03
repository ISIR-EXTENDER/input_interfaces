import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import (
    PathJoinSubstitution
)
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # YAML param
    #controllers_yaml_path = PathJoinSubstitution(
    #    [
    #        FindPackageShare("visual_servoing"),            # package name
    #        "config",                                       # file localization
    #        "visual_servoing_param.yaml",                   # Yaml file name
    #    ]
    #)

    visual_servoing = Node(
        package="visual_servoing",
        executable="visual_servoing",
        name="visual_servoing",
        output="screen",
        parameters=[
            '/home/robingibaud/ros2_ws/src/extender_workspace/src/visual_servoing/config/visual_servoing.yaml',                                             #R.G param robot
        ],
    )

    return LaunchDescription([
        visual_servoing
        #shared_control_visualization_node,
    ])
