from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    image_folder_arg = DeclareLaunchArgument(
        "image_folder",
        default_value="",
        description="Folder path containing test images for offline_media_publisher",
    )
    image_fps_arg = DeclareLaunchArgument(
        "image_fps",
        default_value="10",
        description="Image publishing rate in Hz",
    )
    points_rate_arg = DeclareLaunchArgument(
        "points_rate_hz",
        default_value="5.0",
        description="Mock points publishing rate in Hz",
    )
    point_ax_arg = DeclareLaunchArgument("point_ax", default_value="220.0")
    point_ay_arg = DeclareLaunchArgument("point_ay", default_value="260.0")
    point_bx_arg = DeclareLaunchArgument("point_bx", default_value="420.0")
    point_by_arg = DeclareLaunchArgument("point_by", default_value="260.0")

    image_pub = Node(
        package="offline_media_publisher",
        executable="image_publisher",
        name="image_publisher",
        output="screen",
        parameters=[
            {
                "folder_path": LaunchConfiguration("image_folder"),
                "fps": LaunchConfiguration("image_fps"),
            }
        ],
    )

    compressor_bridge = Node(
        package="tablet_interface",
        executable="raw_to_compressed_bridge",
        name="petanque_image_compressor_bridge",
        output="screen",
        parameters=[
            {
                "input_image_topic": "/camera/color/image_raw",
                "output_compressed_topic": "/petanque/measure/request_image/compressed",
                "jpeg_quality": 90,
            }
        ],
    )

    tablet_interface = Node(
        package="tablet_interface",
        executable="tablet_interface_node",
        name="tablet_interface_node",
        output="screen",
        parameters=[
            {
                "petanque_measurements_enabled": True,
                "petanque_measurement_request_image_topic": "/petanque/measure/request_image/compressed",
                "petanque_measurement_points_topic": "/petanque_measurements/points",
                "petanque_measurement_result_image_topic": "/petanque/measure/result_image/compressed",
                "petanque_measurement_result_vectors_topic": "/petanque/measure/result_vectors",
                "petanque_intrinsics_mode": "image",
            }
        ],
    )

    mock_points = Node(
        package="tablet_interface",
        executable="mock_points_publisher",
        name="petanque_mock_points_publisher",
        output="screen",
        parameters=[
            {
                "points_topic": "/petanque_measurements/points",
                "publish_rate_hz": LaunchConfiguration("points_rate_hz"),
                "point_ax": LaunchConfiguration("point_ax"),
                "point_ay": LaunchConfiguration("point_ay"),
                "point_bx": LaunchConfiguration("point_bx"),
                "point_by": LaunchConfiguration("point_by"),
            }
        ],
    )

    return LaunchDescription(
        [
            image_folder_arg,
            image_fps_arg,
            points_rate_arg,
            point_ax_arg,
            point_ay_arg,
            point_bx_arg,
            point_by_arg,
            image_pub,
            compressor_bridge,
            tablet_interface,
            mock_points,
        ]
    )
