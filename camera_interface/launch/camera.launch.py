"""Bring up any camera on one set of topic names: <namespace>/image_raw, camera_info and image_raw/compressed."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

DEFAULT_NAMESPACE = "/camera/color"

# Each driver's own topic names, remapped onto the convention.
DRIVERS = {
    "usb_cam": {
        "package": "usb_cam",
        "plugin": "usb_cam::UsbCamNode",
        "executable": "usb_cam_node_exe",
        "node_name": "camera",
        "image": "image_raw",
        "camera_info": "camera_info",
    },
    "camera_ros": {
        "package": "camera_ros",
        "plugin": "camera::CameraNode",
        "executable": "camera_node",
        "node_name": "camera",
        "image": "~/image_raw",
        "camera_info": "~/camera_info",
    },
    # Included rather than spawned: its launch also owns the RTSP config, depth and static transforms.
    "kinova_vision": {
        "package": "kinova_vision",
        "launch_file": "kinova_vision.launch.py",
        "image": "color/image_raw",
        "camera_info": "color/camera_info",
    },
}


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([*_declare_arguments(), OpaqueFunction(function=_camera_nodes)])


def _declare_arguments() -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "driver",
            default_value="usb_cam",
            choices=[*DRIVERS.keys(), "none"],
            description="Which camera driver to start. 'none' when the camera is already running.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value="",
            description="A robot's camera parameters, such as this package's config/explorer_camera.yaml.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value=DEFAULT_NAMESPACE,
            description="Where the normalized topics come out.",
        ),
        DeclareLaunchArgument(
            "republish_compressed",
            default_value="false",
            description=(
                "Add an image_transport republisher. Only needed for a driver that publishes raw "
                "without the compressed transport plugin."
            ),
        ),
        DeclareLaunchArgument(
            "container_name",
            default_value="",
            description=(
                "Join this running component container instead of starting one, so the driver and "
                "a detector pass frames without serializing them."
            ),
        ),
    ]


def _camera_nodes(context, *_args, **_kwargs):
    driver = LaunchConfiguration("driver").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context).rstrip("/")
    params_file = LaunchConfiguration("params_file").perform(context)
    container_name = LaunchConfiguration("container_name").perform(context)
    republish = LaunchConfiguration("republish_compressed").perform(context).lower() in ("true", "1")

    composable: list[ComposableNode] = []
    plain: list = []
    if driver != "none":
        node = _driver_description(driver, namespace, params_file)
        (composable if isinstance(node, ComposableNode) else plain).append(node)
    if republish:
        plain.append(_republisher(namespace))

    if not composable and not plain:
        return []

    if container_name:
        if not composable:
            return plain
        return [
            *plain,
            LoadComposableNodes(composable_node_descriptions=composable, target_container=container_name),
        ]
    # Our own container, so a detector can join it and read frames without a copy.
    return [*plain, *_own_container(composable)]


def _driver_description(driver: str, namespace: str, params_file: str):
    spec = DRIVERS[driver]

    if spec.get("launch_file"):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare(spec["package"]), "launch", spec["launch_file"]])
            ),
            launch_arguments={"camera": kinova_namespace_argument(namespace)}.items(),
        )

    parameters = [params_file] if params_file else []
    remappings = normalized_remappings(driver, namespace)

    if spec["plugin"]:
        return ComposableNode(
            package=spec["package"],
            plugin=spec["plugin"],
            name=spec["node_name"],
            parameters=parameters,
            remappings=remappings,
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    return Node(
        package=spec["package"],
        executable=spec["executable"],
        name=spec["node_name"],
        parameters=parameters,
        remappings=remappings,
        output="screen",
    )


def normalized_remappings(driver: str, namespace: str = DEFAULT_NAMESPACE) -> list[tuple[str, str]]:
    """The driver's own topic names, mapped onto the convention."""
    spec = DRIVERS[driver]
    namespace = namespace.rstrip("/")
    return [
        (spec["image"], f"{namespace}/image_raw"),
        # image_transport names its compressed topic from the original name, so it needs its own remap.
        (f"{spec['image']}/compressed", f"{namespace}/image_raw/compressed"),
        (spec["camera_info"], f"{namespace}/camera_info"),
    ]


def kinova_namespace_argument(namespace: str = DEFAULT_NAMESPACE) -> str:
    """`kinova_vision` appends `color/` itself, so `/camera/color` means passing it `camera`."""
    namespace = namespace.rstrip("/")
    if not namespace.endswith("/color"):
        raise RuntimeError(
            f"kinova_vision publishes under '<camera>/color', so it cannot serve '{namespace}'. "
            "Use a namespace ending in /color, or driver:=none and its own launch file."
        )
    return namespace[: -len("/color")].lstrip("/")


def apriltag_remappings(namespace: str = DEFAULT_NAMESPACE) -> list[tuple[str, str]]:
    """`apriltag_detector` subscribes to absolute `/image_raw` and `/camera_info`."""
    namespace = namespace.rstrip("/")
    return [
        ("/image_raw", f"{namespace}/image_raw"),
        ("/camera_info", f"{namespace}/camera_info"),
    ]


def _republisher(namespace: str) -> Node:
    """For a driver that publishes raw without the compressed transport plugin."""
    return Node(
        package="image_transport",
        executable="republish",
        name="camera_republisher",
        arguments=["raw", "compressed"],
        remappings=[
            ("in", f"{namespace}/image_raw"),
            ("out/compressed", f"{namespace}/image_raw/compressed"),
        ],
        output="screen",
    )


def _own_container(composable_nodes):
    if not composable_nodes:
        return []
    return [
        ComposableNodeContainer(
            name="camera_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=composable_nodes,
            output="screen",
        )
    ]
