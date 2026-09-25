"""Bring up a camera, whichever one it is, on one set of topic names.

Every consumer in this workspace wants the same three things: an image, its
calibration, and a compressed image small enough to send somewhere. What they
get today depends on the camera: `kortex_vision` publishes under
`/camera/color`, `usb_cam` publishes relative `image_raw`, and `camera_ros`
publishes under its own node name. So each consumer learned one camera, and
changing camera meant changing the consumer.

This launch file is the seam. Pick a driver, and the topics come out at
`<namespace>/image_raw`, `<namespace>/camera_info` and
`<namespace>/image_raw/compressed` whatever the driver was. `driver:=none`
covers the camera somebody else already started, such as the one that comes up
with a Kinova arm.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

#: What every consumer in this workspace is pointed at. `color` leaves room for a depth stream
#: beside it without renaming anything that already works.
DEFAULT_NAMESPACE = "/camera/color"

#: Each driver's own names, which this file remaps onto the convention. The key is the `driver`
#: argument; `none` is a camera somebody else started.
DRIVERS = {
    "usb_cam": {
        "package": "usb_cam",
        "plugin": "usb_cam::UsbCamNode",
        "executable": "usb_cam_node_exe",
        "node_name": "camera",
        # Relative, so the remap is direct.
        "image": "image_raw",
        "camera_info": "camera_info",
    },
    "camera_ros": {
        "package": "camera_ros",
        "plugin": "camera::CameraNode",
        "executable": "camera_node",
        "node_name": "camera",
        # Node-relative: `~/image_raw` resolves to `/camera/image_raw` under this node name.
        "image": "~/image_raw",
        "camera_info": "~/camera_info",
    },
    # Included rather than spawned: its own launch brings up colour and depth, the RTSP stream
    # configuration and two static transforms, and duplicating that here would go stale.
    "kinova_vision": {
        "package": "kinova_vision",
        "launch_file": "kinova_vision.launch.py",
        # It publishes `<camera>/color/image_raw` and friends, which is already the convention.
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
        # `driver:=none` with no republisher is a legitimate, and silent, no-op: the camera is
        # already publishing on the convention and this file has nothing left to do.
        return []

    if container_name:
        if not composable:
            return plain
        return [
            *plain,
            LoadComposableNodes(composable_node_descriptions=composable, target_container=container_name),
        ]
    # A container even when we start it ourselves, so a detector can join by name later and read
    # frames without a copy. That is what apriltag_detector's own launch file does today.
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
    # No registered component, so it runs on its own and its frames are copied to whoever reads them.
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
    """`kinova_vision` takes the parent namespace and appends `color/` itself.

    So `/camera/color` means passing it `camera`. A namespace that does not end in `/color` cannot
    be honoured, because the `color/` segment is fixed in its own remappings.
    """
    namespace = namespace.rstrip("/")
    if not namespace.endswith("/color"):
        raise RuntimeError(
            f"kinova_vision publishes under '<camera>/color', so it cannot serve '{namespace}'. "
            "Use a namespace ending in /color, or driver:=none and its own launch file."
        )
    return namespace[: -len("/color")].lstrip("/")


def apriltag_remappings(namespace: str = DEFAULT_NAMESPACE) -> list[tuple[str, str]]:
    """What `apriltag_detector` needs to read this camera.

    Its subscriptions name `/image_raw` and `/camera_info` absolutely, in
    `detector.cpp`, so it cannot follow a namespace on its own. Remapping at
    launch leaves its code alone.
    """
    namespace = namespace.rstrip("/")
    return [
        ("/image_raw", f"{namespace}/image_raw"),
        ("/camera_info", f"{namespace}/camera_info"),
    ]


def _republisher(namespace: str) -> Node:
    """Only for a driver that publishes raw without the compressed transport plugin.

    Kept as a plain node rather than a component: `republish` takes its transports as command line
    arguments, which is the form every example uses and the one least likely to move.
    """
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
