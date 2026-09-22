"""The promise this package makes: same topic names, whichever camera.

These run without a camera, a driver package or a ROS graph. They read the
launch description, which is where the promise is actually kept or broken.
"""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext

LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "camera.launch.py"


def load_launch_module():
    spec = importlib.util.spec_from_file_location("camera_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def launch_module():
    return load_launch_module()


def test_every_driver_lands_on_the_same_two_topics(launch_module):
    # The whole point of the package: a consumer names one topic and any camera can fill it.
    for driver in launch_module.DRIVERS:
        if launch_module.DRIVERS[driver].get("launch_file"):
            continue  # It is included with a namespace argument, not remapped.
        remappings = dict(launch_module.normalized_remappings(driver))
        assert set(remappings.values()) == {
            "/camera/color/image_raw",
            "/camera/color/camera_info",
        }, driver


def test_a_custom_namespace_moves_both_topics_together(launch_module):
    remappings = dict(launch_module.normalized_remappings("usb_cam", "/gripper/color"))

    assert set(remappings.values()) == {
        "/gripper/color/image_raw",
        "/gripper/color/camera_info",
    }


def test_a_trailing_slash_does_not_double_up(launch_module):
    remappings = dict(launch_module.normalized_remappings("usb_cam", "/gripper/color/"))

    assert "/gripper/color//image_raw" not in remappings.values()
    assert "/gripper/color/image_raw" in remappings.values()


def test_kinova_vision_is_asked_for_the_right_namespace(launch_module):
    # It takes the parent and appends `color/` itself, so /camera/color means passing it `camera`.
    assert launch_module.kinova_namespace_argument("/camera/color") == "camera"
    assert launch_module.kinova_namespace_argument("/gripper/color") == "gripper"


def test_a_namespace_kinova_vision_cannot_serve_is_refused_out_loud(launch_module):
    # Its own remappings fix the `color/` segment, so silently publishing somewhere else would be
    # worse than saying no.
    with pytest.raises(RuntimeError, match="color"):
        launch_module.kinova_namespace_argument("/gripper/rgb")


def test_apriltag_is_remapped_rather_than_edited(launch_module):
    # apriltag_detector subscribes to absolute /image_raw and /camera_info in detector.cpp, so it
    # cannot follow a namespace. Launch-time remapping leaves its source alone.
    assert dict(launch_module.apriltag_remappings()) == {
        "/image_raw": "/camera/color/image_raw",
        "/camera_info": "/camera/color/camera_info",
    }


def test_the_driver_choices_match_what_the_file_can_actually_start(launch_module):
    description = launch_module.generate_launch_description()
    choices = next(
        action.choices for action in description.entities if getattr(action, "name", "") == "driver"
    )

    assert set(choices) == {*launch_module.DRIVERS.keys(), "none"}


def test_every_driver_names_a_package_and_a_way_to_start_it(launch_module):
    for driver, spec in launch_module.DRIVERS.items():
        assert spec["package"], driver
        assert spec.get("executable") or spec.get("launch_file"), driver


def context_with(**values):
    """A real launch context, so the arguments are read the way launch reads them."""
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "driver": "usb_cam",
            "namespace": "/camera/color",
            "params_file": "",
            "republish_compressed": "false",
            "container_name": "",
            **values,
        }
    )
    return context


def test_an_already_running_camera_starts_nothing(launch_module):
    # driver:=none with nothing to republish is a legitimate no-op, not a misconfiguration.
    assert launch_module._camera_nodes(context_with(driver="none")) == []


def test_a_driver_on_its_own_gets_a_container_so_a_detector_can_join_it(launch_module):
    [node] = launch_module._camera_nodes(context_with(driver="usb_cam"))

    assert type(node).__name__ == "ComposableNodeContainer"


def test_kinova_vision_is_included_rather_than_rebuilt(launch_module):
    # Its own launch brings up colour, depth, the RTSP configuration and two static transforms.
    # Spawning the node here instead would go stale the first time any of that changes.
    [action] = launch_module._camera_nodes(context_with(driver="kinova_vision"))

    assert type(action).__name__ == "IncludeLaunchDescription"


def test_joining_an_existing_container_loads_into_it_instead(launch_module):
    [action] = launch_module._camera_nodes(context_with(driver="usb_cam", container_name="/vision_container"))

    assert type(action).__name__ == "LoadComposableNodes"
