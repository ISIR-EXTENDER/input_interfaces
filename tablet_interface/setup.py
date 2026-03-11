from setuptools import setup
from glob import glob

package_name = "tablet_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/{0}".format(package_name), ["package.xml"]),
        ("share/{0}/config".format(package_name), glob("config/*.yaml")),
        ("share/{0}/launch".format(package_name), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="user@todo.todo",
    description="Tablet interface input package.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tablet_interface_node = tablet_interface.main:main",
            "mock_points_publisher = tablet_interface.mock_points_publisher_node:main",
            "raw_to_compressed_bridge = tablet_interface.raw_to_compressed_bridge_node:main",
        ],
    },
)
