from glob import glob

from setuptools import setup

package_name = "camera_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Susana Guerry Sanchez",
    maintainer_email="guerrysanchez@isir.upmc.fr",
    description="One camera bring-up and one set of topic names, whichever camera the robot has.",
    license="MIT",
    tests_require=["pytest"],
)
