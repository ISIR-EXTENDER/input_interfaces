from glob import glob
from setuptools import setup


package_name = "keyboard_interface"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="user@todo.todo",
    description="Keyboard interface input package.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_interface_node = keyboard_interface.main:main",
        ],
    },
)