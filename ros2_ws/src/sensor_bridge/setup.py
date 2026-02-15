from setuptools import find_packages, setup
import os
from glob import glob

package_name = "sensor_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nelson Durrant",
    maintainer_email="snelsondurrant@gmail.com",
    description="Sensor conversion utilities for the CougUV",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "dvl_odom_converter = sensor_bridge.dvl_odom_converter_node:main",
            "gps_odom_converter = sensor_bridge.gps_odom_converter_node:main",
        ],
    },
)
