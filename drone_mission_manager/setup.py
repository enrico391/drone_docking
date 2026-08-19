from glob import glob
import os

from setuptools import setup


package_name = "drone_mission_manager"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "config", "missions"),
            glob("config/missions/*.yaml"),
        ),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Enrico",
    maintainer_email="user@example.com",
    description="Typed mission and precision landing manager for drone docking.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_manager = drone_mission_manager.mission_manager:main",
        ],
    },
)
