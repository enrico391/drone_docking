from glob import glob
import os

from setuptools import setup


package_name = "flight_cycle_manager"


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
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pymavlink>=2.4.49,<3"],
    zip_safe=True,
    maintainer="Enrico",
    maintainer_email="user@example.com",
    description="Takeoff, ArduPilot mission handoff, and precision docking manager.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "flight_cycle_manager = flight_cycle_manager.manager:main",
        ],
    },
)
