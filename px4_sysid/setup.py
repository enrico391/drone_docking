from setuptools import setup

package_name = "px4_sysid"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Enrico Moro",
    maintainer_email="user@example.com",
    description="PX4 Offboard velocity system identification for the outer docking PID.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "px4_sysid = px4_sysid.px4_sysid_node:main",
        ],
    },
)
