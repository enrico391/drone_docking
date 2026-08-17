from setuptools import setup
from glob import glob
import os

package_name = 'dock_apriltag_localizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Enrico',
    description='Multi-AprilTag two-stage docking localizer for ROS 2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dock_localizer = dock_apriltag_localizer.dock_localizer:main',
        ],
    },
)
