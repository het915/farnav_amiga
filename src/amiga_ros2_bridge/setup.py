from setuptools import setup
import os
from glob import glob

package_name = 'amiga_ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'farm-ng-amiga', 'farm-ng-core'],
    zip_safe=True,
    maintainer='het',
    maintainer_email='het@todo.todo',
    description='ROS2 Humble bridge for farm-ng Amiga robot via gRPC',
    license='MIT',
    entry_points={
        'console_scripts': [
            'amiga_bridge = amiga_ros2_bridge.amiga_bridge_node:main',
            'amiga_streams = amiga_ros2_bridge.streams_node:main',
            'amiga_cmd_vel = amiga_ros2_bridge.cmd_vel_node:main',
        ],
    },
)
