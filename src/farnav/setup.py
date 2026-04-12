from glob import glob
from setuptools import find_packages, setup

package_name = 'farnav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='het',
    maintainer_email='het@todo.todo',
    description='farnav package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner = farnav.global_planner:main',
            'controller_mpc = farnav.controller_mpc:main',
            'path_overlay = farnav.path_overlay_node:main',
            'live_path_viz = farnav.live_path_viz:main',
            'gps_odom = farnav.gps_odom_node:main',
            'data_logger = farnav.data_logger:main',
        ],
    },
)
