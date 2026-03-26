import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('amiga_ros2_bridge')
    default_config = os.path.join(pkg_dir, 'config', 'amiga_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to the bridge config YAML',
        ),
        DeclareLaunchArgument(
            'host',
            default_value='localhost',
            description='Amiga robot IP or hostname',
        ),
        Node(
            package='amiga_ros2_bridge',
            executable='amiga_bridge',
            name='amiga_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'host': LaunchConfiguration('host')},
            ],
        ),
    ])
