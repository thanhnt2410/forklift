# SPDX-License-Identifier: Apache-2.0
"""Run only the physical LiDAR driver on the Pi; no wheel commands or TF."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = str(Path(get_package_share_directory('forklift_hardware')) /
                         'config' / 'ydlidar.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='LiDAR port and measured hardware settings'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='Use /scan_raw with navigation_real.launch.py'),
        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
             name='ydlidar_ros2_driver_node', output='screen',
             parameters=[LaunchConfiguration('params_file')],
             remappings=[('/scan', LaunchConfiguration('scan_topic'))]),
    ])
