# SPDX-License-Identifier: Apache-2.0
"""Add PS4 manual input to an existing SIM bringup; does not start Gazebo."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('forklift_control')) / 'config/ps4.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('device_name', default_value=''),
        Node(package='joy', executable='game_controller_node', name='ps4_driver',
             parameters=[config, {'use_sim_time': False,
                         'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=int),
                         'device_name': ParameterValue(LaunchConfiguration('device_name'), value_type=str)}],
             output='screen'),
        Node(package='forklift_control', executable='ps4_teleop.py', name='ps4_teleop',
             parameters=[config, {'use_sim_time': False}], output='screen'),
    ])
