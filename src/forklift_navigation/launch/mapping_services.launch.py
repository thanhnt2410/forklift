# SPDX-License-Identifier: Apache-2.0
"""Lifecycle-managed paired saver, shared by real and simulated mapping.

Humble SLAM Toolbox is a regular Node: only the saver belongs in node_names.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    common = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('save_timeout', default_value='30.0'),
        Node(package='forklift_navigation', executable='paired_map_saver',
             name='map_saver_server', parameters=[common, {
                 'save_timeout': ParameterValue(LaunchConfiguration('save_timeout'), value_type=float)}], output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='Mapping saver exited'))),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_mapping',
             parameters=[common, {
                 'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                 'node_names': ['map_saver_server'], 'bond_timeout': 4.0,
                 'attempt_respawn_reconnection': False}], output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='Mapping lifecycle manager exited'))),
    ])
