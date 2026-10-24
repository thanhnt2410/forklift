# SPDX-License-Identifier: Apache-2.0
"""Manual ESP32 USB/UART bringup; explicit arm, no Gazebo or navigation."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    config = LaunchConfiguration('config').perform(context)
    port = LaunchConfiguration('port').perform(context)
    # One file supplies geometry/limits to the bridge and the command gate.
    params = yaml.safe_load(Path(config).read_text())['esp32_bridge']['ros__parameters']
    gate = {key: params[key] for key in ('wheel_radius', 'kinematic_sum', 'max_wheel_speed')}
    gate.update(use_sim_time=False, autonomous_enabled=False, require_navigation_ready=False)
    def shutdown(reason):
        return EmitEvent(event=Shutdown(reason=reason))
    return [
        Node(package='forklift_hardware', executable='esp32_bridge.py',
             parameters=[config, {'port': port, 'use_sim_time': False}], output='screen',
             on_exit=shutdown('ESP32 bridge exited')),
        Node(package='forklift_control', executable='command_gate.py',
             parameters=[str(Path(share('forklift_control'))/'config/command_gate.yaml'), gate],
             output='screen', on_exit=shutdown('Command gate exited')),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', description='ESP32 USB serial path; prefer /dev/serial/by-id/...'),
        DeclareLaunchArgument('config', default_value=str(Path(share('forklift_hardware'))/'config/esp32.yaml')),
        OpaqueFunction(function=setup),
    ])
