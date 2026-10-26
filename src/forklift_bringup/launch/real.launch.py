# SPDX-License-Identifier: Apache-2.0
"""Manual ESP32 USB/UART bringup with optional automatic startup arm."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    config = LaunchConfiguration('config').perform(context)
    port = LaunchConfiguration('port').perform(context)
    lidar_config = LaunchConfiguration('lidar_params_file').perform(context)
    lidar_port = yaml.safe_load(Path(lidar_config).read_text())['ydlidar_ros2_driver_node']['ros__parameters']['port']
    if not isinstance(lidar_port, str) or not Path(lidar_port).is_absolute():
        raise ValueError('LiDAR config must specify an absolute serial path so ESP32 auto can exclude it')
    # One file supplies geometry/limits to the bridge and the command gate.
    params = yaml.safe_load(Path(config).read_text())['esp32_bridge']['ros__parameters']
    gate = {key: params[key] for key in ('wheel_radius', 'kinematic_sum', 'max_wheel_speed')}
    gate.update(use_sim_time=False, autonomous_enabled=False, require_navigation_ready=False)
    def shutdown(reason):
        return EmitEvent(event=Shutdown(reason=reason))
    return [
        Node(package='forklift_hardware', executable='esp32_bridge.py',
             parameters=[config, {'port': port, 'lidar_port': lidar_port, 'use_sim_time': False,
                                 'auto_arm': ParameterValue(LaunchConfiguration('auto_arm'), value_type=bool)}],
             output='screen',
             on_exit=shutdown('ESP32 bridge exited')),
        Node(package='forklift_control', executable='command_gate.py',
             parameters=[str(Path(share('forklift_control'))/'config/command_gate.yaml'), gate],
             output='screen', on_exit=shutdown('Command gate exited')),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='auto',
                              description='auto detects firmware telemetry; or an explicit ESP32 serial path'),
        DeclareLaunchArgument('config', default_value=str(Path(share('forklift_hardware'))/'config/esp32.yaml')),
        DeclareLaunchArgument('lidar_params_file',
                              default_value=str(Path(share('forklift_hardware'))/'config/ydlidar.yaml'),
                              description='Reserve this LiDAR serial port during ESP32 detection'),
        DeclareLaunchArgument('auto_arm', default_value='true', choices=['true', 'false'],
                              description='Automatically arm once at startup after healthy telemetry and zero input'),
        OpaqueFunction(function=setup),
    ])
