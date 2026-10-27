# SPDX-License-Identifier: Apache-2.0
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_ekf', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('auto_home', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('map', default_value=str(Path(share('forklift_navigation'))/'maps/warehouse.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('forklift_bringup'))/'launch/sim.launch.py')),
            launch_arguments={'world':'warehouse', 'navigation':'mapping',
                'gui':LaunchConfiguration('gui'), 'rviz':LaunchConfiguration('rviz'),
                'autostart':LaunchConfiguration('autostart'),
                'map':LaunchConfiguration('map'), 'use_ekf':LaunchConfiguration('use_ekf'),
                'lift_mode':'supervised','auto_home':LaunchConfiguration('auto_home')}.items()),
    ])
