# SPDX-License-Identifier: Apache-2.0
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('world',default_value='minimal'),
        DeclareLaunchArgument('navigation',default_value='none'),
        DeclareLaunchArgument('lift_test_mass',default_value='0.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('forklift_bringup'))/'launch/sim.launch.py')),
            launch_arguments={**{k:LaunchConfiguration(k) for k in ['gui','rviz','world','navigation','lift_test_mass']},
                              'lift_mode':'supervised','auto_home':'false'}.items())])
