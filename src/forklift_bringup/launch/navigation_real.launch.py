# SPDX-License-Identifier: Apache-2.0
"""Localization + dynamic missions with existing hardware /scan_raw, /odom, lift.
Does not connect serial ports or enable the hardware command gate.
"""
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot=xacro.process_file(str(Path(share('forklift_description'))/'urdf/forklift.urdf.xacro')).toxml()
    def include(package,file,arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share(package))/'launch'/file)),launch_arguments=arguments.items())
    return LaunchDescription([
        DeclareLaunchArgument('map',description='Absolute path to the saved navigation map YAML'),
        DeclareLaunchArgument('rviz',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('publish_robot_description',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('controller',default_value='mppi',choices=['mppi','rpp']),
        DeclareLaunchArgument('staging_distance',default_value='0.65'),
        DeclareLaunchArgument('pallet_face_width',default_value='0.24'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'use_sim_time':False,'robot_description':robot}],
             condition=IfCondition(LaunchConfiguration('publish_robot_description')),output='screen'),
        include('forklift_navigation','navigation.launch.py',{'mode':'localization','use_sim_time':'false','use_ekf':'false','p6':'true',
            'controller':LaunchConfiguration('controller'),'map':LaunchConfiguration('map')}),
        include('forklift_bringup','mission.launch.py',{'use_sim_time':'false','scan_topic':'/scan_raw','load_feedback':'external',
            'staging_distance':LaunchConfiguration('staging_distance'),'pallet_face_width':LaunchConfiguration('pallet_face_width')}),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(Path(share('forklift_bringup'))/'rviz/mission.rviz')],
             parameters=[{'use_sim_time':False}],condition=IfCondition(LaunchConfiguration('rviz')),output='screen'),
    ])
