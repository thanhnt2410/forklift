# SPDX-License-Identifier: Apache-2.0
"""Mission nodes for an existing Nav2/lift stack; no hardware is started here.
Nav2 velocity_smoother input must be /nav/cmd_vel_selected (p6:=true).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    common={'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('scan_topic',default_value='/scan_raw'),
        DeclareLaunchArgument('load_feedback',default_value='external',choices=['external','contacts']),
        DeclareLaunchArgument('staging_distance',default_value='0.65'),
        DeclareLaunchArgument('pallet_face_width',default_value='0.24'),
        DeclareLaunchArgument('initial_detection_timeout',default_value='2.0'),
        Node(package='forklift_task_manager',executable='motion_mux.py',parameters=[common],output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='Mission mux exited'))),
        Node(package='forklift_task_manager',executable='task_manager.py',parameters=[common,{
            'approved_route_file':'', 'scan_topic':LaunchConfiguration('scan_topic'),
            'load_feedback':LaunchConfiguration('load_feedback'),
            'staging_distance':ParameterValue(LaunchConfiguration('staging_distance'),value_type=float),
            'initial_detection_timeout':ParameterValue(LaunchConfiguration('initial_detection_timeout'),value_type=float),
            'pallet_face_width':ParameterValue(LaunchConfiguration('pallet_face_width'),value_type=float)}],output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='Mission manager exited'))),
        Node(package='forklift_task_manager',executable='mission_input.py',parameters=[common],output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='Mission input exited'))),
    ])
