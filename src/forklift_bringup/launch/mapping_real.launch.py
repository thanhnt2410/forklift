# SPDX-License-Identifier: Apache-2.0
"""Map with existing hardware /scan and /odom, using the CAD robot TF tree."""
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, EmitEvent
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = Path(share('forklift_description')) / 'urdf/forklift.urdf.xacro'
    robot = xacro.process_file(str(description)).toxml()
    common = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('publish_robot_description', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('relay_replay_tf', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('max_scan_age', default_value='0.5'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=str(Path(share('forklift_navigation')) / 'config/slam.yaml')),
        # URDF sensor/control tags are inert in robot_state_publisher: it only
        # publishes the CAD transforms and consumes hardware /joint_states.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[common, {'robot_description': robot}], output='screen',
             condition=IfCondition(LaunchConfiguration('publish_robot_description'))),
        Node(package='forklift_navigation', executable='mapping_scan_filter.py',
             parameters=[common, {'max_scan_age': ParameterValue(
                 LaunchConfiguration('max_scan_age'), value_type=float),
                 'relay_replay_tf': ParameterValue(LaunchConfiguration('relay_replay_tf'), value_type=bool)}],
             output='screen', on_exit=EmitEvent(event=Shutdown(reason='Mapping scan filter exited'))),
        Node(package='forklift_navigation', executable='probability_slam',
             name='slam_toolbox',
             parameters=[LaunchConfiguration('slam_params_file'), common,
                         {'scan_topic': '/mapping/scan'}],
             output='screen', on_exit=EmitEvent(event=Shutdown(reason='SLAM exited'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(share('forklift_navigation')) / 'launch/mapping_services.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'autostart': LaunchConfiguration('autostart')}.items()),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(Path(share('forklift_bringup')) / 'rviz/mapping_real.rviz')],
             parameters=[common], condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ])
