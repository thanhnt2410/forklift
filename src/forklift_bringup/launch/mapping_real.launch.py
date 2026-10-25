# SPDX-License-Identifier: Apache-2.0
"""Map with existing hardware /scan and /odom, using the CAD robot TF tree."""
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description = Path(share('forklift_description')) / 'urdf/forklift.urdf.xacro'
    robot = xacro.process_file(str(description)).toxml()
    common = {'use_sim_time': False}
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=str(Path(share('forklift_navigation')) / 'config/slam.yaml')),
        # URDF sensor/control tags are inert in robot_state_publisher: it only
        # publishes the CAD transforms and consumes hardware /joint_states.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[common, {'robot_description': robot}], output='screen'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox',
             parameters=[LaunchConfiguration('slam_params_file'), common],
             output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(Path(share('forklift_bringup')) / 'rviz/mapping_real.rviz')],
             parameters=[common], condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ])
