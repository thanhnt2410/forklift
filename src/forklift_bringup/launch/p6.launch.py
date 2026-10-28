# SPDX-License-Identifier: Apache-2.0
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, EmitEvent
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    common={'use_sim_time':True}
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('controller',default_value='mppi',choices=['rpp','mppi']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('forklift_bringup'))/'launch/sim.launch.py')),
            launch_arguments={'world':'p6','navigation':'localization','lift_mode':'supervised','auto_home':'true',
                              'gui':LaunchConfiguration('gui'),'rviz':LaunchConfiguration('rviz'),
                              'rviz_config':str(Path(share('forklift_bringup'))/'rviz/mission.rviz'),
                              'controller':LaunchConfiguration('controller')}.items()),
        Node(package='forklift_task_manager',executable='contact_throttle',parameters=[common],output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='P6 contact throttle exited'))),
        Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[common],
             arguments=['/p6/contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'],output='screen'),
        Node(package='forklift_task_manager',executable='motion_mux.py',parameters=[common],output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='P6 motion mux exited'))),
        Node(package='forklift_task_manager',executable='task_manager.py',parameters=[common],output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='P6 task manager exited'))),
        Node(package='forklift_task_manager',executable='mission_input.py',parameters=[common],output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='Mission input exited'))),
    ])
