# SPDX-License-Identifier: Apache-2.0
"""Mapping OR localization/navigation. Exactly one owner of map -> odom."""
from pathlib import Path
import tempfile
import os
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    mode=LaunchConfiguration('mode').perform(context)
    common={'use_sim_time':True}
    use_ekf=LaunchConfiguration('use_ekf').perform(context).lower()=='true'
    p6=LaunchConfiguration('p6').perform(context)=='true'
    relay=Node(package='forklift_navigation',executable='scan_relay.py',parameters=[common,{'p6_load_filter':p6, 'geometry_file':str(Path(share('forklift_description'))/'config/robot.yaml')}],output='screen',
               on_exit=EmitEvent(event=Shutdown(reason='Scan relay exited')))
    if mode=='mapping':
        return [relay,Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',
                     parameters=[str(Path(share('forklift_navigation'))/'config/slam.yaml'),common],output='screen')]
    map_file=Path(LaunchConfiguration('map').perform(context))
    if not map_file.is_file():raise FileNotFoundError(f'Save a SLAM map before navigation: {map_file}')
    config=yaml.safe_load((Path(share('forklift_navigation'))/'config/nav2.yaml').read_text())
    controller=LaunchConfiguration('controller').perform(context)
    if controller=='mppi':
        overlay=yaml.safe_load((Path(share('forklift_navigation'))/'config/nav2_mppi.yaml').read_text())
        config['controller_server']['ros__parameters'].update(overlay['controller_server']['ros__parameters'])
    g=yaml.safe_load((Path(share('forklift_description'))/'config/robot.yaml').read_text())
    # Include wheel contact spheres, chassis and fork tips in collision footprint.
    xmin=g['footprint_xmin']
    xmax=g['footprint_xmax']
    halfwidth=g['footprint_halfwidth']
    if p6:
        xmax=max(xmax,g['loaded_xmax'])
        halfwidth=max(halfwidth,g['loaded_halfwidth'])
    footprint=str([[xmin,-halfwidth],[xmax,-halfwidth],[xmax,halfwidth],[xmin,halfwidth]])
    for key in ['local_costmap','global_costmap']:
        config[key][key]['ros__parameters']['footprint']=footprint
    config['map_server']['ros__parameters']['yaml_filename']=str(map_file.resolve())
    fd,path=tempfile.mkstemp(prefix='forklift_nav2_',suffix='.yaml')
    with os.fdopen(fd,'w') as stream:yaml.safe_dump(config,stream)
    nodes=[relay]
    def node(package,executable,name=None,remappings=None):
        return Node(package=package,executable=executable,name=name or executable,parameters=[path,common],
                    remappings=remappings or [],output='screen')
    nodes.extend([
        node('nav2_map_server','map_server'),node('nav2_amcl','amcl'),
        Node(package='nav2_controller',executable='controller_server',name='controller_server',
             parameters=[path,common],remappings=[('cmd_vel','/nav/cmd_vel_raw')],
             additional_env={'OMP_NUM_THREADS':'1'},output='screen'),
        node('nav2_planner','planner_server'),node('nav2_smoother','smoother_server'),
        node('nav2_behaviors','behavior_server',remappings=[('cmd_vel','/nav/cmd_vel_raw')]),
        node('nav2_bt_navigator','bt_navigator'),node('nav2_waypoint_follower','waypoint_follower'),
        node('nav2_velocity_smoother','velocity_smoother',remappings=[('cmd_vel','/nav/cmd_vel_selected' if p6 else '/nav/cmd_vel_raw'),('cmd_vel_smoothed','/nav/cmd_vel_smoothed')]),
        Node(package='forklift_navigation',executable='navigation_guard.py',parameters=[common,{'require_imu':use_ekf}],output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='Navigation guard exited'))),
    ])
    for label,names in [('localization',['map_server','amcl']),('navigation',['controller_server','planner_server','smoother_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother'])]:
        nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name=f'lifecycle_manager_{label}',
            parameters=[common,{'autostart':True,'node_names':names,'bond_timeout':4.}],output='screen'))
    def cleanup(event,context):Path(path).unlink(missing_ok=True);return []
    return [RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),*nodes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('p6',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('controller',default_value='mppi',choices=['rpp','mppi']),
        DeclareLaunchArgument('mode',default_value='mapping',choices=['mapping','localization']),
        DeclareLaunchArgument('use_ekf',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('map',default_value=str(Path(share('forklift_navigation'))/'maps/warehouse.yaml')),
        OpaqueFunction(function=setup)])
