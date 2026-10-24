# SPDX-License-Identifier: Apache-2.0
"""Fortress simulation. Controller YAML is derived from the same geometry as URDF."""
import os
from pathlib import Path
import tempfile

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def checked_next(next_actions, label):
    def callback(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=f'{label} failed: {event.returncode}'))]
        return next_actions
    return callback


def setup(context):
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    navigation = LaunchConfiguration('navigation').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    use_ekf = LaunchConfiguration('use_ekf').perform(context).lower() == 'true'
    lift_mode = LaunchConfiguration('lift_mode').perform(context)
    auto_home = LaunchConfiguration('auto_home').perform(context).lower() == 'true'
    test_mass = float(LaunchConfiguration('lift_test_mass').perform(context))
    if not 0<=test_mass<=1:raise ValueError('SIM test mass must be between 0 and 1 kg')
    if world not in ('minimal', 'warehouse', 'p6'):
        raise ValueError('world must be minimal, warehouse or p6')
    geometry = Path(share('forklift_description')) / 'config/robot.yaml'
    g = yaml.safe_load(geometry.read_text())
    controllers = yaml.safe_load((Path(share('forklift_control')) / 'config/controllers.yaml').read_text())
    params = controllers['base_controller']['ros__parameters']['kinematics']
    params['wheels_radius'] = g['wheel_radius']
    kinematic_sum = (g['wheel_separation_x']+g['wheel_separation_y'])/2
    params['sum_of_robot_center_projection_on_X_Y_axis'] = kinematic_sum
    controllers['base_controller']['ros__parameters']['enable_odom_tf'] = not use_ekf
    # Each launch owns its generated YAML. Remove it when launch shuts down.
    fd, config_path = tempfile.mkstemp(prefix='forklift_controllers_', suffix='.yaml')
    with os.fdopen(fd, 'w') as stream:
        yaml.safe_dump(controllers, stream)
    robot = xacro.process_file(str(Path(share('forklift_description'))/'urdf/forklift.urdf.xacro'),
                               mappings={'controllers_file': config_path,
                                         'geometry_file': str(geometry),
                                         'lift_test_mass': str(test_mass),
                                         'p6': str(world=='p6').lower(),
                                         'odometry_topic': '/wheel/odometry' if use_ekf else '/odom'}).toxml()
    # Spawn the exact expanded model from a private file. This avoids waiting
    # indefinitely for transient-local robot_description discovery at startup.
    fd, robot_path = tempfile.mkstemp(prefix='forklift_model_', suffix='.urdf')
    with os.fdopen(fd, 'w') as stream: stream.write(robot)
    common = {'use_sim_time': True}
    # Direct argv avoids a shell wrapper leaving the server behind at Ctrl+C.
    # Fortress uses the `ign gazebo` executable (Gazebo major version 6).
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', *([] if test_mass>0 else ['-r']), '-v', '2', *(['-s'] if world=='p6' or not gui else []),
             *(['--gui-config', str(Path(share('forklift_gazebo'))/'config'/('p6_gui.config' if world=='p6' else 'gui.config'))] if gui and world!='p6' else []),
             str(Path(share('forklift_gazebo'))/'worlds'/f'{world}.sdf')],
        additional_env={'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.pathsep.join([
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', '')])},
        output='screen', on_exit=EmitEvent(event=Shutdown(reason='Gazebo exited')))
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[common, {'robot_description': robot}], output='screen')
    bridge_config = str(Path(share('forklift_gazebo'))/'config/bridge.yaml')
    bridge_temp = None
    if navigation != 'none':
        spec = yaml.safe_load(Path(bridge_config).read_text())
        for item in spec:
            if item['ros_topic_name']=='/scan': item['ros_topic_name']='/scan_raw'
        fd, bridge_temp = tempfile.mkstemp(prefix='forklift_bridge_', suffix='.yaml')
        with os.fdopen(fd, 'w') as stream: yaml.safe_dump(spec, stream)
        bridge_config = bridge_temp
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[common, {'config_file': bridge_config}],
                  output='screen')
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', 'forklift', '-file', robot_path,
                            '-name', 'forklift', '-allow_renaming', 'false', '-z', '0.0' if test_mass>0 else '0.005'],
                 parameters=[common], output='screen')

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '60', '--param-file', config_path],
                    parameters=[common], output='screen')
    js, base, lift = (spawner(n) for n in ('joint_state_broadcaster', 'base_controller', 'lift_controller'))
    gate = Node(package='forklift_control', executable='command_gate.py',
                parameters=[str(Path(share('forklift_control'))/'config/command_gate.yaml'),
                            {'wheel_radius': g['wheel_radius'], 'kinematic_sum': kinematic_sum,
                             'autonomous_enabled': navigation == 'localization',
                             'require_navigation_ready': navigation == 'localization'}],
                output='screen', on_exit=EmitEvent(event=Shutdown(reason='Command gate exited')))
    lift_helper = Node(package='forklift_control', executable='lift_sim.py',
                       parameters=[common, {'travel': g['lift_travel'], 'velocity': g['lift_velocity']}],
                       output='screen', on_exit=EmitEvent(event=Shutdown(reason='Lift helper exited')))
    ready = [gate, lift_helper]
    if lift_mode=='supervised':
        lift_config=str(Path(share('forklift_lift'))/'config/lift.yaml')
        ready=[gate,
            Node(package='forklift_lift',executable='lift_sim_io.py',parameters=[lift_config,common],
                 output='screen',on_exit=EmitEvent(event=Shutdown(reason='Lift SIM I/O exited'))),
            Node(package='forklift_lift',executable='lift_supervisor.py',
                 parameters=[lift_config,common,{'auto_home':auto_home}],output='screen',
                 on_exit=EmitEvent(event=Shutdown(reason='Lift supervisor exited')))]
    resume = Node(package='forklift_bringup',executable='resume_loaded_sim.py',output='screen') if test_mass>0 else None
    first,second,third=(lift,js,base) if test_mass>0 else (js,base,lift)
    if use_ekf:
        ready.extend([
            Node(package='forklift_control', executable='imu_adapter.py', parameters=[common],
                 output='screen', on_exit=EmitEvent(event=Shutdown(reason='IMU adapter exited'))),
            Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
                 parameters=[str(Path(share('forklift_control'))/'config/ekf.yaml'), common],
                 # Preserve /odom for Nav2, guard and existing SIM clients.
                 remappings=[('odometry/filtered', '/odom')], output='screen',
                 on_exit=EmitEvent(event=Shutdown(reason='EKF exited'))),
        ])
    if navigation != 'none':
        ready.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(share('forklift_navigation'))/'launch/navigation.launch.py')),
            launch_arguments={'mode': navigation, 'map': map_file,
                              'controller': LaunchConfiguration('controller'),
                              'use_ekf': str(use_ekf).lower(), 'p6': str(world=='p6').lower()}.items()))
    if rviz:
        ready.append(Node(package='rviz2', executable='rviz2', parameters=[common],
                          arguments=['-d', str(Path(share('forklift_bringup'))/'rviz'/('navigation.rviz' if navigation != 'none' else ('p5.rviz' if lift_mode=='supervised' else 'sim.rviz')))], output='screen'))

    if gui and world=='p6':
        # Start the server and activate controllers before starting the GUI.
        # Fortress combined startup can block entity creation on this machine.
        ready.append(ExecuteProcess(cmd=['ign','gazebo','-g','--gui-config',
            str(Path(share('forklift_gazebo'))/'config/p6_gui.config')],output='screen',
            on_exit=EmitEvent(event=Shutdown(reason='P6 Gazebo GUI exited'))))

    def cleanup(event, ctx):
        Path(config_path).unlink(missing_ok=True)
        Path(robot_path).unlink(missing_ok=True)
        if bridge_temp: Path(bridge_temp).unlink(missing_ok=True)
        return []
    # Fortress rewrites package:// mesh URIs to model://. Both the server
    # and the separately launched GUI must search the parent of forklift_description/.
    resource_path=os.pathsep.join(filter(None,[str(Path(share('forklift_description')).parent),
                                               os.environ.get('IGN_GAZEBO_RESOURCE_PATH','')]))
    return [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',resource_path),
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        *([RegisterEventHandler(OnProcessExit(target_action=resume,on_exit=checked_next([], 'loaded SIM resume')))] if resume else []),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=checked_next(([resume] if resume else [])+[first], 'spawn'))),
        RegisterEventHandler(OnProcessExit(target_action=first, on_exit=checked_next([second], 'first controller'))),
        RegisterEventHandler(OnProcessExit(target_action=second, on_exit=checked_next([third], 'second controller'))),
        RegisterEventHandler(OnProcessExit(target_action=third, on_exit=checked_next(ready, 'third controller'))),
        gazebo, rsp, bridge, spawn,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='minimal', choices=['minimal', 'warehouse', 'p6']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('navigation', default_value='none', choices=['none', 'mapping', 'localization']),
        DeclareLaunchArgument('controller', default_value='mppi', choices=['rpp','mppi']),
        DeclareLaunchArgument('use_ekf', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('lift_mode', default_value='supervised', choices=['supervised','legacy']),
        DeclareLaunchArgument('auto_home', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('lift_test_mass', default_value='0.0'),
        DeclareLaunchArgument('map', default_value=str(Path(share('forklift_navigation'))/'maps/warehouse.yaml')),
        OpaqueFunction(function=setup),
    ])
