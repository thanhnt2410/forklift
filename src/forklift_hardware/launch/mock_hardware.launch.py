# SPDX-License-Identifier: Apache-2.0
"""P3: isolated PTY hardware-in-the-loop substitute. No Gazebo/physical port."""
import os
from pathlib import Path
import tempfile
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node


def setup(context):
    directory = tempfile.TemporaryDirectory(prefix='forklift_mock_')
    port_file = str(Path(directory.name)/'mcu.pty')
    g = yaml.safe_load((Path(share('forklift_description'))/'config/robot.yaml').read_text())
    data = yaml.safe_load((Path(share('forklift_control'))/'config/controllers.yaml').read_text())
    for name in data:
        data[name]['ros__parameters']['use_sim_time'] = False
    k = data['base_controller']['ros__parameters']['kinematics']
    k['wheels_radius'] = g['wheel_radius']
    k['sum_of_robot_center_projection_on_X_Y_axis'] = (g['wheel_separation_x']+g['wheel_separation_y'])/2
    config = str(Path(directory.name)/'controllers.yaml')
    Path(config).write_text(yaml.safe_dump(data))
    robot = xacro.process_file(str(Path(share('forklift_description'))/'urdf/forklift.urdf.xacro'),
        mappings={'backend':'mock','port_file':port_file,'controllers_file':config}).toxml()
    common = {'use_sim_time':False}
    def shutdown(reason):
        return EmitEvent(event=Shutdown(reason=reason))
    nodes = [
        Node(package='forklift_hardware',executable='mock_mcu.py',parameters=[{'port_file':port_file,
             'wheel_radius':g['wheel_radius'],'kinematic_sum':k['sum_of_robot_center_projection_on_X_Y_axis']}],output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[common,{'robot_description':robot}],output='screen'),
        Node(package='controller_manager',executable='ros2_control_node',parameters=[common,{'robot_description':robot},config],
             remappings=[('/base_controller/reference_unstamped','/cmd_vel'),('/base_controller/odometry','/odom'),('/base_controller/tf_odometry','/tf')],
             output='screen',on_exit=shutdown('Mock controller manager exited')),
        Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','base_controller','lift_controller','--controller-manager-timeout','30'],output='screen'),
        Node(package='forklift_control',executable='command_gate.py',parameters=[str(Path(share('forklift_control'))/'config/command_gate.yaml'),common,
             {'wheel_radius':g['wheel_radius'],'kinematic_sum':k['sum_of_robot_center_projection_on_X_Y_axis']}],output='screen'),
        Node(package='forklift_control',executable='lift_sim.py',parameters=[common,{'travel':g['lift_travel'],'velocity':g['lift_velocity']}],output='screen'),
    ]
    def cleanup(event, context):
        directory.cleanup()
        return []
    return [RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),*nodes]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=setup)])
