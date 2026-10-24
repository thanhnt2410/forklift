#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Run a simulation check in its own ROS domain/process groups, then clean up.

Source /opt/ros/humble/setup.bash and install/setup.bash first.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('phase', choices=['baseline', 'p1'])
    parser.add_argument('--world', default='minimal', choices=['minimal', 'warehouse'])
    parser.add_argument('--output')
    parser.add_argument('--domain', type=int, default=86)
    parser.add_argument('--gui', action='store_true', help='Show Gazebo and RViz during P1 checks')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = Path(args.output or root/'results'/('p0' if args.phase=='baseline' else 'p1_final')).resolve()
    output.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
               IGN_PARTITION=f'forklift_check_{os.getpid()}',
               ROS_LOG_DIR=str(output/'ros_logs'),
               IGN_LOG_PATH=tempfile.mkdtemp(prefix='forklift_ign_logs_'))
    processes, files = [], []

    def launch(command, logfile):
        stream = (output/logfile).open('w')
        files.append(stream)
        p = subprocess.Popen(command, env=env, cwd=root, stdout=stream,
                             stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        return p

    if args.phase=='baseline':
        demo = Path(get_package_share_directory('gz_ros2_control_demos'))
        urdf = subprocess.check_output(['xacro', str(demo/'urdf/test_mecanum_drive.xacro.urdf')], text=True)
        urdf = urdf.replace('xmlns:gz=', 'xmlns:ignition=').replace('gz:expressed_in', 'ignition:expressed_in')
        model = output/'baseline_fortress.urdf'
        model.write_text(urdf)
        code = (demo/'launch/mecanum_drive_example.launch.py').read_text()
        a, b = code.index('    robot_description_content = Command('), code.index('    robot_description = ')
        code = code[:a]+f'    robot_description_content = open({str(model)!r}).read()\n'+code[b:]
        code = code.replace(' -r -v 1 empty.sdf', ' -s -r -v 2 empty.sdf')
        launchfile = output/'baseline_headless.launch.py'
        launchfile.write_text(code)
        command = ['ros2','launch',str(launchfile)]
        world, pose_topic = 'empty', '/baseline/poses'
        test = ['python3',str(root/'tests/check_baseline.py'),'--output',str(output)]
    else:
        visual = 'true' if args.gui else 'false'
        command = ['ros2','launch','forklift_bringup','sim.launch.py',f'gui:={visual}',f'rviz:={visual}',f'world:={args.world}','lift_mode:=legacy']
        world, pose_topic = 'forklift', '/test/poses'
        test = ['python3',str(root/'tests/check_sim.py'),'--output',str(output)]
    try:
        launch(command,'launch.log')
        topic=f'/world/{world}/dynamic_pose/info'
        launch(['ros2','run','ros_gz_bridge','parameter_bridge',
                f'{topic}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '--ros-args','-r',f'{topic}:={pose_topic}'],'test_pose_bridge.log')
        with (output/'test.log').open('w') as stream:
            p=subprocess.Popen(test,env=env,cwd=root,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(p)
            try:
                result=p.wait(timeout=160 if args.phase=='p1' else 60)
            except subprocess.TimeoutExpired:
                result=124
        print((output/'test.log').read_text())
        return result
    finally:
        # Signal entire groups, including any shell-created Gazebo descendants.
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
            for p in reversed(processes):
                try:
                    if sig == signal.SIGINT:
                        # Let ROS launch forward the first signal cleanly.
                        # The later group signals also clean up shell descendants.
                        if p.poll() is None:
                            p.send_signal(sig)
                    else:
                        os.killpg(p.pid,sig)
                except ProcessLookupError:
                    pass
            for p in processes:
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    pass
        for stream in files:
            stream.close()


if __name__=='__main__':
    sys.exit(main())
