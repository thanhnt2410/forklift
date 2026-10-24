#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Launch P5 simulator in isolation, run acceptance and clean up its processes."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--gui',action='store_true')
    parser.add_argument('--domain',type=int,default=122);parser.add_argument('--repetitions',type=int,default=10)
    parser.add_argument('--test-mass',type=float,default=0.)
    parser.add_argument('--nominal-only',action='store_true');parser.add_argument('--output',default='results/p5/unloaded')
    args=parser.parse_args();root=Path(__file__).resolve().parents[1];out=(root/args.output).resolve();out.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',
        IGN_PARTITION=f'forklift_p5_{os.getpid()}',ROS_LOG_DIR=f'/tmp/forklift_p5_{os.getpid()}_logs')
    processes=[];files=[]
    def start(command,name):
        stream=(out/name).open('w');files.append(stream)
        p=subprocess.Popen(command,env=env,cwd=root,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);return p
    try:
        visual='true' if args.gui else 'false'
        launch=start(['ros2','launch','forklift_bringup','p5.launch.py',f'gui:={visual}',f'rviz:={visual}',f'lift_test_mass:={args.test_mass}'],'launch.log')
        start(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge',
            '/world/forklift/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '--ros-args','-r','/world/forklift/dynamic_pose/info:=/test/lift_poses'],'pose_bridge.log')
        command=['python3','tests/check_lift.py','--output',str(out),'--repetitions',str(args.repetitions),
                 '--launch-pid',str(launch.pid),'--test-mass',str(args.test_mass)]
        if args.nominal_only:command.append('--nominal-only')
        p=start(command,'test.log');code=p.wait(timeout=900)
        print((out/'test.log').read_text());return code
    finally:
        for p in reversed(processes):
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:pass
            try:os.killpg(p.pid,signal.SIGTERM)
            except ProcessLookupError:pass
        for stream in files:stream.close()


if __name__=='__main__':sys.exit(main())
