#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Reproducible P4 simulator + map survey or navigation acceptance."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser();parser.add_argument('phase',choices=['mapping','navigation'])
    parser.add_argument('--gui',action='store_true');parser.add_argument('--smoke',action='store_true')
    parser.add_argument('--scenario',choices=['all','obstacle','return'],default='all')
    parser.add_argument('--domain',type=int,default=113);parser.add_argument('--output',default='results/p4')
    parser.add_argument('--attach',action='store_true',help='Use the existing simulator/domain/partition')
    args=parser.parse_args();root=Path(__file__).resolve().parents[1];out=(root/args.output).resolve();out.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy()
    if not args.attach:env.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',IGN_PARTITION=f'forklift_p4_{os.getpid()}',ROS_LOG_DIR=f'/tmp/forklift_p4_{os.getpid()}_logs',IGN_LOG_PATH=f'/tmp/forklift_p4_{os.getpid()}_ign')
    processes=[];streams=[]
    def start(command,logfile):
        stream=(out/logfile).open('w');streams.append(stream)
        p=subprocess.Popen(command,env=env,cwd=root,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
    try:
        if not args.attach:
            visual='true' if args.gui else 'false'
            mode='mapping' if args.phase=='mapping' else 'localization'
            start(['ros2','launch','forklift_bringup','sim.launch.py','world:=warehouse',f'navigation:={mode}',f'gui:={visual}',f'rviz:={visual}','auto_home:=true',f'map:={root}/src/forklift_navigation/maps/warehouse.yaml'],f'{args.phase}_launch.log')
        if args.phase=='navigation':
            start(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge',
                   '/world/forklift/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                   '--ros-args','-r','/world/forklift/dynamic_pose/info:=/test/poses'],'physical_pose_bridge.log')
            command=['python3','tests/check_navigation.py','--output',str(out),'--scenario',args.scenario]+(['--smoke'] if args.smoke else [])
        else:command=['python3','scripts/map_warehouse.py']
        p=start(command,f'{args.phase}_test.log')
        code=p.wait(timeout=480 if args.phase=='navigation' else 220)
        print((out/f'{args.phase}_test.log').read_text());return code
    finally:
        for p in reversed(processes):
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:pass
            try:os.killpg(p.pid,signal.SIGTERM)
            except ProcessLookupError:pass
        for stream in streams:stream.close()


if __name__=='__main__':sys.exit(main())
