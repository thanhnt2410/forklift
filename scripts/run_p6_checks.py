#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Isolated Fortress P6 acceptance. Never opens real ports or flashes firmware."""
import argparse
import json
import hashlib
import shutil
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

def main():
    p=argparse.ArgumentParser();p.add_argument('--gui',action='store_true');p.add_argument('--repetitions',type=int,default=1)
    p.add_argument('--output',default='results/p6/smoke');p.add_argument('--domain',type=int,default=126)
    p.add_argument('--controller',default='mppi',choices=['rpp','mppi'])
    p.add_argument('--scenario',default='nominal',choices=['nominal','cancel','scan_loss','scan_loss_nav','imu_loss','estop','missing_pallet','misaligned','blocked','blocked_nav','lift_fault','tf_loss','load_loss','cancel_loaded','cancel_orient','cancel_reverse'])
    a=p.parse_args();root=Path(__file__).resolve().parents[1];out=(root/a.output).resolve();out.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1',IGN_PARTITION=f'forklift_p6_{os.getpid()}',ROS_LOG_DIR=f'/tmp/forklift_p6_{os.getpid()}_logs')
    (out/'run.json').write_text(json.dumps(dict(domain=a.domain,partition=env['IGN_PARTITION'],repetitions=a.repetitions,scenario=a.scenario,controller=a.controller,gui=a.gui,physical_hardware_used=False),indent=2)+'\n')
    snapshot=out/'config';snapshot.mkdir(exist_ok=True)
    inputs=['src/forklift_gazebo/worlds/p6.sdf','src/forklift_task_manager/config/stations.yaml',
            'src/forklift_description/config/robot.yaml','src/forklift_navigation/config/nav2.yaml',
            'src/forklift_control/config/command_gate.yaml','src/forklift_lift/config/lift.yaml',
            'src/forklift_task_manager/config/approved_route.json']
    if a.controller=='mppi':
        inputs.extend(['src/forklift_navigation/config/nav2_mppi.yaml',
                       'src/forklift_navigation/src/guarded_mppi_controller.cpp',
                       'src/forklift_navigation/include/forklift_navigation/route_guard.hpp'])
    for file in inputs:shutil.copy2(root/file,snapshot/Path(file).name)
    (snapshot/'manifest.json').write_text(json.dumps({f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in inputs},indent=2)+'\n')
    processes=[];files=[]
    def start(cmd,name):
        f=(out/name).open('w');files.append(f);proc=subprocess.Popen(cmd,env=env,cwd=root,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc);return proc
    try:
        visual='true' if a.gui else 'false'
        start(['ros2','launch','forklift_bringup','p6.launch.py',f'gui:={visual}',f'rviz:={visual}',f'controller:={a.controller}'],'launch.log')
        start(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge','/world/forklift/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V','--ros-args','-r','/world/forklift/dynamic_pose/info:=/test/p6_poses'],'pose_bridge.log')
        test=start(['python3','tests/check_mission.py','--output',str(out),'--repetitions',str(a.repetitions),'--scenario',a.scenario],'test.log')
        code=test.wait(timeout=400*a.repetitions+120);print((out/'test.log').read_text());return code
    finally:
        # Launch may leave detached children; address this run's processes by
        # partition/domain, never by a process group that could include a caller.
        ancestors={os.getpid()};parent=os.getppid()
        while parent>1 and parent not in ancestors:
            ancestors.add(parent)
            try:parent=int(Path(f'/proc/{parent}/stat').read_text().rsplit(')',1)[1].split()[1])
            except (OSError,ValueError,IndexError):break
        marker=('IGN_PARTITION='+env['IGN_PARTITION']).encode()
        domain=('ROS_DOMAIN_ID='+env['ROS_DOMAIN_ID']).encode()
        def remaining():
            ids=[]
            for procdir in Path('/proc').glob('[0-9]*'):
                try:
                    pid=int(procdir.name)
                    if pid in ancestors:continue
                    fields=(procdir/'environ').read_bytes().split(b'\0')
                    if marker in fields and domain in fields:ids.append(pid)
                except (OSError,ValueError):pass
            return ids
        def stop(ids, signum):
            for pid in ids:
                try:os.kill(pid,signum)
                except ProcessLookupError:pass
        initial=remaining()
        for signum,seconds in ((signal.SIGINT,3.),(signal.SIGTERM,2.),(signal.SIGKILL,1.)):
            stop(remaining(),signum);deadline=time.monotonic()+seconds
            while remaining() and time.monotonic()<deadline:time.sleep(.1)
        for proc in processes:
            try:proc.wait(timeout=.2)
            except subprocess.TimeoutExpired:pass
        (out/'cleanup.json').write_text(json.dumps(dict(initial_pids=initial,remaining_pids=remaining(),partition=env['IGN_PARTITION']),indent=2)+'\n')
        for f in files:f.close()
if __name__=='__main__':sys.exit(main())
