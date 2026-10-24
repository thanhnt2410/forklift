#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Record only the Gazebo window while running one independently scored P6 demo."""
import argparse
import ctypes
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

parser=argparse.ArgumentParser()
parser.add_argument('--output',default='results/p6/demo')
parser.add_argument('--domain',type=int,default=155)
parser.add_argument('--controller',default='mppi',choices=['rpp','mppi'])
args=parser.parse_args()
root=Path(__file__).resolve().parents[1];out=(root/args.output).resolve();out.mkdir(parents=True,exist_ok=True)
run=subprocess.Popen([sys.executable,str(root/'scripts/run_p6_checks.py'),'--gui','--controller',args.controller,
    '--scenario','nominal','--repetitions','1','--domain',str(args.domain),'--output',str(out)],cwd=root,stdout=(out/'runner.log').open('w'),stderr=subprocess.STDOUT)
video=None;log=None
try:
    deadline=time.monotonic()+90;window=None
    while time.monotonic()<deadline and run.poll() is None:
        tree=subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True).stdout
        matches=re.findall(r'(0x[0-9a-f]+) "Gazebo"',tree)
        if matches:window=int(matches[-1],16);break
        time.sleep(.25)
    if window is None:raise RuntimeError('Gazebo window did not appear')
    x=ctypes.CDLL('libX11.so.6');x.XOpenDisplay.restype=ctypes.c_void_p
    x.XRaiseWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong]
    x.XMoveResizeWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint]
    x.XFlush.argtypes=[ctypes.c_void_p]
    display=x.XOpenDisplay(None)
    if not display:raise RuntimeError('X display unavailable')
    x.XMoveResizeWindow(display,window,10,30,1100,760);x.XRaiseWindow(display,window);x.XFlush(display)
    time.sleep(1.)
    log=(out/'recording.log').open('w')
    video=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','warning','-y','-f','x11grab','-framerate','15',
        '-window_id',str(window),'-i',os.environ.get('DISPLAY',':1'),'-vf','pad=ceil(iw/2)*2:ceil(ih/2)*2','-c:v','libx264','-threads','2','-preset','veryfast','-crf','23','-pix_fmt','yuv420p',str(out/'mission_demo.mp4')],stdout=log,stderr=subprocess.STDOUT)
    # Keep the Gazebo window visible if RViz finishes opening after it.
    until=time.monotonic()+15
    while time.monotonic()<until and run.poll() is None:
        x.XRaiseWindow(display,window);x.XFlush(display);time.sleep(.5)
    code=run.wait(timeout=600)
finally:
    if video is not None and video.poll() is None:
        video.send_signal(signal.SIGINT)
        try:video.wait(timeout=8)
        except subprocess.TimeoutExpired:video.kill()
    if log is not None:log.close()
    if run.poll() is None:
        run.send_signal(signal.SIGINT)
        try:run.wait(timeout=30)
        except subprocess.TimeoutExpired:run.kill()
sys.exit(code)
