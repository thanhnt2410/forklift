#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
import os
from pathlib import Path
import signal
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
out=root/'results/p3';out.mkdir(parents=True,exist_ok=True)
cases=sys.argv[1:] or ['nominal','drop','corrupt','reset','disconnect']
failed=[]
for index,case in enumerate(cases):
    env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(101+index),ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=f'/tmp/forklift-p3-{case}-logs')
    with (out/f'{case}_launch.log').open('w') as stream:
        launch=subprocess.Popen(['ros2','launch','forklift_hardware','mock_hardware.launch.py'],env=env,cwd=root,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            test=subprocess.run(['python3','tests/check_mock.py','--case',case],env=env,cwd=root,timeout=70,capture_output=True,text=True)
            (out/f'{case}_test.log').write_text(test.stdout+test.stderr)
            print(case,test.returncode,test.stdout,flush=True)
            if test.returncode:failed.append(case)
        finally:
            launch.send_signal(signal.SIGINT)
            try:launch.wait(timeout=8)
            except subprocess.TimeoutExpired:pass
            try:os.killpg(launch.pid,signal.SIGTERM)
            except ProcessLookupError:pass
sys.exit(bool(failed))
