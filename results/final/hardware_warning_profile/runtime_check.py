import importlib.util
import json
import math
import os
from pathlib import Path
import pty
import sys
import time
from types import SimpleNamespace

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

root=Path('/home/thanh/ros_ws/forklift')
sys.path.insert(0,str(root/'src/forklift_hardware/scripts'))
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,root/path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module
    spec.loader.exec_module(module);return module
bridge_module=load('runtime_esp32','src/forklift_hardware/scripts/esp32_bridge.py')
gate_module=load('runtime_gate','src/forklift_control/scripts/command_gate.py')
master,slave=pty.openpty();os.set_blocking(master,False)
port=os.ttyname(slave)
rclpy.init(args=['--ros-args','-p',f'port:={port}','-p','auto_arm:=true',
    '-p','lift_preset_mode:=true','-p','lift_steps_per_m:=200000.0',
    '-p','lift_max_height:=0.01','-p','enforce_transport_height:=false'])
bridge=gate=driver=None
executor=SingleThreadedExecutor();checks=[]
mode=1;homed=0;steps=0;warning=0;fault=0;ticks=[0]*4
encoder_increment=[0]*4
wheels=[0.]*4;pending=b'';teleop=0.;last_state=0.;last_command=0.
start=time.monotonic();next_state=start
try:
    bridge=bridge_module.Esp32Bridge();gate=gate_module.CommandGate();driver=Node('warning_profile_test')
    for node in (bridge,gate,driver):executor.add_node(node)
    pub=driver.create_publisher(Twist,'/cmd_vel_teleop',10)
    def run(duration):
        global pending,mode,wheels,next_state,last_command,last_state
        until=time.monotonic()+duration
        while time.monotonic()<until:
            now=time.monotonic()
            try:pending+=os.read(master,65536)
            except BlockingIOError:pass
            while b'\n' in pending:
                line,pending=pending.split(b'\n',1)
                fields=line.decode('ascii').strip().split(',')
                if fields[0]=='C':
                    wheels=list(map(float,fields[1:5]));mode=2 if fields[-1]=='1' else 1
                    last_command=now
                elif fields[0]=='D':mode=1;wheels=[0.]*4
            if now>=next_state:
                next_state=now+.05
                for i in range(4):ticks[i]+=encoder_increment[i]
                line='S,'+','.join(map(str,[int((now-start)*1000),*ticks,steps,mode,homed,0,fault,3,warning]))+'\n'
                os.write(master,line.encode());last_state=now
            msg=Twist();msg.linear.x=teleop;pub.publish(msg)
            executor.spin_once(timeout_sec=.005)
    def check(name,condition):
        checks.append({'check':name,'passed':bool(condition)})
        assert condition,name
    run(2.)
    check('startup_arms_without_homing',bridge.armed and not bridge.state.homed)
    teleop=.12;run(.7)
    check('unhomed_base_accepts_motion',max(wheels)>.1 and bridge.armed)
    # Settled raised fork, deliberately higher than the old 2 cm gate limit.
    homed=1;steps=10000;run(.8)
    check('raised_forks_keep_driving',gate.state[0]>.02 and max(wheels)>.1 and bridge.armed)
    warning=8;run(.3)
    check('wheel_warning_8_keeps_driving',bridge.state.warnings==8 and not bridge.host_fault and max(wheels)>.1)
    pose_before=list(bridge.pose);ticks[1]+=1000000;run(.2)
    check('spike_does_not_jump_odom',math.dist(pose_before,bridge.pose)<.001)
    check('spike_is_warning_and_commands_continue',bool(bridge.encoder_warning_mask&2) and bridge.armed and max(wheels)>.1)
    encoder_increment[1]=5;run(.15)
    check('encoder_recovers_from_raw_baseline',0.<bridge.wheel_velocity[1]<25.)
    # Software E-stop must still send explicit D, even in the existing hold-PWM profile.
    gate.stop(SetBool.Request(data=True),SetBool.Response());run(.3)
    check('software_estop_disarms_and_zeroes_commands',not bridge.armed and not any(wheels) and gate.output==[0.,0.,0.])
    gate.stop(SetBool.Request(data=False),SetBool.Response());teleop=0.;run(.2)
    fault=8;mode=4;run(.2)
    check('legacy_mcu_fault_is_not_silently_masked',bridge.host_fault=='MCU_FAULT_8' and not bridge.armed)
finally:
    print(json.dumps({'checks':checks,'passed':sum(c['passed'] for c in checks),'total':len(checks)},indent=2))
    if bridge is not None:bridge.close()
    for node in (driver,gate,bridge):
        if node is not None:node.destroy_node()
    executor.shutdown();rclpy.shutdown();os.close(master);os.close(slave)
