#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Virtual switches, trajectory transport and independent local watchdog.

Switches and feedback derive from Gazebo joint state, never commanded height.
Validity flags model a diagnosed input channel, NOT an actual NC wiring circuit.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from forklift_interfaces.msg import LiftCommand,LiftSensors


class LiftSimIO(Node):
    def __init__(self):
        super().__init__('lift_sim_io')
        for key,value in dict(lower_position=.0015,upper_position=.245,switch_hysteresis=.0005,
                              command_timeout=.3,lower_mode='normal',upper_mode='normal',
                              drop_feedback=False,jammed=False).items():self.declare_parameter(key,value)
        self.add_on_set_parameters_callback(self.parameters)
        self.state=None;self.command_stamp=0.;self.sequence=None;self.enabled=False
        self.watchdog=False;self.error=False;self.estopped=False;self.controller_handle=None
        self.lower=False;self.upper=False
        self.client=ActionClient(self,FollowJointTrajectory,'/lift_controller/follow_joint_trajectory')
        self.hold_pub=self.create_publisher(JointTrajectory,'/lift_controller/joint_trajectory',10)
        self.pub=self.create_publisher(LiftSensors,'/lift/sensors',10)
        self.create_subscription(JointState,'/joint_states',self.joints,10)
        self.create_subscription(LiftCommand,'/lift/sim_command',self.command,10)
        self.create_subscription(Bool,'/safety/estop',self.estop,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger,'/lift/sim_io/reset',self.reset)
        self.create_timer(.02,self.tick,clock=rclpy.clock.Clock())

    def p(self,key):return self.get_parameter(key).value

    def parameters(self,values):
        for value in values:
            if value.name in ('lower_mode','upper_mode') and value.value not in ('normal','stuck_active','stuck_inactive','disconnected'):
                return SetParametersResult(successful=False,reason='Invalid switch fault mode')
            if value.name not in ('lower_mode','upper_mode','drop_feedback','jammed'):
                return SetParametersResult(successful=False,reason='Geometry/watchdog configuration is fixed at launch')
        return SetParametersResult(successful=True)

    def joints(self,msg):
        if 'lift_joint' not in msg.name:return
        i=msg.name.index('lift_joint')
        if i>=len(msg.position) or i>=len(msg.velocity):return
        if not all(math.isfinite(v) for v in [msg.position[i],msg.velocity[i]]):return
        wheels=[v for name,v in zip(msg.name,msg.velocity) if name.endswith('_wheel_joint')]
        self.state=(msg.position[i],msg.velocity[i],time.monotonic(),len(wheels)==4 and max(abs(v) for v in wheels)<.05)

    def trajectory(self,position,duration):
        point=JointTrajectoryPoint(positions=[position],velocities=[0.],
            time_from_start=Duration(sec=int(duration),nanosec=int(duration%1*1e9)))
        return JointTrajectory(joint_names=['lift_joint'],points=[point])

    def hold(self):
        if self.controller_handle is not None:self.controller_handle.cancel_goal_async();self.controller_handle=None
        if self.state is not None:
            self.hold_pub.publish(self.trajectory(max(0.,min(.25,self.state[0])),.15))

    def estop(self,msg):
        self.estopped=msg.data
        if msg.data:self.hold();self.enabled=False

    def command(self,msg):
        self.command_stamp=time.monotonic()
        if not msg.enabled:
            if self.enabled or self.sequence!=msg.sequence:self.hold()
            self.enabled=False;self.sequence=msg.sequence
            # A disabled fresh heartbeat acknowledges a watchdog stop; the
            # supervisor still keeps its own fault and invalidates homing.
            self.watchdog=False
            return
        if (self.estopped or self.watchdog or self.error or self.p('jammed')
                or self.state is None or not self.client.server_is_ready()):return
        if not all(math.isfinite(v) for v in [msg.position,msg.duration]) or not 0<=msg.position<=.25 or not .1<=msg.duration<=120:
            self.error=True;self.hold();return
        if self.sequence==msg.sequence:return
        self.sequence=msg.sequence;self.enabled=True
        goal=FollowJointTrajectory.Goal();goal.trajectory=self.trajectory(msg.position,msg.duration)
        sequence=msg.sequence
        future=self.client.send_goal_async(goal)
        def accepted(f):
            handle=f.result()
            if sequence!=self.sequence:
                if handle.accepted:handle.cancel_goal_async()
                return
            if not handle.accepted:self.error=True;self.hold();return
            self.controller_handle=handle
            result=handle.get_result_async()
            def finished(r):
                if sequence==self.sequence and self.enabled and r.result().status!=4:
                    self.error=True;self.hold()
            result.add_done_callback(finished)
        future.add_done_callback(accepted)

    def reset(self,req,res):
        res.success=(not self.enabled and not self.estopped and not self.p('jammed') and self.client.server_is_ready())
        if res.success:self.error=False;self.watchdog=False
        res.message='SIM I/O reset' if res.success else 'Cause still present';return res

    def tick(self):
        now=time.monotonic()
        if self.enabled and (self.p('jammed') or now-self.command_stamp>self.p('command_timeout')):
            self.hold();self.enabled=False
            if not self.p('jammed'):self.watchdog=True
        if self.state is None:return
        position,velocity,received,stationary=self.state
        lower=self.p('lower_position');upper=self.p('upper_position');h=self.p('switch_hysteresis')
        self.lower=position<=lower+(h if self.lower else 0.)
        self.upper=position>=upper-(h if self.upper else 0.)
        def switch(name,value):
            mode=self.p(name+'_mode')
            return (value if mode=='normal' else mode=='stuck_active'),mode!='disconnected'
        lo,lov=switch('lower',self.lower);hi,hiv=switch('upper',self.upper)
        if not self.p('drop_feedback'):
            msg=LiftSensors(position=position,velocity=velocity,feedback_valid=now-received<.3,
                lower_active=lo,upper_active=hi,lower_valid=lov,upper_valid=hiv,
                controller_ok=self.client.server_is_ready() and not self.error and not self.p('jammed') and not self.watchdog,
                base_stationary=stationary,watchdog_latched=self.watchdog)
            msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link';self.pub.publish(msg)


def main():
    rclpy.init();node=LiftSimIO()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.hold();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
