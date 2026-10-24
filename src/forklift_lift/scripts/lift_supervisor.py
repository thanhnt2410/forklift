#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""P5 SIM action server. No physical ports, GPIO or firmware access."""
import math
import time
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from forklift_interfaces.action import HomeLift, MoveLift
from forklift_interfaces.msg import LiftCommand, LiftSensors, LiftState
from forklift_lift.core import LiftCore


class LiftSupervisor(Node):
    def __init__(self):
        super().__init__('lift_supervisor')
        defaults=dict(max_height=.24,move_velocity=.04,home_fast_velocity=.02,
                      home_slow_velocity=.006,home_backoff=.008,position_tolerance=.002,
                      state_timeout=.4,home_timeout=60.)
        for key,value in defaults.items():self.declare_parameter(key,value)
        p={key:float(self.get_parameter(key).value) for key in defaults}
        if not all(math.isfinite(v) and v>0 for v in p.values()) or p['max_height']>.24:
            raise ValueError('Invalid SIM lift limits')
        self.core=LiftCore(p);self.reserved=False;self.goal_handle=None;self.completion=None
        self.declare_parameter('auto_home',False)
        self.auto_home=bool(self.get_parameter('auto_home').value);self.auto_attempted=False
        self.last_fault='';self.last_diag=0.;self.last_visual=0.
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.busy=self.create_publisher(Bool,'/lift/moving',latched)
        self.status=self.create_publisher(String,'/lift/status',10)
        self.state_pub=self.create_publisher(LiftState,'/lift/state',latched)
        self.command_pub=self.create_publisher(LiftCommand,'/lift/sim_command',10)
        self.diag=self.create_publisher(DiagnosticArray,'/diagnostics',10)
        self.marker_pub=self.create_publisher(Marker,'/lift/marker',latched)
        self.create_subscription(LiftSensors,'/lift/sensors',self.sensors,10)
        self.create_subscription(Bool,'/safety/estop',self.estop,latched)
        self.create_service(Trigger,'/lift/reset',self.reset)
        self.create_service(Trigger,'/lift/stop',self.stop)
        self.home_server=ActionServer(self,HomeLift,'/lift/home',execute_callback=self.execute_home,
            goal_callback=lambda g:self.accept('home',0.,g.timeout),cancel_callback=self.cancel)
        self.move_server=ActionServer(self,MoveLift,'/lift/move',execute_callback=self.execute_move,
            goal_callback=lambda g:self.accept('move',g.height,g.timeout),cancel_callback=self.cancel)
        self.create_timer(.02,self.tick,clock=rclpy.clock.Clock())

    def sensors(self,msg):self.core.receive(msg,time.monotonic())
    def estop(self,msg):self.core.estopped=msg.data

    def accept(self,kind,height,timeout):
        if self.reserved or not self.core.acceptable(kind,height,timeout,time.monotonic()):
            self.get_logger().warning(f'Rejected {kind}: state={self.core.state}, homed={self.core.homed}, fault={self.core.fault}')
            return GoalResponse.REJECT
        self.reserved=True
        # Lock wheels before the first lift trajectory can be issued.
        self.busy.publish(Bool(data=True))
        return GoalResponse.ACCEPT

    def cancel(self,handle):return CancelResponse.ACCEPT

    async def execute(self,handle,kind,action_type):
        self.goal_handle=handle;self.completion=Future()
        request=handle.request
        started=self.core.start(kind,request.height if kind=='move' else 0.,request.timeout,time.monotonic())
        if not started:
            handle.abort();self.goal_handle=None;self.reserved=False
            return action_type.Result(success=False,code='STATE_CHANGED',height=self.core.height,homed=self.core.homed)
        data=await self.completion
        result=action_type.Result(success=data['success'],code=data['code'],height=data['height'],homed=data['homed'])
        return result

    async def execute_home(self,handle):return await self.execute(handle,'home',HomeLift)
    async def execute_move(self,handle):return await self.execute(handle,'move',MoveLift)

    def reset(self,req,res):
        res.success=not self.reserved and self.core.reset(time.monotonic())
        res.message='Reset to UNHOMED; home again' if res.success else 'Cause still present, moving or stale feedback'
        return res

    def stop(self,req,res):
        if self.core.active:self.core.finish(False,'STOPPED',canceled=True)
        else:self.core.command()
        res.success=True;res.message='Holding actual SIM position';return res

    def tick(self):
        now=time.monotonic();c=self.core
        if self.auto_home and not self.auto_attempted and not self.reserved and c.acceptable('home',0.,0.,now):
            self.auto_attempted=True;c.start('home',0.,0.,now)
            self.get_logger().info('SIM auto-home started (configured once at startup)')
        if self.goal_handle is not None and self.goal_handle.is_cancel_requested and c.active:
            c.finish(False,'CANCELED',canceled=True)
        c.tick(now)
        self.command_pub.publish(LiftCommand(header=self.header(),sequence=c.sequence,
            enabled=c.enabled,position=c.command_position,duration=c.duration))
        inhibited=c.inhibit(now) or self.reserved
        self.busy.publish(Bool(data=inhibited))
        self.status.publish(String(data=c.fault or c.state))
        s=c.sensor
        self.state_pub.publish(LiftState(header=self.header(),state=c.state,fault_code=c.fault,
            homed=c.homed,moving=c.active,inhibited=inhibited,estopped=c.estopped,
            height=c.height,target=c.target,home_offset=c.offset,
            lower_active=s.lower_active if s else False,upper_active=s.upper_active if s else False))
        if self.goal_handle is not None:
            cls=HomeLift if c.kind=='home' else MoveLift
            feedback=cls.Feedback(state=c.state,height=c.height,
                lower_active=s.lower_active if s else False,upper_active=s.upper_active if s else False)
            if c.kind=='move':feedback.target=c.target
            self.goal_handle.publish_feedback(feedback)
            if c.outcome is not None:
                data=c.outcome;c.outcome=None
                if data['canceled'] and self.goal_handle.is_cancel_requested:self.goal_handle.canceled()
                elif data['success']:self.goal_handle.succeed()
                else:self.goal_handle.abort()
                self.completion.set_result(data);self.goal_handle=None;self.reserved=False
        elif c.outcome is not None:
            self.get_logger().info(f'Internal home result: {c.outcome}');c.outcome=None
        if c.fault and c.fault!=self.last_fault:self.get_logger().error(f'Lift fault latched: {c.fault}')
        self.last_fault=c.fault
        if now-self.last_visual>.2:
            self.last_visual=now
            marker=Marker(header=self.header(),ns='lift_status',id=0,type=Marker.TEXT_VIEW_FACING,action=Marker.ADD)
            marker.pose.orientation.w=1.;marker.pose.position.z=.55;marker.scale.z=.035
            marker.lifetime=Duration(sec=1)
            marker.color.a=1.;marker.color.r=1. if c.fault or not c.homed else .2
            marker.color.g=.2 if c.fault else .9;marker.color.b=.2
            marker.text=f'LIFT:{c.fault or c.state}\nH={max(0.,c.height)*100:.1f}cm'
            if not c.homed and not c.active:marker.text+='\nHOME_REQUIRED'
            self.marker_pub.publish(marker)
        if now-self.last_diag>1.:
            self.last_diag=now
            status=DiagnosticStatus(name='forklift_lift/supervisor',hardware_id='SIM_ONLY',
                level=DiagnosticStatus.ERROR if c.fault else (DiagnosticStatus.OK if c.homed else DiagnosticStatus.WARN),
                message=c.fault or c.state,values=[KeyValue(key='homed',value=str(c.homed)),KeyValue(key='height_m',value=str(c.height))])
            self.diag.publish(DiagnosticArray(header=self.header(),status=[status]))

    def header(self):
        from std_msgs.msg import Header
        return Header(stamp=self.get_clock().now().to_msg(),frame_id='base_link')


def main():
    rclpy.init();node=LiftSupervisor()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():
            node.core.command();node.command_pub.publish(LiftCommand(sequence=node.core.sequence,enabled=False,duration=.15))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
