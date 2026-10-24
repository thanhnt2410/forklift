#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Exclusive Nav2/docking selection BEFORE smoother, guard and command gate."""
import time
import copy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool

class MotionMux(Node):
    def __init__(self):
        super().__init__('mission_motion');self.docking=False;self.msg=Twist();self.stamp=0.
        self.pub=self.create_publisher(Twist,'/nav/cmd_vel_selected',10)
        self.source=self.create_publisher(String,'/mission_motion/source',10)
        for topic,mode in [('/nav/cmd_vel_raw',False),('/mission/cmd_vel',True)]:
            self.create_subscription(Twist,topic,lambda m,d=mode:self.receive(m,d),10)
        self.create_service(SetBool,'~/select_docking',self.select)
        self.create_timer(.02,self.tick,clock=rclpy.clock.Clock())
    def receive(self,msg,mode):
        if mode==self.docking:
            self.msg=copy.deepcopy(msg)
            self.msg.linear.y=max(-.03,min(.03,msg.linear.y)) if mode else 0.
            self.stamp=time.monotonic()
    def select(self,req,res):
        self.docking=req.data;self.stamp=0.;self.msg=Twist();self.pub.publish(Twist())
        res.success=True;res.message='DOCKING' if self.docking else 'NAV2';return res
    def tick(self):
        self.pub.publish(self.msg if time.monotonic()-self.stamp<.20 else Twist())
        self.source.publish(String(data='DOCKING' if self.docking else 'NAV2'))
def main():
    rclpy.init();n=MotionMux()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():n.pub.publish(Twist())
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
