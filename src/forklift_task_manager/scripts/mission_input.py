#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Collect RViz A/B poses; explicit start snapshots them into one mission goal."""
import copy
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from forklift_interfaces.action import RunMission
from forklift_task_manager.dynamic_mission import pose_values


class MissionInput(Node):
    def __init__(self):
        super().__init__('mission_input')
        self.poses={};self.pending=False;self.mission_handle=None;self.message='Chon A va B tren RViz'
        self.declare_parameter('timeout',600.)
        self.client=ActionClient(self,RunMission,'/mission/run',callback_group=ReentrantCallbackGroup())
        self.markers=self.create_publisher(MarkerArray,'/mission/input_markers',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status=self.create_publisher(String,'/mission/input_status',10)
        for key in ('pick','drop'):
            self.create_subscription(PoseStamped,'/mission/'+key+'_pose',lambda msg,k=key:self.receive(k,msg),10)
        self.create_service(Trigger,'/mission/start',self.start)
        self.create_service(Trigger,'/mission/clear',self.clear)
        self.create_timer(.5,self.publish)

    def receive(self,key,msg):
        if self.pending or self.mission_handle is not None:
            self.message='Dang chay: khong thay doi A/B';return
        try:pose_values(msg)
        except ValueError as e:self.message=str(e);return
        self.poses[key]=copy.deepcopy(msg);self.message='Da nhan '+key;self.publish()

    async def start(self,req,res):
        if self.pending or self.mission_handle is not None:
            res.success=False;res.message='MISSION_BUSY';return res
        if set(self.poses)!={'pick','drop'} or not self.client.server_is_ready():
            res.success=False;res.message='Set A/B and wait for mission action server';return res
        self.pending=True
        try:
            goal=RunMission.Goal(mission='pick_and_place',timeout=float(self.get_parameter('timeout').value),
                                 pick_pose=copy.deepcopy(self.poses['pick']),drop_pose=copy.deepcopy(self.poses['drop']))
            self.mission_handle=await self.client.send_goal_async(goal)
            if not self.mission_handle.accepted:
                self.mission_handle=None;res.success=False;res.message='MISSION_REJECTED: check /mission/status and /navigation/status'
            else:
                self.mission_handle.get_result_async().add_done_callback(self.finished)
                res.success=True;res.message='MISSION_ACCEPTED'
        except Exception as e:
            res.success=False;res.message=str(e);self.mission_handle=None
        finally:self.pending=False
        self.message=res.message;return res

    def finished(self,future):
        try:
            code=future.result().result.code
            self.message={
                'NO_VALID_PATH_NAV_PICK':'A unreachable: move staging pose back (~0.65 m before pallet face)',
                'NO_VALID_PATH_NAV_DROP':'B unreachable: choose a free release pose with room for the loaded robot',
            }.get(code,code)
        except Exception as e:self.message=str(e)
        self.mission_handle=None

    def clear(self,req,res):
        res.success=not self.pending and self.mission_handle is None
        if res.success:self.poses.clear();self.message='Chon A va B tren RViz';self.publish()
        res.message=self.message;return res

    def publish(self):
        markers=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
        for index,(key,pose) in enumerate(self.poses.items()):
            arrow=Marker();arrow.header=pose.header;arrow.ns='mission_inputs';arrow.id=index*2
            arrow.type=Marker.ARROW;arrow.action=Marker.ADD;arrow.pose=copy.deepcopy(pose.pose)
            arrow.scale.x=.35;arrow.scale.y=.06;arrow.scale.z=.08;arrow.color.a=1.
            arrow.color.g=1. if key=='drop' else .5;arrow.color.b=1. if key=='pick' else .2
            markers.markers.append(arrow)
            label=copy.deepcopy(arrow);label.id+=1;label.type=Marker.TEXT_VIEW_FACING
            label.pose.position.z=.35;label.scale.z=.12;label.text='A: PICK staging' if key=='pick' else 'B: RELEASE robot pose'
            markers.markers.append(label)
        self.markers.publish(markers)
        self.status.publish(String(data=json.dumps(dict(poses=list(self.poses),busy=self.pending or self.mission_handle is not None,message=self.message))))


def main():
    rclpy.init();node=MissionInput()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
