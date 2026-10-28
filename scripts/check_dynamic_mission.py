#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""SIM acceptance: independent physical pose oracle, never a controller input."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool, Empty
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from forklift_interfaces.action import RunMission
from forklift_interfaces.msg import LiftState


def yaw(q):return math.atan2(2*q.w*q.z,1-2*q.z*q.z)
def angle(a):return math.atan2(math.sin(a),math.cos(a))
def pose(x,y,a=0.):
    p=PoseStamped();p.header.frame_id='map';p.pose.position.x=x;p.pose.position.y=y
    p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2);return p


class Check:
    def __init__(self,out,scenario):
        self.out=Path(out);self.scenario=scenario;self.n=rclpy.create_node('dynamic_mission_acceptance')
        self.data={};self.poses={};self.rows=[];self.checks=[];self.last=0.;self.events=[]
        self.tf=Buffer();self.listener=TransformListener(self.tf,self.n)
        self.routes=[];self.active_paths=[];self.navigation_goals=0
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.n.create_subscription(String,'/mission/route_info',self.route_received,latched)
        self.n.create_subscription(NavPath,'/mission/active_path',
            lambda m:self.active_paths.append(m) if m.poses else None,latched)
        from action_msgs.msg import GoalStatusArray
        self.n.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',
            lambda m:setattr(self,'navigation_goals',max(self.navigation_goals,len(m.status_list))),latched)
        for cls,topic,key,qos in [(String,'/mission/status','mission',10),(Bool,'/navigation/ready','ready',10),
                (LiftState,'/lift/state','lift',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
                (Twist,'/cmd_vel','cmd',10),(Odometry,'/odom','odom',10),
                (String,'/mission/input_status','input',10)]:
            self.n.create_subscription(cls,topic,lambda m,k=key:self.data.update({k:m}),qos)
        self.n.create_subscription(TFMessage,'/test/p6_poses',self.physical_pose,10)
        self.client=ActionClient(self.n,RunMission,'/mission/run')
        self.publishers={key:self.n.create_publisher(PoseStamped,'/mission/'+key+'_pose',10) for key in ('pick','drop')}

    def physical_pose(self,msg):
        for t in msg.transforms:self.poses[t.child_frame_id]=t.transform
    def route_received(self,msg):
        route=json.loads(msg.data)
        if route.get('segments'):
            route['received_before_motion']=not any(max(abs(row[k]) for k in ('vx','vy','wz'))>.001 for row in self.rows)
            self.routes.append(route)

    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(self.n,timeout_sec=.02)
            if time.monotonic()-self.last<.1 or 'mission' not in self.data:continue
            self.last=time.monotonic();m=json.loads(self.data['mission'].data)
            if not self.events or self.events[-1]!=m['state']:
                self.events.append(m['state']);print(m['state'],flush=True)
            b=self.poses.get('forklift');p=self.poses.get('p6_pallet');cmd=self.data.get('cmd',Twist())
            if b and p:
                try:
                    localized=self.tf.lookup_transform('map','base_link',rclpy.time.Time()).transform
                    map_x,map_y,map_yaw=localized.translation.x,localized.translation.y,yaw(localized.rotation)
                except Exception:map_x=map_y=map_yaw=None
                self.rows.append(dict(t=self.last,state=m['state'],robot_x=b.translation.x,robot_y=b.translation.y,
                    robot_yaw=yaw(b.rotation),pallet_x=p.translation.x,pallet_y=p.translation.y,pallet_z=p.translation.z,
                    pallet_yaw=yaw(p.rotation),vx=cmd.linear.x,vy=cmd.linear.y,wz=cmd.angular.z,load=m['load_detected'],map_x=map_x,map_y=map_y,map_yaw=map_yaw))

    def wait(self,future,timeout=10.):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:self.spin(.05)
        if not future.done():raise TimeoutError('ROS_RESPONSE_TIMEOUT')
        return future.result()

    def call(self,cls,name,req):
        c=self.n.create_client(cls,name)
        if not c.wait_for_service(timeout_sec=5.):raise RuntimeError(name+' unavailable')
        try:return self.wait(c.call_async(req))
        finally:self.n.destroy_client(c)

    def mark(self,name,passed,**details):
        row=dict(test=name,passed=bool(passed),**details);self.checks.append(row);print(json.dumps(row),flush=True)

    def entity(self,service,kind,request):
        r=subprocess.run(['ign','service','-s',service,'--reqtype',kind,'--reptype','ignition.msgs.Boolean','--timeout','3000','--req',request],capture_output=True,text=True,timeout=5.)
        if r.returncode or 'true' not in r.stdout:raise RuntimeError(r.stdout+r.stderr)

    def run(self):
        end=time.monotonic()+90.
        while not ('ready' in self.data and self.data['ready'].data and 'lift' in self.data and self.data['lift'].homed and 'p6_pallet' in self.poses and 'input' in self.data):
            self.spin(.1)
            if time.monotonic()>end:
                lift=self.data.get('lift',LiftState())
                diagnostic=dict(topics=list(self.data),poses=list(self.poses),lift_state=lift.state,
                    lift_fault=lift.fault_code,homed=lift.homed,ready=bool(self.data.get('ready',Bool()).data))
                (self.out/'startup_diagnostics.json').write_text(json.dumps(diagnostic,indent=2)+'\n')
                raise RuntimeError('STARTUP_TIMEOUT: '+json.dumps(diagnostic))
        self.mark('startup_ready',True)
        if self.scenario=='early_direct':
            # Reproduce an already localized robot, using AMCL's scan updates;
            # never feed the simulator ground-truth pose into localization.
            for _ in range(8):
                self.call(Empty,'/request_nomotion_update',Empty.Request());self.spin(.3)
        self.mark('start_without_poses_rejected',not self.call(Trigger,'/mission/start',Trigger.Request()).success)
        assert self.client.wait_for_server(timeout_sec=5.)
        bad=RunMission.Goal(mission='pick_and_place',pick_pose=pose(.35,.65),drop_pose=pose(.72,-.60))
        bad.pick_pose.header.frame_id='odom'
        self.mark('invalid_frame_rejected',not self.wait(self.client.send_goal_async(bad)).accepted)
        # Intentional inaccurate A: both lateral position and heading differ.
        a=pose(.35,.70,.06);b=pose(.72,-.60)
        if self.scenario=='user_drop':b=pose(-.982239,-.801960,3.130279)
        requested_drop=[b.pose.position.x,b.pose.position.y,yaw(b.pose.orientation)]
        if self.scenario=='invalid_pick':a=pose(.892022,.570133,-.087618);b=pose(-.982239,-.801960,3.130279)
        if self.scenario=='invalid_drop':b=pose(1.0,.65)
        if self.scenario=='rotated':
            self.entity('/world/forklift/set_pose','ignition.msgs.Pose','name: "p6_pallet" position: {x: 0.95 y: 0.65 z: 0.0} orientation: {z: 0.039989334 w: 0.999200107}')
        if self.scenario=='missing_pallet':
            self.entity('/world/forklift/remove','ignition.msgs.Entity','name: "p6_pallet" type: MODEL')
        if self.scenario=='early_fallback':
            self.entity('/world/forklift/set_pose','ignition.msgs.Pose','name: "p6_pallet" position: {x: 2.4 y: 2.4 z: 0.0} orientation: {w: 1.0}')
        self.spin(.5)
        # Transport through the same A/B topics used by the RViz tools.
        for key,value in [('pick',a),('drop',b)]:
            self.publishers[key].publish(value);self.spin(.3)
        self.mark('both_rviz_inputs_received',set(json.loads(self.data['input'].data)['poses'])=={'pick','drop'})
        self.mark('start_service_accepted',self.call(Trigger,'/mission/start',Trigger.Request()).success)
        start=len(self.rows);end=time.monotonic()+440.;injected=False
        automatic_faults=('nominal','rotated','user_drop','early_direct','early_fallback','moved_pallet','missing_pallet','invalid_pick','invalid_drop')
        if self.scenario not in automatic_faults:
            target={'cancel':'DETECT_PICK','cancel_initial':'DETECT_PICK_INITIAL','scan_loss_initial':'DETECT_PICK_INITIAL',
                    'cancel_loaded':'NAV_DROP','cancel_retreat':'RETREAT_DROP','scan_loss':'ALIGN'}[self.scenario]
        self.spin(.3)
        self.mark('concurrent_start_rejected',not self.call(Trigger,'/mission/start',Trigger.Request()).success)
        self.mark('clear_while_active_rejected',not self.call(Trigger,'/mission/clear',Trigger.Request()).success)
        self.publishers['pick'].publish(pose(4.,4.));self.spin(.2)
        while time.monotonic()<end:
            self.spin(.1);m=json.loads(self.data['mission'].data)
            if self.scenario=='early_fallback' and not injected and (m.get('initial_detection') or {}).get('outcome')=='UNAVAILABLE':
                self.entity('/world/forklift/set_pose','ignition.msgs.Pose','name: "p6_pallet" position: {x: 0.95 y: 0.65 z: 0.0} orientation: {w: 1.0}')
                injected=True
            if self.scenario=='moved_pallet' and not injected and m['state']=='NAV_PICK':
                self.entity('/world/forklift/set_pose','ignition.msgs.Pose','name: "p6_pallet" position: {x: 0.95 y: 0.55 z: 0.0} orientation: {w: 1.0}')
                injected=True
            if self.scenario not in automatic_faults and not injected and m['state']==target:
                if self.scenario in ('scan_loss','scan_loss_initial'):self.call(SetBool,'/scan_relay/drop_scan',SetBool.Request(data=True))
                else:self.call(Trigger,'/mission/stop',Trigger.Request())
                injected=True
            if m['state'] in ('DONE','FAULT','CANCELLED') and not m['active']:break
        else:raise TimeoutError('MISSION_TIMEOUT')
        self.spin(1.);m=json.loads(self.data['mission'].data);rows=self.rows[start:]
        (self.out/'final_status.json').write_text(json.dumps(m,indent=2)+'\n')
        cmd=self.data.get('cmd',Twist())
        self.mark('stopped_at_end',max(abs(cmd.linear.x),abs(cmd.linear.y),abs(cmd.angular.z))<1e-6)
        self.mark('single_velocity_owner',self.n.count_publishers('/cmd_vel')==1)
        self.mark('no_return_home','RETURN_HOME' not in self.events)
        self.mark('no_navigate_to_pose_recovery',self.navigation_goals==0)
        self.mark('initial_detection_attempted','DETECT_PICK_INITIAL' in self.events)
        if self.scenario in ('cancel_initial','scan_loss_initial'):
            expected='CANCELED' if self.scenario=='cancel_initial' else 'FEEDBACK_OR_NAVIGATION_FAULT'
            self.mark('expected_initial_fault',m['code']==expected,code=m['code'])
            self.mark('initial_fault_injected',injected)
            self.mark('initial_fault_not_fallback',m.get('initial_detection') is None)
            self.mark('no_base_motion_before_initial_fault',not any(max(abs(r[k]) for k in ('vx','vy','wz'))>.001 for r in rows))
            self.mark('no_path_or_navigation_before_initial_fault',not self.routes and not self.active_paths and 'NAV_PICK' not in self.events)
            self.mark('fault_latches_new_mission',not self.call(Trigger,'/mission/start',Trigger.Request()).success)
            return
        if self.scenario in ('invalid_pick','invalid_drop'):
            expected='NO_VALID_PATH_NAV_PICK' if self.scenario=='invalid_pick' else 'NO_VALID_PATH_NAV_DROP'
            self.mark('preflight_rejected_unreachable_goal',m['code']==expected,code=m['code'])
            self.mark('no_base_motion_before_failure',not any(max(abs(r[k]) for k in ('vx','vy','wz'))>.001 for r in rows))
            self.mark('navigation_not_started','NAV_PICK' not in self.events and not self.active_paths)
            return
        self.mark('whole_route_before_motion',bool(self.routes) and self.routes[0]['planned_before_motion'] and self.routes[0]['received_before_motion'])
        if self.routes:
            initial=self.routes[0];navs=[s for s in initial['segments'] if s['kind']=='navigation']
            self.mark('legacy_candidate_selection',initial['route_source']=='dynamic_planned' and len(navs)==2 and
                all(s['planner'] in ('Pallet','Pallet11','Pallet13','Pallet15') and s['candidates_evaluated']>0 for s in navs))
        self.mark('active_path_published',bool(self.active_paths))
        if self.scenario not in ('nominal','rotated','user_drop','early_direct','early_fallback','moved_pallet'):
            expected='PALLET_DETECTION_NOT_STABLE' if self.scenario=='missing_pallet' else 'FEEDBACK_OR_NAVIGATION_FAULT' if self.scenario=='scan_loss' else 'CANCELED'
            self.mark('expected_fault',m['code']==expected,code=m['code'])
            self.mark('fault_injected',injected or self.scenario=='missing_pallet')
            self.mark('fault_latches_new_mission',not self.call(Trigger,'/mission/start',Trigger.Request()).success)
            return
        self.mark('mission_succeeded',m['state']=='DONE' and m['code']=='SUCCEEDED',code=m['code'])
        self.mark('lidar_refinement_executed',m['refinement'] is not None and m['refinement']['samples']>=5)
        f=m['refinement']
        early=m.get('initial_detection') or {}
        expected_early='UNAVAILABLE' if self.scenario=='early_fallback' else 'DETECTED'
        self.mark('early_detection_outcome',early.get('outcome')==expected_early,observation=early)
        observations=[v for v in (early,f) if v and 'corrected' in v]
        self.mark('coarse_pose_corrected',any(math.hypot(v['corrected'][0]-v['initial'][0],v['corrected'][1]-v['initial'][1])>.025 for v in observations),refinement=f)
        if early.get('outcome')=='DETECTED':
            first_pick=next(s for s in self.routes[0]['segments'] if s['stage']=='NAV_PICK')['poses'][-1]
            self.mark('initial_path_targets_corrected_A',math.hypot(first_pick[0]-early['corrected'][0],first_pick[1]-early['corrected'][1])<.005 and abs(angle(first_pick[2]-early['corrected'][2]))<.01)
        self.mark('near_verification_before_insert','DETECT_PICK' in self.events and self.events.index('NAV_PICK')<self.events.index('DETECT_PICK') and ('INSERT' not in self.events or self.events.index('DETECT_PICK')<self.events.index('INSERT')))
        self.mark('refined_navigation_matches_observation',bool(f) and bool(f['replan_required'])==('REFINE_PICK' in self.events))
        if early.get('outcome')=='DETECTED' and f and not f['replan_required']:
            self.mark('docking_join_footprint_validated',f.get('alignment_path_validated') is True)
        if self.scenario=='early_direct':self.mark('no_extra_pick_navigation','REFINE_PICK' not in self.events)
        if self.scenario in ('early_fallback','moved_pallet'):
            self.mark('scene_change_injected',injected)
            self.mark('late_target_replan_executed','REFINE_PICK' in self.events)
        lift=next((r for r in rows if r['state']=='LIFT_PICK'),None)
        if lift:
            a=lift['pallet_yaw'];dx=lift['robot_x']-lift['pallet_x'];dy=lift['robot_y']-lift['pallet_y']
            lateral=-math.sin(a)*dx+math.cos(a)*dy;heading=angle(lift['robot_yaw']-a)
            self.mark('physical_pick_alignment',abs(lateral)<.014 and abs(heading)<.055,lateral=lateral,yaw=heading)
        else:self.mark('physical_pick_alignment',False)
        carry=[r for r in rows if r['state'] in ('RETREAT_PICK','NAV_DROP')]
        self.mark('physically_carried',bool(carry) and min(r['pallet_z'] for r in carry)>.004)
        insert=[r for r in rows if r['state']=='INSERT']
        self.mark('straight_insertion',bool(insert) and all(abs(r['vy'])<1e-6 and abs(r['wz'])<1e-6 for r in insert))
        p=self.poses['p6_pallet'].translation
        # Empty B has no local reference: certify the Nav2 map-frame goal and
        # separately measure global/physical placement in a 15 cm delivery region.
        release=next((r for r in rows if r['state']=='LOWER_RELEASE'),None)
        map_error=math.hypot(release['map_x']-requested_drop[0],release['map_y']-requested_drop[1]) if release and release['map_x'] is not None else float('inf')
        self.mark('release_at_requested_map_pose',map_error<.04 and abs(angle(release['map_yaw']-requested_drop[2]))<.06,map_error_m=map_error)
        expected_pallet=[requested_drop[0]+.23*math.cos(requested_drop[2]),requested_drop[1]+.23*math.sin(requested_drop[2])]
        physical_error=math.hypot(p.x-expected_pallet[0],p.y-expected_pallet[1])
        self.mark('pallet_in_delivery_region',physical_error<.15 and abs(p.z)<.005,x=p.x,y=p.y,z=p.z,error_m=physical_error,region_radius_m=.15)
        retreat=[r for r in rows if r['state']=='RETREAT_DROP']
        distance=math.hypot(retreat[-1]['robot_x']-retreat[0]['robot_x'],retreat[-1]['robot_y']-retreat[0]['robot_y']) if retreat else 0.
        self.mark('withdrawal_clearance',distance>=.32,distance=distance)
        self.mark('released',not m['load_detected'] and not m['carrying'])
        expected=['DETECT_PICK_INITIAL','PLAN_MISSION','NAV_PICK','DETECT_PICK']
        if f and f['replan_required']:expected.append('REFINE_PICK')
        expected+=['ORIENT_PICK','ALIGN','INSERT','LIFT_PICK','VERIFY_LOAD','RETREAT_PICK','NAV_DROP','LOWER_RELEASE','RETREAT_DROP','DONE']
        i=0
        for e in self.events:
            if i<len(expected) and e==expected[i]:i+=1
        self.mark('phase_order',i==len(expected),events=self.events)

    def finish(self):
        (self.out/'route_versions.json').write_text(json.dumps(self.routes,indent=2)+'\n')
        if self.rows:
            with (self.out/'mission.csv').open('w') as f:
                w=csv.DictWriter(f,fieldnames=self.rows[0]);w.writeheader();w.writerows(self.rows)
        (self.out/'checks.json').write_text(json.dumps(self.checks,indent=2)+'\n')
        (self.out/'summary.json').write_text(json.dumps(dict(passed=bool(self.checks) and all(c['passed'] for c in self.checks),checks_passed=sum(c['passed'] for c in self.checks),checks_total=len(self.checks),scenario=self.scenario,physical_hardware_used=False),indent=2)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--scenario',default='nominal');a=p.parse_args()
    rclpy.init();t=Check(a.output,a.scenario)
    try:t.run()
    except Exception as e:t.mark('exception',False,error=str(e));raise
    finally:
        t.finish();t.n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if all(c['passed'] for c in t.checks) else 1


if __name__=='__main__':raise SystemExit(main())
