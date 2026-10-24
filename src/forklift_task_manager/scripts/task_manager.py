#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""P6 SIM: scan docking, actual deck contacts and a bounded action sequence.
No Gazebo world pose subscription, teleport or attach/detach in the controller.
"""
import json
import os
import math
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, PoseStamped, Point, Polygon, Point32
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from nav2_msgs.action import NavigateToPose, Spin, ComputePathToPose, FollowPath
from nav2_msgs.srv import IsPathValid
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.srv import CancelGoal
from ros_gz_interfaces.msg import Contacts
from tf2_ros import Buffer, TransformListener
from forklift_interfaces.action import RunMission, MoveLift, HomeLift
from forklift_interfaces.msg import LiftState
from forklift_task_manager.docking import detect_face, clamp, wrap, can_advance
from forklift_task_manager.mission_route import station_poses, straight_samples, load_approved_route


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class MissionError(Exception):pass

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.declare_parameter('stations',str(Path(get_package_share_directory('forklift_task_manager'))/'config/stations.yaml'))
        self.p=yaml.safe_load(Path(self.get_parameter('stations').value).read_text())['mission']
        self.g=yaml.safe_load((Path(get_package_share_directory('forklift_description'))/'config/robot.yaml').read_text())
        self.p['laser_x']=self.g['laser_xyz'][0]
        self.declare_parameter('approved_route_file',str(Path(get_package_share_directory('forklift_task_manager'))/'config/approved_route.json'))
        filename=self.get_parameter('approved_route_file').value
        self.approved_file=Path(filename) if filename else None
        self.approved_sha256=''
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.data={};self.stamps={};self.sleepers=[];self.active=False;self.failed=False
        self.state='IDLE';self.code='';self.mission_goal=None;self.client_goal=None
        self.stop_requested=False;self.load_required=False;self.contact_time=0.;self.face=None
        self.contact_debug=[];self.contact_messages=0;self.started=0.;self.deadline=0.;self.events=[];self.stable=None
        self.velocity=self.create_publisher(Twist,'/mission/cmd_vel',10)
        self.status=self.create_publisher(String,'/mission/status',10)
        self.loaded=self.create_publisher(Bool,'/mission/loaded',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for cls,topic,key,qos in [(LaserScan,'/scan_raw','scan',qos_profile_sensor_data),
                (Odometry,'/odom','odom',10),(LiftState,'/lift/state','lift',latched),
                (Bool,'/navigation/ready','ready',10),(Bool,'/safety/estop','estop',latched)]:
            self.create_subscription(cls,topic,lambda m,k=key:self.receive(k,m),qos)
        self.create_subscription(Contacts,'/p6/contacts',self.contacts,10)
        self.nav_cancel=self.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
        self.route_segments=[];self.route_revision=0
        self.route_path=self.create_publisher(NavPath,'/mission/route',latched)
        self.route_markers=self.create_publisher(MarkerArray,'/mission/route_markers',latched)
        self.route_info=self.create_publisher(String,'/mission/route_info',latched)
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.active_path=self.create_publisher(NavPath,'/mission/active_path',latched)
        self.footprints=[self.create_publisher(Polygon,'/'+name+'/footprint',10)
                         for name in ('local_costmap','global_costmap')]
        self.follow=ActionClient(self,FollowPath,'/follow_path')
        self.path_valid=self.create_client(IsPathValid,'/is_path_valid')
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.spin_action=ActionClient(self,Spin,'/spin')
        self.lift=ActionClient(self,MoveLift,'/lift/move')
        self.home=ActionClient(self,HomeLift,'/lift/home')
        self.mux=self.create_client(SetBool,'/mission_motion/select_docking')
        self.lift_stop=self.create_client(Trigger,'/lift/stop')
        self.create_service(Trigger,'/mission/stop',self.stop)
        self.create_service(Trigger,'/mission/reset',self.reset)
        self.server=ActionServer(self,RunMission,'/mission/run',execute_callback=self.execute,
            goal_callback=self.accept,cancel_callback=lambda h:CancelResponse.ACCEPT)
        self.create_timer(.02,self.tick,clock=rclpy.clock.Clock())

    def receive(self,key,msg):self.data[key]=msg;self.stamps[key]=time.monotonic()
    def contacts(self,msg):
        self.receive('contacts',msg)
        self.contact_messages+=1
        if msg.contacts:self.contact_debug=[dict(a=c.collision1.name,b=c.collision2.name,aid=c.collision1.id,bid=c.collision2.id) for c in msg.contacts]
        # Names are supplied by Gazebo contact sensor, not a geometric pose oracle.
        if any('forklift' in c.collision1.name+c.collision2.name and
               ('fork_' in c.collision1.name+c.collision2.name or 'lift_carriage' in c.collision1.name+c.collision2.name)
               for c in msg.contacts):self.contact_time=time.monotonic()
    def has_load(self):return time.monotonic()-self.contact_time<.3
    def fresh(self,key,age=.6):return time.monotonic()-self.stamps.get(key,0.)<age
    def ready(self):
        return (all(self.fresh(k) for k in ['scan','odom','lift','ready']) and self.data['ready'].data
                and not self.data.get('estop',Bool()).data and not self.data['lift'].fault_code)
    def accept(self,g):
        if self.active or self.failed or g.mission not in ('','a_to_b') or not math.isfinite(g.timeout) or not 0<=g.timeout<=900 or not self.ready():return GoalResponse.REJECT
        self.active=True;self.stop_requested=False;return GoalResponse.ACCEPT
    def stop(self,req,res):
        self.stop_requested=True;self.command();res.success=True;res.message='Stopping mission; load position held';return res
    def reset(self,req,res):
        low='lift' in self.data and self.data['lift'].height+self.data['lift'].home_offset<.02
        res.success=not self.active and self.ready() and low and not self.has_load()
        if res.success:self.failed=False;self.code='';self.state='IDLE';self.stop_requested=False
        if res.success:self.load_required=False;self.loaded.publish(Bool(data=False))
        res.message='Reset; send a new mission' if res.success else 'Not ready, active or carrying load; recover explicitly';return res
    def check(self):
        if self.stop_requested or (self.mission_goal and self.mission_goal.is_cancel_requested):raise MissionError('CANCELED')
        if time.monotonic()>self.deadline:raise MissionError('MISSION_TIMEOUT')
        if not self.ready():raise MissionError('FEEDBACK_OR_NAVIGATION_FAULT')
        if self.load_required and not self.has_load():raise MissionError('LOAD_CONTACT_LOST')
        if self.state not in ('CHECK_READY','HOME_LIFT','LOWER_FOR_PICK','LIFT_PICK','LOWER_RELEASE'):
            lift=self.data['lift']
            if lift.moving or lift.inhibited or not lift.homed or lift.height+lift.home_offset>.02:
                raise MissionError('LIFT_INTERLOCK_CHANGED')
    async def pause(self,seconds=.05,check=True):
        f=Future();self.sleepers.append((time.monotonic()+seconds,f));await f
        if check:self.check()
    async def wait(self,future,timeout):
        end=time.monotonic()+timeout
        while not future.done():
            await self.pause()
            if time.monotonic()>end:raise MissionError('ACTION_OR_SERVICE_TIMEOUT')
        if future.exception():raise MissionError(str(future.exception()))
        return future.result()
    def stage(self,state):
        self.state=state;self.events.append(dict(state=state,wall_time=time.monotonic()))
        self.get_logger().info('P6 '+state)
    def command(self,vx=0.,vy=0.,wz=0.):
        m=Twist();m.linear.x=float(vx);m.linear.y=float(vy);m.angular.z=float(wz);self.velocity.publish(m)
    async def select(self,docking):
        self.command();r=await self.wait(self.mux.call_async(SetBool.Request(data=docking)),3.)
        if not r.success:raise MissionError('SOURCE_SWITCH_FAILED')
        await self.pause(.3)
    async def action(self,client,goal,timeout):
        if not client.server_is_ready():raise MissionError('ACTION_SERVER_UNAVAILABLE')
        pending=client.send_goal_async(goal)
        # Retain handle even if cancel arrives while goal acknowledgement is pending.
        pending.add_done_callback(lambda f:setattr(self,'client_goal',f.result()) if f.result() and f.result().accepted else None)
        h=await self.wait(pending,5.)
        if not h.accepted:raise MissionError('ACTION_REJECTED')
        self.client_goal=h
        result=await self.wait(h.get_result_async(),timeout)
        self.client_goal=None
        if result.status!=4 or (hasattr(result.result,'success') and not result.result.success):
            raise MissionError('ACTION_FAILED_'+str(getattr(result.result,'code',result.status)))
        return result.result
    async def nav_to(self,pose,state):
        self.stage(state)
        # Stop/preempt any external RViz goal while the docking source is held at
        # zero. Do not enable Nav2 with an old goal during a source transition.
        await self.select(True)
        await self.wait(self.nav_cancel.call_async(CancelGoal.Request()),5.)
        await self.pause(.2)
        if self.approved_file:
            await self.follow_approved_route(state)
            return
        await self.select(False)
        g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.header.stamp=self.get_clock().now().to_msg()
        g.pose.pose.position.x=float(pose[0]);g.pose.pose.position.y=float(pose[1]);g.pose.pose.orientation.z=math.sin(pose[2]/2);g.pose.pose.orientation.w=math.cos(pose[2]/2)
        await self.action(self.nav,g,120.);await self.select(True);await self.stationary()
    async def follow_approved_route(self,state):
        # The carried pallet extends beyond the forks. After release it is an
        # obstacle, not part of the robot: restore the CAD unloaded footprint.
        xmax=self.g['footprint_xmax'];halfwidth=self.g['footprint_halfwidth']
        if self.load_required:
            xmax=max(xmax,self.g['loaded_xmax'])
            halfwidth=max(halfwidth,self.g['loaded_halfwidth'])
        polygon=Polygon(points=[Point32(x=float(x),y=float(y)) for x,y in
            [(self.g['footprint_xmin'],-halfwidth),(xmax,-halfwidth),
             (xmax,halfwidth),(self.g['footprint_xmin'],halfwidth)]])
        for publisher in self.footprints:publisher.publish(polygon)
        await self.pause(.3)
        segment=next((s for s in self.route_segments if s['stage']==state),None)
        if segment is None:raise MissionError('APPROVED_SEGMENT_MISSING')
        path=NavPath();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        path.poses=[self.map_pose(p) for p in segment['poses']]
        transform=self.tf.lookup_transform('map','base_link',rclpy.time.Time()).transform
        closest=0;best=float('inf');travelled=0.
        for i,p in enumerate(path.poses):
            if i:travelled+=math.hypot(p.pose.position.x-path.poses[i-1].pose.position.x,p.pose.position.y-path.poses[i-1].pose.position.y)
            if travelled>.25:break
            distance=math.hypot(p.pose.position.x-transform.translation.x,p.pose.position.y-transform.translation.y)
            if distance<best:best=distance;closest=i
        if best>.25:raise MissionError('APPROVED_ROUTE_START_MISMATCH')
        # Preserve original vertices/tangents: never invent a cusp by anchoring
        # the first vertex to a slightly off-path localization estimate.
        path.poses=path.poses[closest:]
        if not self.path_valid.service_is_ready():raise MissionError('PATH_VALIDATION_UNAVAILABLE')
        valid=await self.wait(self.path_valid.call_async(IsPathValid.Request(path=path)),5.)
        if not valid.is_valid:raise MissionError('APPROVED_ROUTE_BLOCKED')
        self.active_path.publish(path)
        self.get_logger().info('Following approved '+state+' path; sha256='+self.approved_sha256)
        await self.select(False)
        goal=FollowPath.Goal(path=path,controller_id='FollowPath',goal_checker_id='general_goal_checker')
        await self.action(self.follow,goal,120.)
        await self.select(True);await self.stationary()
    def map_pose(self, values):
        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=self.get_clock().now().to_msg()
        pose.pose.position.x=float(values[0]);pose.pose.position.y=float(values[1])
        pose.pose.orientation.z=math.sin(values[2]/2);pose.pose.orientation.w=math.cos(values[2]/2)
        return pose
    async def plan_mission(self):
        self.stage('PLAN_MISSION');await self.select(True);await self.stationary()
        await self.wait(self.nav_cancel.call_async(CancelGoal.Request()),5.)
        await self.pause(.2)
        transform=self.tf.lookup_transform('map','base_link',rclpy.time.Time()).transform
        start=[transform.translation.x,transform.translation.y,yaw(transform.rotation)]
        ap,ai,ar=station_poses(self.p,self.p['a']);bp,bi,br=station_poses(self.p,self.p['b'])
        if self.approved_file:
            approved=load_approved_route(self.approved_file,self.p)
            self.approved_sha256=approved['sha256']
            self.publish_route(approved['segments'],start,approved['lift_at'],approved['release_at'])
            return
        segments=[]
        async def navigation(name,begin,end):
            goal=ComputePathToPose.Goal(start=self.map_pose(begin),goal=self.map_pose(end),use_start=True,planner_id='GridBased')
            result=await self.action(self.planner,goal,12.)
            if len(result.path.poses)<2:raise MissionError('MISSION_ROUTE_EMPTY_'+name)
            segments.append(dict(stage=name,kind='navigation',collision_checked=True,
                poses=[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in result.path.poses]))
        def line(name,begin,end):
            # Intentional contact/docking legs use the existing LiDAR/sweep
            # interlocks at execution. They are not claimed as Nav2 free space.
            segments.append(dict(stage=name,kind='lidar_docking',collision_checked=False,poses=straight_samples(begin,end)))
        await navigation('NAV_PICK',start,self.p['a']['approach'])
        line('ALIGN_PICK',self.p['a']['approach'],ap);line('INSERT_PICK',ap,ai)
        line('RETREAT_PICK',ai,ar)
        await navigation('NAV_DROP',ar,self.p['b']['approach'])
        line('ALIGN_DROP',self.p['b']['approach'],bp);line('INSERT_DROP',bp,bi)
        line('RETREAT_DROP',bi,br)
        await navigation('RETURN_HOME',br,self.p['home'])
        self.publish_route(segments,start,ai,bi)
    def publish_route(self,segments,start,ai,bi):
        self.route_segments=segments;self.route_revision+=1
        combined=NavPath();combined.header=self.map_pose(start).header
        markers=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
        palette={'NAV_PICK':(.2,.65,1.),'NAV_DROP':(.2,1.,.3),'RETURN_HOME':(.8,.3,1.)}
        for index,segment in enumerate(segments):
            poses=segment['poses'];combined.poses.extend(self.map_pose(p) for p in poses)
            color=palette.get(segment['stage'],(1.,.5,.05) if 'RETREAT' in segment['stage'] else (.15,.9,.9))
            marker=Marker();marker.header=combined.header;marker.ns='mission_route';marker.id=index
            marker.type=Marker.LINE_STRIP;marker.action=Marker.ADD;marker.pose.orientation.w=1.;marker.scale.x=.025
            marker.color.r,marker.color.g,marker.color.b=color;marker.color.a=.85
            marker.points=[Point(x=float(p[0]),y=float(p[1]),z=.035) for p in poses];markers.markers.append(marker)
            label=Marker();label.header=combined.header;label.ns='mission_labels';label.id=index
            label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.orientation.w=1.
            label.pose.position=Point(x=float(poses[len(poses)//2][0]),y=float(poses[len(poses)//2][1]),z=.18+.04*(index%3))
            label.scale.z=.075;label.color.r=label.color.g=label.color.b=label.color.a=1.;label.text=f'{index+1}. '+segment['stage'];markers.markers.append(label)
        self.route_path.publish(combined);self.route_markers.publish(markers)
        self.route_info.publish(String(data=json.dumps(dict(revision=self.route_revision,planned_before_motion=True,
            route_source='approved_preview' if self.approved_file else 'live_planner',approved_sha256=self.approved_sha256,withdrawal_clearance=self.p['retreat_distance'],home=self.p['home'],docking_uses_live_lidar=True,terminal_distance=self.p.get('terminal_distance',.5),retreat_minimum=self.p['retreat_distance'],turn_setup_distance=self.p.get('turn_setup_distance',0.),lift_at=ai,release_at=bi,segments=segments))))
        self.get_logger().info('Full mission route planned before NAV_PICK: '+str(len(segments))+' segments')
    async def stationary(self):
        self.command();end=time.monotonic()+4.;stable=None
        while True:
            t=self.data['odom'].twist.twist
            if max(abs(t.linear.x),abs(t.linear.y),abs(t.angular.z))<.008:
                stable=stable or time.monotonic()
                if time.monotonic()-stable>.4:return
            else:stable=None
            if time.monotonic()>end:raise MissionError('BASE_NOT_STATIONARY')
            await self.pause()
    async def move_lift(self,q,state):
        self.stage(state);await self.stationary();offset=self.data['lift'].home_offset
        if not .0<=q-offset<=self.g['lift_travel']:raise MissionError('LIFT_REFERENCE_INVALID')
        await self.action(self.lift,MoveLift.Goal(height=q-offset,timeout=15.),20.)
        await self.pause(.35)
    def observation(self,station):
        try:t=self.tf.lookup_transform('map','base_link',rclpy.time.Time()).transform
        except Exception:return None
        a=yaw(t.rotation);dx=station['face'][0]+station['marker_offset_x']-t.translation.x;dy=station['face'][1]+station['marker_offset_y']-t.translation.y
        expected=(math.cos(a)*dx+math.sin(a)*dy-self.p['laser_x'],-math.sin(a)*dx+math.cos(a)*dy)
        f=detect_face(self.data['scan'],expected,station['width'],wrap(station['marker_yaw']-a))
        if f is not None:
            f['yaw']=wrap(f['yaw']-station['marker_yaw'])
            stamp=(self.data['scan'].header.stamp.sec,self.data['scan'].header.stamp.nanosec)
            previous=getattr(self,'filtered_face',None)
            if stamp!=getattr(self,'fit_stamp',None):
                if previous is not None:
                    for key in ('x','y'):f[key]=.5*f[key]+.5*previous[key]
                    f['yaw']=wrap(previous['yaw']+.5*wrap(f['yaw']-previous['yaw']))
                self.filtered_face=f;self.fit_stamp=stamp
            f=self.filtered_face
        self.face=f
        if f is None:
            self.debug_saved=True
            scan=self.data['scan']
            debug=dict(expected=expected,map_yaw=a,station=station,angle_min=scan.angle_min,
                angle_increment=scan.angle_increment,range_min=scan.range_min,range_max=scan.range_max,ranges=list(scan.ranges))
            try:Path('/tmp/forklift_p6_docking_'+os.environ.get('ROS_DOMAIN_ID','0')+'.json').write_text(json.dumps(debug))
            except OSError:pass
        return f
    def clear_sweep(self,vx):
        # Check the end opposite the intended pallet. Target face is handled by
        # docking geometry; unknown points alongside the insertion corridor stop.
        scan=self.data['scan']
        for i,r in enumerate(scan.ranges):
            if not math.isfinite(r):continue
            a=scan.angle_min+i*scan.angle_increment;x=r*math.cos(a)+self.p['laser_x'];y=r*math.sin(a)
            if vx<0 and self.g['footprint_xmin']-.12<x<self.g['footprint_xmin'] and abs(y)<self.g['footprint_halfwidth']:return False
        return True
    async def orient(self,station,state):
        # LiDAR supplies the desired correction; Nav2 Spin checks the full
        # costmap footprint while executing it. Stop translation at standoff.
        self.stage(state);await self.stationary()
        self.filtered_face=None;self.fit_stamp=None
        for attempt in range(6):
            end=time.monotonic()+1.5;f=None
            while f is None and time.monotonic()<end:
                f=self.observation(station)
                if f is None:await self.pause()
            if f is None:raise MissionError('PARCEL_FACE_NOT_OBSERVABLE')
            if abs(f['yaw'])<.015:return
            # Under-step the measured angle to absorb the smoother/gate stop
            # delay; re-observe LiDAR after each collision-checked rotation.
            await self.select(False)
            goal=Spin.Goal();goal.target_yaw=float(.75*f['yaw']);goal.time_allowance.sec=12
            await self.action(self.spin_action,goal,15.)
            await self.select(True);await self.stationary()
            self.filtered_face=None;self.fit_stamp=None
        raise MissionError('PALLET_ORIENTATION_TIMEOUT')
    async def dock(self,station,insert=False):
        self.stage('INSERT' if insert else 'ALIGN');target=self.p['inserted_distance' if insert else 'preinsert_distance']
        end=time.monotonic()+45;last_good=time.monotonic();stable=None;self.debug_saved=False;self.filtered_face=None;self.fit_stamp=None
        while time.monotonic()<end:
            f=self.observation(station)
            if f is None:
                self.command()
                if time.monotonic()-last_good>1.5:raise MissionError('PARCEL_FACE_NOT_OBSERVABLE')
                await self.pause();continue
            last_good=time.monotonic();ex=f['x']-station['marker_offset_x']*math.cos(f['yaw'])+station['marker_offset_y']*math.sin(f['yaw'])-target
            # Correct the fiducial's lateral offset in its own rotated frame.
            ey=f['y']-station['marker_offset_x']*math.sin(f['yaw'])-station['marker_offset_y']*math.cos(f['yaw'])
            lateral_tolerance=station.get('lateral_tolerance',.006)
            if insert and (abs(ey)>max(.014,2*lateral_tolerance) or abs(f['yaw'])>.055):raise MissionError('INSERT_ALIGNMENT_LOST')
            aligned=can_advance(ex,ey,f['yaw'],lateral_tolerance,insert)
            if abs(ex)<station.get('distance_tolerance',.003) and aligned:
                self.command();stable=stable or time.monotonic()
                if time.monotonic()-stable>.5:return
            else:
                stable=None
                vx=clamp(.8*ex,self.p.get('insertion_speed',.015) if insert else self.p.get('approach_speed',.035)) if aligned else 0.
                if not insert and abs(f['yaw'])>.015:
                    self.command(0.,0.,clamp(f['yaw'],.08))
                else:
                    self.command(vx,0. if insert else clamp(.8*ey,.02),0. if insert else clamp(f['yaw'],.04))
                if not self.clear_sweep(vx):raise MissionError('DOCK_SWEEP_BLOCKED')
            await self.pause()
        raise MissionError('DOCK_TIMEOUT')
    async def retreat(self,state,station):
        self.stage(state);o=self.data['odom'].pose.pose;start=(o.position.x,o.position.y,yaw(o.orientation));end=time.monotonic()+40
        _,_,target=station_poses(self.p,station)
        transform=self.tf.lookup_transform('map','base_link',rclpy.time.Time()).transform
        a=station['approach'][2]
        required=max(self.p['retreat_distance'],(transform.translation.x-target[0])*math.cos(a)+(transform.translation.y-target[1])*math.sin(a))
        while time.monotonic()<end:
            o=self.data['odom'].pose.pose
            distance=-((o.position.x-start[0])*math.cos(start[2])+(o.position.y-start[1])*math.sin(start[2]))
            if distance>=required:self.command();await self.stationary();return
            if not self.clear_sweep(-.04):raise MissionError('RETREAT_BLOCKED')
            self.command(-min(self.p.get('retreat_speed',.045),max(.008,.6*(required-distance))),0.,clamp(wrap(start[2]-yaw(o.orientation)),.04))
            await self.pause()
        raise MissionError('RETREAT_TIMEOUT')
    async def execute(self,handle):
        self.mission_goal=handle;self.events=[];self.code='';self.started=time.monotonic();self.deadline=self.started+(handle.request.timeout or 420.)
        try:
            self.stage('CHECK_READY');self.check();await self.select(True);await self.stationary()
            if not self.data['lift'].homed:
                self.stage('HOME_LIFT');await self.action(self.home,HomeLift.Goal(timeout=60.),65.)
            await self.move_lift(self.data['lift'].home_offset,'LOWER_FOR_PICK')
            await self.plan_mission()
            await self.nav_to(self.p['a']['approach'],'NAV_PICK')
            await self.orient(self.p['a'],'ORIENT_PICK')
            await self.dock(self.p['a']);await self.dock(self.p['a'],True)
            await self.move_lift(self.p['carry_joint_position'],'LIFT_PICK')
            if not self.has_load():raise MissionError('NO_PALLET_CONTACT')
            self.load_required=True;self.loaded.publish(Bool(data=True));self.stage('VERIFY_LOAD');await self.pause(.8)
            await self.retreat('RETREAT_PICK',self.p['a'])
            await self.nav_to(self.p['b']['approach'],'NAV_DROP')
            await self.orient(self.p['b'],'ORIENT_DROP')
            await self.dock(self.p['b']);await self.dock(self.p['b'],True)
            self.load_required=False
            await self.move_lift(self.data['lift'].home_offset,'LOWER_RELEASE')
            end=time.monotonic()+3.
            while self.has_load() and time.monotonic()<end:await self.pause()
            if self.has_load():raise MissionError('LOAD_NOT_RELEASED')
            self.loaded.publish(Bool(data=False));await self.retreat('RETREAT_DROP',self.p['b'])
            await self.nav_to(self.p['home'],'RETURN_HOME')
            self.stage('DONE');self.code='SUCCEEDED';handle.succeed()
        except Exception as e:
            self.code=str(e);self.failed=True;self.stage('CANCELLED' if self.code=='CANCELED' else 'FAULT')
            self.command()
            if self.client_goal is not None and self.client_goal.accepted:
                self.client_goal.cancel_goal_async()
            self.lift_stop.call_async(Trigger.Request())
            # Reserve docking source at zero so a canceled Nav2 goal cannot replay.
            self.mux.call_async(SetBool.Request(data=True))
            if handle.is_cancel_requested:handle.canceled()
            else:handle.abort()
            self.get_logger().error('P6 '+self.code)
        finally:
            self.command();self.active=False;self.mission_goal=None;self.client_goal=None
        return RunMission.Result(success=self.code=='SUCCEEDED',code=self.code,final_state=self.state)
    def tick(self):
        now=time.monotonic()
        for deadline,f in self.sleepers[:]:
            if now>=deadline:
                self.sleepers.remove((deadline,f))
                if not f.done():f.set_result(True)
        if not self.active:self.command()
        msg=dict(route_ready=bool(self.route_segments),route_revision=self.route_revision,state=self.state,code=self.code,active=self.active,load_detected=self.has_load(),
                 carrying=self.load_required,contacts=self.contact_debug,contact_messages=self.contact_messages,face=self.face,elapsed=now-self.started if self.started else 0.,events=self.events)
        self.status.publish(String(data=json.dumps(msg)))
        if self.mission_goal is not None:
            self.mission_goal.publish_feedback(RunMission.Feedback(state=self.state,load_detected=self.has_load(),lift_height=self.data.get('lift',LiftState()).height))

def main():
    rclpy.init();n=TaskManager()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():n.command()
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
