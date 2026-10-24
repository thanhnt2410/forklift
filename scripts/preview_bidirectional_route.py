#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Plan on a static Nav2 costmap without Gazebo, controllers, or robot motion.
This is a proposed route for review, not a physics acceptance run.
"""
import argparse,copy,json,math,os,signal,subprocess,time
from pathlib import Path
import yaml
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from forklift_task_manager.mission_route import straight_samples

p=argparse.ArgumentParser();p.add_argument('--output',default='results/bidirectional_preview');p.add_argument('--domain',type=int,default=199)
p.add_argument('--clearance',type=float,default=.35);a=p.parse_args()
root=Path(__file__).resolve().parents[1];out=(root/a.output).resolve();out.mkdir(parents=True,exist_ok=True)
if not .3<=a.clearance<=1.8:raise ValueError('Withdrawal must clear the fork/deck before turning')
env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(out/'ros_logs'))
os.environ.update(ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1')
config=yaml.safe_load((root/'src/forklift_navigation/config/nav2.yaml').read_text())
g=yaml.safe_load((root/'src/forklift_description/config/robot.yaml').read_text())
mission=yaml.safe_load((root/'src/forklift_task_manager/config/stations.yaml').read_text())['mission']
params=copy.deepcopy(config['planner_server']['ros__parameters']);params['use_sim_time']=False
pallet=copy.deepcopy(params.pop('GridBased'));pallet.update(reverse_penalty=1.0,change_penalty=.15,non_straight_penalty=1.5)
transit=copy.deepcopy(pallet);transit['plugin']='nav2_smac_planner/SmacPlannerHybrid';transit.pop('terminal_distance',None)
profiles={'Pallet':.9,'Pallet11':1.1,'Pallet13':1.3,'Pallet15':1.5}
params.update(planner_plugins=[*profiles,'Transit'],Transit=transit)
for name,length in profiles.items():params[name]={**copy.deepcopy(pallet),'terminal_distance':length}
map_params=copy.deepcopy(config['global_costmap']['global_costmap']['ros__parameters']);map_params.update(use_sim_time=False,plugins=['static_layer','inflation_layer'])
map_params['footprint']=str([[g['footprint_xmin'],-g['footprint_halfwidth']],[g['loaded_xmax'],-g['footprint_halfwidth']],[g['loaded_xmax'],g['footprint_halfwidth']],[g['footprint_xmin'],g['footprint_halfwidth']]])
static_config={'planner_server':{'ros__parameters':params},'global_costmap':{'global_costmap':{'ros__parameters':map_params}},'map_server':{'ros__parameters':dict(use_sim_time=False,yaml_filename=str(root/'src/forklift_navigation/maps/warehouse.yaml'))}}
config_path=out/'static_planner.yaml';config_path.write_text(yaml.safe_dump(static_config))
processes=[];logs=[]
def start(args,name):
    f=(out/name).open('w');logs.append(f);proc=subprocess.Popen(args,env=env,cwd=root,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc)

def pose(values,node):
    m=PoseStamped();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg();m.pose.position.x=float(values[0]);m.pose.position.y=float(values[1]);m.pose.orientation.z=math.sin(values[2]/2);m.pose.orientation.w=math.cos(values[2]/2);return m

def statistics(points):
    distance=reverse=turn=0.;directions=[]
    for u,v in zip(points,points[1:]):
        dx,dy=v[0]-u[0],v[1]-u[1];d=math.hypot(dx,dy);distance+=d
        turn+=abs(math.atan2(math.sin(v[2]-u[2]),math.cos(v[2]-u[2])))
        if d<1e-5:continue
        s=1 if dx*math.cos(u[2])+dy*math.sin(u[2])>0 else -1
        if s<0:reverse+=d
        if not directions or directions[-1]!=s:directions.append(s)
    return dict(length_m=distance,reverse_m=reverse,gear_changes=max(0,len(directions)-1),turn_radians=turn)

try:
    start(['ros2','run','tf2_ros','static_transform_publisher','0','0','0','0','0','0','map','base_link'],'tf.log')
    start(['ros2','run','nav2_map_server','map_server','--ros-args','--params-file',str(config_path)],'map.log')
    start(['ros2','run','nav2_planner','planner_server','--ros-args','--params-file',str(config_path)],'planner.log')
    start(['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-p','autostart:=true','-p','node_names:=[map_server,planner_server]'],'lifecycle.log')
    rclpy.init();node=rclpy.create_node('static_route_preview');client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    if not client.wait_for_server(timeout_sec=35.):raise RuntimeError('Static planner unavailable')
    # Action discovery precedes costmap lifecycle activation.
    time.sleep(3.)
    def wait(future,timeout=15.):
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done():raise TimeoutError('Static planner action timeout')
        return future.result()
    segments=[]
    def request_path(name,begin,end,planner):
        goal=ComputePathToPose.Goal(start=pose(begin,node),goal=pose(end,node),planner_id=planner,use_start=True)
        handle=wait(client.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('Static path rejected: '+name)
        result=wait(handle.get_result_async())
        if result.status!=4 or not result.result.path.poses:raise RuntimeError('No static route: '+name)
        points=[[p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in result.result.path.poses]
        return dict(stage=name,kind='navigation',planner=planner,collision_checked=True,poses=points,statistics=statistics(points),goal=end,terminal_distance=profiles.get(planner,0.))
    def choose(candidates):
        shortest=min(v['statistics']['length_m'] for v in candidates)
        allowed=[v for v in candidates if v['statistics']['length_m']<=shortest*1.5]
        def score(v):
            z=v['statistics'];return z['length_m']+.6*z['gear_changes']+.12*z['turn_radians']
        best=min(allowed,key=score);best['selection_score']=score(best);return best
    def navigation(name,begin,end,planner):
        candidates=[]
        goals=[end] if planner=='Pallet' else [[end[0],end[1],-math.pi+i*math.pi/12] for i in range(24)]
        for candidate_goal in goals:
            for planner_name in (profiles if planner=='Pallet' else ['Transit']):
                try:candidates.append(request_path(name,begin,candidate_goal,planner_name))
                except RuntimeError:continue
        if not candidates:raise RuntimeError('No candidate route: '+name)
        best=choose(candidates);best['candidates_evaluated']=len(candidates);segments.append(best)
    def line(name,begin,end):
        points=straight_samples(begin,end);segments.append(dict(stage=name,kind='lidar_docking',collision_checked=False,poses=points,statistics=statistics(points)))
    def station(s):
        angle=s['approach'][2];c,sn=math.cos(angle),math.sin(angle)
        def at(d):return [s['face'][0]-d*c,s['face'][1]-d*sn,angle]
        pre,insert=at(mission['preinsert_distance']),at(mission['inserted_distance'])
        return pre,insert,[insert[0]-a.clearance*c,insert[1]-a.clearance*sn,angle]
    ap,ai,ar=station(mission['a']);bp,bi,br=station(mission['b'])
    navigation('NAV_PICK',[0.,0.,0.],mission['a']['approach'],'Pallet')
    line('ALIGN_PICK',mission['a']['approach'],ap);line('INSERT_PICK',ap,ai);line('RETREAT_PICK',ai,ar)
    navigation('NAV_DROP',ar,mission['b']['approach'],'Pallet')
    line('ALIGN_DROP',mission['b']['approach'],bp);line('INSERT_DROP',bp,bi);line('RETREAT_DROP',bi,br)
    navigation('RETURN_HOME',br,mission['home'],'Transit')
    report=dict(preview_only=True,gazebo_started=False,robot_motion_enabled=False,loaded_footprint=True,map_source='warehouse.yaml, static layers only',withdrawal_clearance=a.clearance,terminal_distance=mission['terminal_distance'],reverse_penalty=1.0,change_penalty=.15,non_straight_penalty=1.5,lift_at=ai,release_at=bi,home=segments[-1]['goal'],home_heading_policy='free for preview; pallet headings remain constrained',selection='Length + 0.6 m per gear change + 0.12 m per radian turning; candidates capped at 1.5x shortest',segments=segments)
    (out/'proposed_route.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({s['stage']:s['statistics'] for s in segments},indent=2))
finally:
    # Only these offline planning processes are owned by this script.
    for proc in reversed(processes):
        try:os.killpg(proc.pid,signal.SIGINT)
        except ProcessLookupError:pass
    for proc in processes:
        try:proc.wait(timeout=3.)
        except subprocess.TimeoutExpired:
            try:os.killpg(proc.pid,signal.SIGTERM)
            except ProcessLookupError:pass
    for log in logs:log.close()
    if rclpy.ok():rclpy.shutdown()
