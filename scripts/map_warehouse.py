#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Drive a deterministic safe aisle survey; SLAM alone produces the map."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String


def main():
    p=argparse.ArgumentParser();p.add_argument('--map',default='src/forklift_navigation/maps/warehouse');args=p.parse_args()
    rclpy.init();n=rclpy.create_node('warehouse_survey');latest={};results=[]
    n.create_subscription(Odometry,'/odom',lambda m:latest.update(odom=m),10)
    n.create_subscription(OccupancyGrid,'/map',lambda m:latest.update(map=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub=n.create_publisher(Twist,'/cmd_vel_teleop',10)
    def spin(seconds,cmd=None):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if cmd:
                m=Twist();m.linear.x,m.linear.y,m.angular.z=map(float,cmd);pub.publish(m)
            rclpy.spin_once(n,timeout_sec=.03)
    try:
        end=time.monotonic()+40
        while 'odom' not in latest or 'map' not in latest:
            spin(.1);assert time.monotonic()<end,'Mapping not ready'
        spin(1.)
        for x,y in [(.9,0),(.9,1.),(-.9,1.),(-.9,-1.),(.9,-1.),(0,0)]:
            deadline=time.monotonic()+35
            while time.monotonic()<deadline:
                pose=latest['odom'].pose.pose;q=pose.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                dx,dy=x-pose.position.x,y-pose.position.y
                if math.hypot(dx,dy)<.035:break
                vx=max(-.18,min(.18,.9*(math.cos(yaw)*dx+math.sin(yaw)*dy)))
                vy=max(-.16,min(.16,.9*(-math.sin(yaw)*dx+math.cos(yaw)*dy)))
                spin(.04,(vx,vy,max(-.3,min(.3,-yaw))))
            else:raise RuntimeError(f'Survey waypoint timeout {x,y}')
            spin(.6,(0,0,0));results.append({'waypoint':[x,y],'reached':True});print(results[-1],flush=True)
        # Rotate once to add viewpoints and exercise scan matching with yaw changes.
        spin(2*math.pi/.5,(0,0,.5));spin(3.,(0,0,0))
        m=latest['map'];known=sum(v>=0 for v in m.data)
        assert known>4000 and any(v==100 for v in m.data),'Map has insufficient observations'
        client=n.create_client(SaveMap,'/slam_toolbox/save_map');assert client.wait_for_service(timeout_sec=5)
        prefix=str(Path(args.map).resolve());Path(prefix).parent.mkdir(parents=True,exist_ok=True)
        req=SaveMap.Request();req.name=String(data=prefix)
        future=client.call_async(req);rclpy.spin_until_future_complete(n,future,timeout_sec=15)
        assert future.done() and future.result().result==0,'SLAM map save failed'
        data={'survey':results,'width':m.info.width,'height':m.info.height,'resolution':m.info.resolution,'known_cells':known,'map_prefix':prefix}
        Path('results/p4/mapping.json').write_text(json.dumps(data,indent=2)+'\n');print(data,flush=True)
    finally:
        pub.publish(Twist());n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
