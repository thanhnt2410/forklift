#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Pass actual Gazebo scans through; explicit sensor-blackout test service."""
import math
import copy
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self.declare_parameter('geometry_file',str(Path(get_package_share_directory('forklift_description'))/'config/robot.yaml'))
        self.geometry=yaml.safe_load(Path(self.get_parameter('geometry_file').value).read_text())
        self.drop = False
        self.loaded = False
        self.declare_parameter('p6_load_filter',False)
        self.p6 = bool(self.get_parameter('p6_load_filter').value)
        self.create_subscription(Bool,'/mission/loaded',lambda m:setattr(self,'loaded',m.data),
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_raw', self.scan, qos_profile_sensor_data)
        self.create_service(SetBool, '~/drop_scan', self.fault)

    def scan(self, msg):
        if not self.drop:
            if self.p6 and self.loaded:
                msg=copy.deepcopy(msg)
                values=list(msg.ranges)
                for i,r in enumerate(values):
                    a=msg.angle_min+i*msg.angle_increment
                    if math.isfinite(r):
                        x=self.geometry['laser_xyz'][0]+r*math.cos(a);y=self.geometry['laser_xyz'][1]+r*math.sin(a)
                        # Only the known carried parcel volume; NaN does NOT
                        # clear unseen obstacles behind its occluded rays.
                        if self.geometry['loaded_xmin']<x<self.geometry['loaded_xmax'] and abs(y)<self.geometry['loaded_halfwidth']:values[i]=float('nan')
                msg.ranges=values
            self.pub.publish(msg)

    def fault(self, req, res):
        self.drop = req.data
        res.success = True
        res.message = 'Scan blackout' if self.drop else 'Scan restored'
        return res


def main():
    rclpy.init();node=ScanRelay()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
