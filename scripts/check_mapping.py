#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Read-only mapping audit; emits JSON, does not command the robot."""
import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


class MappingAudit(Node):
    def __init__(self, sim_time, scan_topic):
        super().__init__('mapping_audit', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=sim_time)])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pending = deque()
        self.result = dict(scans=0, rays=0, valid_rays=0, tf_ok=0, tf_failed=0,
                           odom_messages=0, out_of_order=0)
        self.ages = []
        self.last_stamp = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaserScan, scan_topic, self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map_probability', self.map, qos)
        self.create_subscription(RosPath, '/mapping/trajectory', self.path, qos)
        self.create_timer(0.1, self.check_tf)

    def odom(self, msg):
        self.result['odom_messages'] += 1

    def scan(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_stamp is not None and stamp.nanoseconds <= self.last_stamp:
            self.result['out_of_order'] += 1
        self.last_stamp = stamp.nanoseconds
        self.ages.append((self.get_clock().now() - stamp).nanoseconds / 1e9)
        self.result['scans'] += 1
        self.result['rays'] += len(msg.ranges)
        self.result['valid_rays'] += sum(math.isfinite(r) and msg.range_min <= r <= msg.range_max
                                         for r in msg.ranges)
        self.pending.append((time.monotonic(), msg.header.frame_id, stamp))

    def check_tf(self):
        remaining = deque()
        while self.pending:
            received, frame, stamp = self.pending.popleft()
            try:
                # Match upstream odom/base lookup and verify sensor extrinsics,
                # using the measurement timestamp, with up to one second grace.
                self.tf.lookup_transform('odom', 'base_link', stamp)
                self.tf.lookup_transform('base_link', frame, stamp)
                self.result['tf_ok'] += 1
            except TransformException:
                if time.monotonic() - received < 1.0:
                    remaining.append((received, frame, stamp))
                else:
                    self.result['tf_failed'] += 1
        self.pending = remaining

    def map(self, msg):
        counts = Counter(msg.data)
        self.result['map'] = dict(width=msg.info.width, height=msg.info.height,
            resolution=msg.info.resolution, frame=msg.header.frame_id,
            unknown_cells=counts[-1], intermediate_cells=sum(n for p, n in counts.items() if 0 < p < 100),
            values=sorted(counts))

    def path(self, msg):
        self.result['trajectory_poses'] = len(msg.poses)
        if len(msg.poses) >= 2:
            a, b = msg.poses[0].pose.position, msg.poses[-1].pose.position
            self.result['start_end_distance_m'] = math.hypot(a.x - b.x, a.y - b.y)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument('--scan-topic', default='/mapping/scan')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be positive')
    rclpy.init()
    node = MappingAudit(args.use_sim_time, args.scan_topic)
    end = time.monotonic() + args.duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.check_tf()
        result = node.result
        result['tf_pending'] = len(node.pending)
        if node.ages:
            ages = sorted(node.ages)
            result['scan_age_seconds'] = dict(min=ages[0], max=ages[-1],
                                               median=ages[len(ages) // 2])
        result['data_available'] = bool(result['scans'] and result['odom_messages'] and
                                         result.get('map') and result.get('trajectory_poses'))
        result['map_geometry_valid'] = bool(result.get('map') and
            result['map']['resolution'] > 0 and result['map']['width'] > 0 and
            result['map']['height'] > 0 and result['map']['frame'] == 'map')
        report = json.dumps(result, indent=2) + '\n'
        print(report, end='')
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(report)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
