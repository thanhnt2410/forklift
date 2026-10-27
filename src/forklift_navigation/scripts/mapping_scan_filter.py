#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Validate physical scans without inventing free space or changing timestamps."""
import math
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def clean_ranges(ranges, minimum, maximum):
    # Invalid / no-return rays stay unknown, never fabricate max-range returns.
    return [r if math.isfinite(r) and minimum <= r <= maximum else float('nan')
            for r in ranges]


class MappingScanFilter(Node):
    def __init__(self):
        super().__init__('mapping_scan_filter')
        self.declare_parameter('min_range', 0.12)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('max_scan_age', 0.5)
        self.declare_parameter('relay_replay_tf', False)
        self.minimum = float(self.get_parameter('min_range').value)
        self.maximum = float(self.get_parameter('max_range').value)
        self.max_age = float(self.get_parameter('max_scan_age').value)
        if not (0 < self.minimum < self.maximum and self.max_age > 0):
            raise ValueError('Invalid mapping scan limits')
        self.last_stamp = None
        self.stats = dict(received=0, published=0, rejected=0, invalid_rays=0)
        self.pub = self.create_publisher(LaserScan, '/mapping/scan', qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/mapping/scan_status', 1)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_timer(2.0, self.report)
        if self.get_parameter('relay_replay_tf').value:
            self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
            self.create_subscription(TFMessage, '/mapping/replay_tf', self.replay_tf, 100)

    def replay_tf(self, msg):
        # The new SLAM run owns map->odom. Keep odometry and other bag TFs.
        msg.transforms = [t for t in msg.transforms if not (
            t.header.frame_id.lstrip('/') == 'map' and t.child_frame_id.lstrip('/') == 'odom')]
        if msg.transforms:
            self.tf_pub.publish(msg)

    def reject(self, reason):
        self.stats['rejected'] += 1
        self.stats['last_rejection'] = reason
        self.get_logger().warning(reason, throttle_duration_sec=5.0)

    def scan(self, msg):
        self.stats['received'] += 1
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        self.stats['last_age_seconds'] = age
        if not msg.header.frame_id or stamp <= 0:
            return self.reject('Missing scan frame or timestamp')
        if age > self.max_age or age < -0.1:
            return self.reject('Scan timestamp outside allowed age; check host clocks')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return self.reject('Scan timestamp repeated or out of order; restart after clock reset')
        if (not msg.ranges or not all(math.isfinite(v) for v in
                (msg.angle_min, msg.angle_max, msg.angle_increment, msg.range_min, msg.range_max))
                or msg.angle_increment <= 0 or msg.range_min < 0
                or msg.range_max <= msg.range_min
                or abs(msg.angle_max - msg.angle_min -
                       (len(msg.ranges) - 1) * msg.angle_increment) > 2 * msg.angle_increment):
            return self.reject('Invalid LaserScan geometry or range limits')
        minimum = max(self.minimum, msg.range_min)
        maximum = min(self.maximum, msg.range_max)
        if minimum >= maximum:
            return self.reject('Sensor range does not overlap mapping range')
        msg.ranges = clean_ranges(msg.ranges, minimum, maximum)
        valid = sum(math.isfinite(r) for r in msg.ranges)
        self.stats['invalid_rays'] += len(msg.ranges) - valid
        self.stats['last_valid_fraction'] = valid / len(msg.ranges)
        if not valid:
            return self.reject('Scan has no usable returns')
        self.last_stamp = stamp
        # Header, scan timing, angles and sensor limits are preserved for SLAM.
        self.pub.publish(msg)
        self.stats['published'] += 1

    def report(self):
        self.status.publish(String(data=json.dumps(self.stats)))


def main():
    rclpy.init()
    node = MappingScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
