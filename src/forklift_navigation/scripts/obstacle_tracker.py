#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Nhận biết vật cản TĨNH / ĐỘNG từ LiDAR 2D (bước 2 né vật, 01/11/2026).

Chỉ hiển thị và công bố, KHÔNG điều khiển xe:
  /obstacles/markers  MarkerArray: xanh = tĩnh, đỏ = động (mũi tên = vị trí sau 1 s),
                      vàng = chưa rõ.
  /obstacles/tracks   String JSON: danh sách vật (id, nhãn, vị trí, vận tốc) cho bước 3.
Bỏ điểm trùng tường trên /map và điểm thuộc thân xe/hàng đang chở, nên chỉ còn vật
KHÔNG có trong bản đồ tĩnh (thùng mới đặt, người, pallet).
"""
import json
import math

import rclpy
import tf2_ros
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

import obstacle_tracking as ot

COLORS = {ot.STATIC: (.15, .75, .25), ot.DYNAMIC: (.95, .15, .15), ot.UNKNOWN: (.95, .8, .1)}


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class ObstacleTracker(Node):
    def __init__(self):
        super().__init__('obstacle_tracker')
        defaults = dict(map_clearance=.10, max_range=6., cluster_gap=.12, merge_distance=.35,
                        merge_extent=.6, min_points=2, gate=.35, max_missing=1.0,
                        velocity_window=.6, min_age=1.0, dynamic_speed=.12, dynamic_time=.3,
                        static_speed=.05, static_time=2.0,
                        footprint_xmin=-.1751, footprint_xmax=.4513, footprint_halfwidth=.225145,
                        loaded_xmax=.52, loaded_halfwidth=.20, self_margin=.05)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: float(self.get_parameter(k).value) for k in defaults}
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.map_frame = self.get_parameter('map_frame').value
        p = self.p
        self.tracker = ot.Tracker(p['gate'], p['max_missing'], p['velocity_window'], p['min_age'],
                                  p['dynamic_speed'], p['dynamic_time'], p['static_speed'], p['static_time'])
        self.occupied = None
        self.loaded = False
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacles/markers', 10)
        self.track_pub = self.create_publisher(String, '/obstacles/tracks', 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(Bool, '/mission/loaded', lambda m: setattr(self, 'loaded', m.data), latched)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.on_scan,
                                 qos_profile_sensor_data)
        self.warned = set()

    def on_map(self, msg):
        # Tập ô chiếm của bản đồ tĩnh, nới bằng map_clearance để bỏ điểm sát tường.
        info = msg.info
        r = max(0, int(math.ceil(self.p['map_clearance']/info.resolution)))
        occ = set()
        W = info.width
        for k, v in enumerate(msg.data):
            if v > 50:
                i, j = k % W, k // W
                for a in range(-r, r+1):
                    for b in range(-r, r+1):
                        occ.add((i+a, j+b))
        self.map_info = info
        self.occupied = occ
        self.get_logger().info(f'Static map: {len(occ)} cells after {self.p["map_clearance"]:.2f} m clearance')

    def lookup(self, target, source):
        try:
            return self.buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception as error:
            key = (target, source)
            if key not in self.warned:
                self.get_logger().warning(f'Waiting for TF {target} <- {source}: {error}')
                self.warned.add(key)
            return None

    def on_scan(self, msg):
        if self.occupied is None:
            return
        laser = self.lookup(self.map_frame, msg.header.frame_id)
        base = self.lookup(self.map_frame, 'base_link')
        if laser is None or base is None:
            return
        p, info = self.p, self.map_info
        lx, ly = laser.transform.translation.x, laser.transform.translation.y
        la = yaw_of(laser.transform.rotation)
        bx, by = base.transform.translation.x, base.transform.translation.y
        ba = yaw_of(base.transform.rotation)
        cb, sb = math.cos(ba), math.sin(ba)
        xmax = max(p['footprint_xmax'], p['loaded_xmax']) if self.loaded else p['footprint_xmax']
        hw = max(p['footprint_halfwidth'], p['loaded_halfwidth']) if self.loaded else p['footprint_halfwidth']
        m = p['self_margin']
        ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
        points = []
        for k, r in enumerate(msg.ranges):
            if not (math.isfinite(r) and msg.range_min < r < min(msg.range_max, p['max_range'])):
                continue
            a = la + msg.angle_min + k*msg.angle_increment
            x, y = lx + r*math.cos(a), ly + r*math.sin(a)
            if (int((x-ox)/res), int((y-oy)/res)) in self.occupied:
                continue
            u, v = cb*(x-bx) + sb*(y-by), -sb*(x-bx) + cb*(y-by)
            if p['footprint_xmin']-m <= u <= xmax+m and abs(v) <= hw+m:
                continue    # chính xe hoặc hàng đang chở
            points.append((x, y, k, r))
        clusters = ot.cluster_points(points, p['cluster_gap'], p['merge_distance'],
                                     p['merge_extent'], int(p['min_points']), list(msg.ranges))
        now = self.get_clock().now()
        tracks = self.tracker.update(clusters, now.nanoseconds*1e-9)
        self.publish(tracks, msg.header.stamp)

    def publish(self, tracks, stamp):
        visible = [t for t in tracks if t.last == max((x.last for x in tracks), default=0.)]
        self.track_pub.publish(String(data=json.dumps(
            dict(frame=self.map_frame, obstacles=[t.as_dict() for t in visible]))))
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        life = Duration(sec=0, nanosec=500_000_000)
        for t in visible:
            r, g, b = COLORS[t.label]
            body = Marker()
            body.header.frame_id, body.header.stamp = self.map_frame, stamp
            body.ns, body.id, body.type, body.action = 'obstacle', t.id, Marker.CYLINDER, Marker.ADD
            body.pose.position.x, body.pose.position.y, body.pose.position.z = t.x, t.y, .2
            body.pose.orientation.w = 1.
            d = max(t.width, .12)
            body.scale.x, body.scale.y, body.scale.z = d, d, .4
            body.color.r, body.color.g, body.color.b, body.color.a = r, g, b, .55
            body.lifetime = life
            arr.markers.append(body)
            text = Marker()
            text.header = body.header
            text.ns, text.id, text.type, text.action = 'label', t.id, Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = t.x, t.y, .55
            text.pose.orientation.w = 1.
            text.scale.z = .09
            text.color.r = text.color.g = text.color.b = text.color.a = 1.
            text.text = (f'#{t.id} {t.label} {t.speed:.2f} m/s' if t.label == ot.DYNAMIC
                         else f'#{t.id} {t.label}')
            text.lifetime = life
            arr.markers.append(text)
            if t.label == ot.DYNAMIC:
                arrow = Marker()
                arrow.header = body.header
                arrow.ns, arrow.id, arrow.type, arrow.action = 'velocity', t.id, Marker.ARROW, Marker.ADD
                arrow.points = [Point(x=t.x, y=t.y, z=.2), Point(x=t.x+t.vx, y=t.y+t.vy, z=.2)]
                arrow.scale.x, arrow.scale.y, arrow.scale.z = .03, .07, .1
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, g, b, 1.
                arrow.lifetime = life
                arr.markers.append(arrow)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = ObstacleTracker()
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
