#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Tự đặt 2D Pose: so khớp LiDAR với /map rồi gửi /initialpose cho AMCL.

  ros2 run forklift_navigation auto_localize.py            # quanh pose AMCL hiện tại
  ros2 run forklift_navigation auto_localize.py --global   # khắp bản đồ (bê xe sang chỗ khác)

Chỉ đọc /map, /scan_raw (hoặc --scan) và TF; không ra lệnh bánh. Chạy trên laptop
(cùng máy Nav2), Pi cũng được nhưng chậm hơn và Pi đang bận LiDAR/ESP32/EKF.
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros
from scipy.ndimage import distance_transform_edt


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Matcher:
    """Điểm khớp = trung bình exp(-d²/2σ²) với d là khoảng cách điểm LiDAR tới ô tường gần nhất."""
    def __init__(self, grid, sigma):
        info = grid.info
        self.res, self.ox, self.oy = info.resolution, info.origin.position.x, info.origin.position.y
        data = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width)
        self.free = (data >= 0) & (data < 25)
        self.dist = distance_transform_edt(data < 65)*self.res
        self.score = np.exp(-self.dist**2/(2*sigma**2)).astype(np.float32)
        self.H, self.W = self.score.shape

    def evaluate(self, poses, pts):
        # poses: (N,3) [x,y,yaw]; pts: (M,2) điểm LiDAR trong base_link. Trả về (N,) điểm khớp.
        c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
        gx = poses[:, 0:1] + c*pts[None, :, 0] - s*pts[None, :, 1]
        gy = poses[:, 1:2] + s*pts[None, :, 0] + c*pts[None, :, 1]
        i = ((gx-self.ox)/self.res).astype(np.int32)
        j = ((gy-self.oy)/self.res).astype(np.int32)
        ok = (i >= 0) & (i < self.W) & (j >= 0) & (j < self.H)
        v = np.zeros(i.shape, np.float32)
        v[ok] = self.score[j[ok], i[ok]]
        return v.mean(axis=1)

    def match_ratio(self, pose, pts, tol):
        # Tỉ lệ điểm LiDAR nằm cách tường <= tol: con số dễ đọc cho người vận hành.
        c, s = math.cos(pose[2]), math.sin(pose[2])
        gx = pose[0]+c*pts[:, 0]-s*pts[:, 1]; gy = pose[1]+s*pts[:, 0]+c*pts[:, 1]
        i = ((gx-self.ox)/self.res).astype(np.int32); j = ((gy-self.oy)/self.res).astype(np.int32)
        ok = (i >= 0) & (i < self.W) & (j >= 0) & (j < self.H)
        d = np.full(len(pts), 9.)
        d[ok] = self.dist[j[ok], i[ok]]
        return float(np.mean(d <= tol))


def search(m, pts, center, radius, angle, step, astep, batch=20000):
    xs = np.arange(-radius, radius+1e-9, step)
    ts = np.arange(-angle, angle+1e-9, astep)
    X, Y, T = np.meshgrid(center[0]+xs, center[1]+xs, center[2]+ts, indexing='ij')
    poses = np.stack([X.ravel(), Y.ravel(), T.ravel()], 1)
    return poses, np.concatenate([m.evaluate(poses[k:k+batch], pts) for k in range(0, len(poses), batch)])


def refine(m, pts_all, pose):
    for radius, angle, step, astep in ((.12, math.radians(6), .02, math.radians(1)),
                                       (.03, math.radians(1.5), .01, math.radians(.5))):
        poses, sc = search(m, pts_all, pose, radius, angle, step, astep)
        pose = poses[int(np.argmax(sc))]
    return pose, float(m.evaluate(np.array([pose]), pts_all)[0])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--global', dest='global_search', action='store_true', help='tìm khắp vùng trống của bản đồ')
    ap.add_argument('--radius', type=float, default=1.0, help='bán kính tìm quanh pose AMCL (m), mặc định 1,0')
    ap.add_argument('--angle', type=float, default=90., help='góc tìm quanh pose AMCL (độ), mặc định ±90')
    ap.add_argument('--scan', default='/scan_raw', help='topic LiDAR (mặc định /scan_raw)')
    ap.add_argument('--dry-run', action='store_true', help='chỉ in kết quả, không gửi /initialpose')
    # Ba cờ dưới cho navigation_real.launch.py tự chạy lúc khởi động (02/11/2026): AMCL
    # set_initial_pose=False nên chưa có frame map tới khi có /initialpose, global costmap
    # chờ mãi "frame does not exist". Scan qua Wi-Fi có thể tới muộn hàng chục giây.
    ap.add_argument('--wait', type=float, default=15., help='chờ /map, scan, TF và AMCL tối đa (s), mặc định 15')
    ap.add_argument('--strict', action='store_true',
                    help='có tư thế khác gần ngang điểm (phòng đối xứng) thì KHÔNG gửi, để người vận hành đặt tay')
    ap.add_argument('--if-unlocalized', action='store_true',
                    help='đã có TF map -> base_link (người vận hành đặt pose rồi) thì bỏ qua')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node('auto_localize')
    log = node.get_logger()
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, node)
    got = {}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, '/map', lambda m: got.__setitem__('map', m), latched)
    node.create_subscription(LaserScan, args.scan, lambda m: got.__setitem__('scan', m), qos_profile_sensor_data)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    started = time.monotonic(); end = started+args.wait; sensor = robot = None
    while time.monotonic() < end and (sensor is None or 'map' not in got or (robot is None and not args.global_search)):
        rclpy.spin_once(node, timeout_sec=.1)
        if 'scan' in got:
            try:
                sensor = buf.lookup_transform('base_link', got['scan'].header.frame_id, rclpy.time.Time()).transform
                robot = buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
            except Exception:
                pass
    if 'map' not in got or sensor is None:
        log.error('Thiếu /map, %s hoặc TF base_link <- laser (navigation/LiDAR đã chạy chưa? cùng ROS_DOMAIN_ID?)' % args.scan)
        return 1
    if robot is None and not args.global_search:
        log.error('Chưa có TF map -> base_link (AMCL chưa chạy); thử --global')
        return 1
    if args.if_unlocalized:
        # Vòng chờ trên thoát ngay khi có /map + scan (chế độ --global), buffer TF có thể
        # chưa kịp nhận map -> odom: nghe thêm tối đa 2 s rồi mới kết luận chưa định vị.
        tf_end = time.monotonic()+2.
        while time.monotonic() < tf_end:
            try:
                buf.lookup_transform('map', 'base_link', rclpy.time.Time())
                log.info('Đã có pose AMCL (TF map -> base_link): không tự định vị lại')
                return 0
            except Exception:
                rclpy.spin_once(node, timeout_sec=.1)

    # Điểm LiDAR trong base_link (LiDAR xe thật lắp xoay +90°: dùng TF thật, không giả định).
    scan = got['scan']; a0 = yaw(sensor.rotation)
    r = np.array(scan.ranges, dtype=np.float64)
    a = scan.angle_min+np.arange(len(r))*scan.angle_increment+a0
    ok = np.isfinite(r) & (r >= max(.12, scan.range_min)) & (r <= scan.range_max)
    pts_all = np.stack([sensor.translation.x+r[ok]*np.cos(a[ok]), sensor.translation.y+r[ok]*np.sin(a[ok])], 1)
    if len(pts_all) < 50:
        log.error('LiDAR chỉ có %d điểm hợp lệ' % len(pts_all)); return 1
    pts = pts_all[::3]
    m = Matcher(got['map'], sigma=.04)
    t0 = time.monotonic()

    if args.global_search:
        # Mọi ô trống (bước 10 cm) × 72 hướng (5°), rồi tinh chỉnh 10 ứng viên tốt nhất.
        jj, ii = np.nonzero(m.free[::2, ::2]) if m.res < .075 else np.nonzero(m.free)
        k = 2 if m.res < .075 else 1
        xy = np.stack([m.ox+(ii*k+.5)*m.res, m.oy+(jj*k+.5)*m.res], 1)
        ts = np.radians(np.arange(0, 360, 5))
        poses = np.concatenate([np.column_stack([xy, np.full(len(xy), t)]) for t in ts])
        sc = np.concatenate([m.evaluate(poses[i:i+20000], pts) for i in range(0, len(poses), 20000)])
        order = np.argsort(sc)[::-1]
        seeds = []
        for idx in order:
            p = poses[idx]
            if all(math.hypot(p[0]-q[0], p[1]-q[1]) > .3 or abs(wrap(p[2]-q[2])) > math.radians(20) for q in seeds):
                seeds.append(p)
            if len(seeds) >= 10:
                break
    else:
        cur = (robot.translation.x, robot.translation.y, yaw(robot.rotation))
        log.info('Pose AMCL hiện tại: x=%.3f y=%.3f yaw=%.1f°, khớp %.0f%%' % (
            cur[0], cur[1], math.degrees(cur[2]), 100*m.match_ratio(cur, pts_all, .08)))
        poses, sc = search(m, pts, cur, args.radius, math.radians(args.angle), .05, math.radians(3))
        order = np.argsort(sc)[::-1]
        seeds = []
        for idx in order:
            p = poses[idx]
            if all(math.hypot(p[0]-q[0], p[1]-q[1]) > .3 or abs(wrap(p[2]-q[2])) > math.radians(20) for q in seeds):
                seeds.append(p)
            if len(seeds) >= 5:
                break

    results = sorted((refine(m, pts_all, s) for s in seeds), key=lambda x: -x[1])
    best, score = results[0]
    ratio = m.match_ratio(best, pts_all, .08)
    log.info('Tốt nhất: x=%.3f y=%.3f yaw=%.1f°, điểm %.3f, khớp bản đồ %.0f%% (%.1f s)' % (
        best[0], best[1], math.degrees(wrap(best[2])), score, 100*ratio, time.monotonic()-t0))
    # Phòng đối xứng có thể cho hai tư thế gần ngang điểm: cảnh báo để người vận hành nhìn lại.
    other = next((p for p, s in results[1:] if s > .9*score and
                  (math.hypot(p[0]-best[0], p[1]-best[1]) > .3 or abs(wrap(p[2]-best[2])) > math.radians(20))), None)
    if other is not None:
        log.warn('Có tư thế khác gần ngang điểm: x=%.2f y=%.2f yaw=%.0f° -> kiểm tra LiDAR trong RViz' % (
            other[0], other[1], math.degrees(wrap(other[2]))))
        if args.strict:
            log.error('--strict: không tự gửi /initialpose khi có hai tư thế gần ngang điểm; đặt 2D Pose Estimate trên RViz')
            return 1
    if ratio < .5:
        log.error('Khớp dưới 50%%: không gửi /initialpose (bản đồ sai phòng, bị che nhiều hoặc đồ đạc đổi chỗ)')
        return 1
    if args.dry_run:
        return 0

    msg = PoseWithCovarianceStamped(); msg.header.frame_id = 'map'
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(best[0]), float(best[1])
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(best[2]/2), math.cos(best[2]/2)
    cov = [0.]*36; cov[0] = cov[7] = .01; cov[35] = .01; msg.pose.covariance = cov
    # AMCL chỉ đăng ký /initialpose khi đã active (lifecycle), có thể sau khi /map đã tới.
    end = max(time.monotonic()+3., started+args.wait)
    while pub.get_subscription_count() == 0 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    if pub.get_subscription_count() == 0:
        log.error('Không có AMCL nhận /initialpose'); return 1
    for _ in range(3):
        msg.header.stamp = node.get_clock().now().to_msg(); pub.publish(msg); rclpy.spin_once(node, timeout_sec=.3)
    log.info('Đã gửi /initialpose. Kiểm tra RViz: điểm LiDAR phải nằm đè lên tường.')
    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
