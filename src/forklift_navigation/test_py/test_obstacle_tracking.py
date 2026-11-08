# SPDX-License-Identifier: Apache-2.0
"""Kiểm thử gom cụm/theo dõi vật cản (không cần ROS): python3 -m pytest src/forklift_navigation/test_py"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import obstacle_tracking as ot  # noqa: E402


def blob(cx, cy, r=.05, n=8):
    """Cung điểm như LiDAR thấy một vật tròn bán kính r (nửa mặt hướng về gốc)."""
    base = math.atan2(-cy, -cx)
    return [(cx+r*math.cos(base+a), cy+r*math.sin(base+a))
            for a in [(-math.pi/2)+k*math.pi/(n-1) for k in range(n)]]


def test_two_objects_two_clusters():
    c = ot.cluster_points(blob(1., 0.) + blob(2., 1.))
    assert len(c) == 2


def test_two_legs_merge_into_one_person():
    c = ot.cluster_points(blob(1.5, -.1, .05) + blob(1.5, .12, .05))
    assert len(c) == 1 and abs(c[0]['y']) < .05


def test_noise_single_points_dropped():
    assert ot.cluster_points([(1., 0.), (3., 3.)]) == []


def run(tracker, positions, dt=.1, noise=0.):
    t = 0.
    for i, (x, y) in enumerate(positions):
        jitter = noise*((-1)**i)
        tracker.update(ot.cluster_points(blob(x+jitter, y)), t)
        t += dt
    return tracker.tracks


def test_still_box_becomes_static_despite_jitter():
    # Thùng đứng yên, tâm cụm rung ±2 cm như LiDAR thật.
    tr = run(ot.Tracker(), [(1., .5)]*30, noise=.02)
    assert len(tr) == 1 and tr[0].label == ot.STATIC


def test_walking_person_becomes_dynamic_with_velocity():
    # Người đi ngang 0,5 m/s theo +y.
    tr = run(ot.Tracker(), [(1.5, -1.+.05*k) for k in range(30)])
    assert len(tr) == 1 and tr[0].label == ot.DYNAMIC
    assert abs(tr[0].vy-.5) < .08 and abs(tr[0].vx) < .08


def test_person_stops_returns_static_after_delay():
    path = [(1.5, -1.+.05*k) for k in range(20)] + [(1.5, -.05)]*35
    tr = run(ot.Tracker(), path)
    assert tr[0].label == ot.STATIC
    # Mới dừng 1 s thì vẫn còn ĐỘNG (trễ 2 s chống chớp nhãn).
    tr = run(ot.Tracker(), [(1.5, -1.+.05*k) for k in range(20)] + [(1.5, -.05)]*10)
    assert tr[0].label == ot.DYNAMIC


def face(cx, cy, half, n=12):
    """Mặt phẳng ngang (thùng) từ cx-half tới cx+half tại y = cy."""
    return [(cx-half+2*half*k/(n-1), cy) for k in range(n)]


def test_partially_occluded_box_stays_static():
    # Thùng mặt 35 cm đứng yên; người che dần nửa mặt -> tâm cụm dời ~9 cm.
    tracker = ot.Tracker()
    t = 0.
    for k in range(40):
        pts = face(2., -1.5, .175) if k < 20 or k >= 30 else face(2.0875, -1.5, .0875, 6)
        tracker.update(ot.cluster_points(pts), t)
        t += .1
        assert all(tr.label != ot.DYNAMIC for tr in tracker.tracks), k


def test_new_track_unknown_then_static():
    tracker = ot.Tracker()
    tracker.update(ot.cluster_points(blob(1., 0.)), 0.)
    assert tracker.tracks[0].label == ot.UNKNOWN


def test_lost_track_removed():
    tracker = ot.Tracker(max_missing=1.)
    tracker.update(ot.cluster_points(blob(1., 0.)), 0.)
    tracker.update([], 1.5)
    assert tracker.tracks == []


def test_two_people_keep_identity_when_passing():
    tracker = ot.Tracker()
    t = 0.
    for k in range(25):
        a = blob(1., -1.+.05*k)
        b = blob(2., 1.-.05*k)
        tracker.update(ot.cluster_points(a+b), t)
        t += .1
    ids = sorted(tr.id for tr in tracker.tracks)
    assert ids == [1, 2] and all(tr.label == ot.DYNAMIC for tr in tracker.tracks)


def beams(objs, n=360, far=5.):
    """Mô phỏng LiDAR ở gốc: objs = [(góc_đầu_độ, góc_cuối_độ, tầm)], vật gần đè vật xa."""
    ranges = [far]*n
    for a0, a1, r in objs:
        for k in range(a0, a1+1):
            ranges[k % n] = min(ranges[k % n], r)
    pts = [(r*math.cos(math.radians(k)), r*math.sin(math.radians(k)), k, r)
           for k, r in enumerate(ranges) if r < far]
    return pts, ranges


def test_occlusion_flag_from_neighbor_beams():
    # Thùng ở 2 m (tia 10..20); người ở 1 m che tia 8..12 -> mép trái thùng bị che.
    pts, ranges = beams([(10, 20, 2.), (8, 12, 1.)])
    by_range = {round(math.hypot(c['x'], c['y'])): c for c in ot.cluster_points(pts, ranges=ranges)}
    assert by_range[2]['occluded'] and not by_range[1]['occluded']
    pts, ranges = beams([(10, 20, 2.)])
    assert not ot.cluster_points(pts, ranges=ranges)[0]['occluded']


def test_robot_body_beams_not_counted_as_occluder():
    pts, ranges = beams([(10, 20, 2.)])
    ranges[9] = .3   # tia chạm cột thân xe ngay cạnh mép thùng
    assert not ot.cluster_points(pts, ranges=ranges)[0]['occluded']
