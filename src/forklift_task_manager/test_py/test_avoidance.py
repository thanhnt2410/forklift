# SPDX-License-Identifier: Apache-2.0
"""Kiểm thử quyết định né vật (không cần ROS): python3 -m pytest src/forklift_task_manager/test_py"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from forklift_task_manager import avoidance as av  # noqa: E402

PATH = [(x/10, 0.) for x in range(0, 31)]       # đường thẳng 3 m theo +x
CORRIDOR = .225 + .10


def obs(label, x, y, vx=0., vy=0., width=.2):
    return dict(id=1, label=label, x=x, y=y, vx=vx, vy=vy, width=width)


def test_ahead_cuts_path_length():
    pts = av.ahead(PATH, 1.0)
    assert pts[-1][0] <= 1.0 + 1e-9 and len(pts) == 11


def test_static_box_on_path_is_static_blocker():
    found = av.blockers([obs(av.STATIC, 1.5, .1)], PATH, CORRIDOR)
    assert av.blocker_kind(found) == 'static'


def test_box_beside_path_not_blocker():
    assert av.blockers([obs(av.STATIC, 1.5, .7)], PATH, CORRIDOR) == []


def test_walking_person_on_path_is_dynamic():
    found = av.blockers([obs(av.STATIC, 2.5, 0.), obs(av.DYNAMIC, 1.2, .1, vy=.5)], PATH, CORRIDOR)
    assert av.blocker_kind(found) == 'dynamic'


def test_unknown_only_costmap_obstacle():
    assert av.blocker_kind([]) is None


def test_person_will_cross_within_horizon():
    # Người ở bên trái 1 m, đi về phía đường 0,8 m/s -> vào hành lang sau ~0,7 s.
    t, o = av.predicted_conflict([obs(av.DYNAMIC, 1.5, 1.0, vy=-.8)], PATH, CORRIDOR)
    assert .5 <= t <= .9


def test_person_walking_away_no_conflict():
    assert av.predicted_conflict([obs(av.DYNAMIC, 1.5, 1.0, vy=.8)], PATH, CORRIDOR) is None


def test_static_object_not_predicted():
    assert av.predicted_conflict([obs(av.STATIC, 1.5, .1)], PATH, CORRIDOR) is None


def test_person_already_inside_corridor_time_zero():
    t, _ = av.predicted_conflict([obs(av.DYNAMIC, 1., 0., vx=-.3)], PATH, CORRIDOR)
    assert t == 0.
