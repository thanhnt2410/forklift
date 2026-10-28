# SPDX-License-Identifier: Apache-2.0
"""Planar geometry and input contracts for user-selected pallet missions."""
import math
import statistics
from .docking import wrap


def pose_values(message):
    if message.header.frame_id != 'map':
        raise ValueError('POSE_FRAME_MUST_BE_MAP')
    p, q = message.pose.position, message.pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        raise ValueError('POSE_NOT_FINITE')
    if abs(p.z) > .05 or abs(q.x) > .001 or abs(q.y) > .001:
        raise ValueError('POSE_MUST_BE_PLANAR')
    if abs(q.z*q.z + q.w*q.w - 1.) > .01:
        raise ValueError('POSE_QUATERNION_INVALID')
    return [p.x, p.y, math.atan2(2*q.w*q.z, 1-2*q.z*q.z)]


def station_from_approach(approach, distance, width, laser_x=0.):
    x, y, a = approach
    return dict(approach=list(approach),
                face=[x+(distance+laser_x)*math.cos(a), y+(distance+laser_x)*math.sin(a)],
                marker_offset_x=0., marker_offset_y=0., marker_yaw=a, width=width)


def compose(pose, local):
    x, y, a = pose
    u, v, b = local
    return [x+math.cos(a)*u-math.sin(a)*v,
            y+math.sin(a)*u+math.cos(a)*v, wrap(a+b)]


def inverse(pose):
    x, y, a = pose
    return [-math.cos(a)*x-math.sin(a)*y, math.sin(a)*x-math.cos(a)*y, -a]


def stable_face(samples, position_tolerance=.025, yaw_tolerance=.05):
    """Samples must come from distinct scans while the robot is stationary."""
    if len(samples) < 5:
        raise ValueError('PALLET_DETECTION_NOT_STABLE')
    a = samples[0][2]
    result = [statistics.median(s[i] for s in samples) for i in (0, 1)]
    result.append(wrap(a+statistics.median(wrap(s[2]-a) for s in samples)))
    if any(math.hypot(s[0]-result[0], s[1]-result[1]) > position_tolerance or
           abs(wrap(s[2]-result[2])) > yaw_tolerance for s in samples):
        raise ValueError('PALLET_DETECTION_NOT_STABLE')
    return result


def refined_approach(face, distance, laser_x=0.):
    return compose(face, [-distance-laser_x, 0., 0.])


def needs_pick_replan(previous, corrected, current=None):
    if current is None:
        return (math.hypot(corrected[0]-previous[0], corrected[1]-previous[1]) > .025 or
                abs(wrap(corrected[2]-previous[2])) > .04)
    # After early detection, compare the observed target with the robot now,
    # in the pallet frame. A map/localization shift alone must not cause a lap
    # back through the corridor entry. Docking keeps its tighter insert gates.
    error = compose(inverse(corrected), current)
    return abs(error[0]) > .15 or abs(error[1]) > .06 or abs(error[2]) > .10


def alignment_samples(current, corrected):
    """Footprint-check the stationary heading correction and lateral docking join."""
    error=compose(inverse(corrected),current)
    aligned=compose(corrected,[error[0],0.,0.])
    turn=wrap(corrected[2]-current[2]);n=max(1,math.ceil(abs(turn)/.025))
    points=[[current[0],current[1],wrap(current[2]+turn*i/n)] for i in range(n+1)]
    n=max(1,math.ceil(abs(error[1])/.01))
    points.extend([current[0]+(aligned[0]-current[0])*i/n,
                   current[1]+(aligned[1]-current[1])*i/n,corrected[2]] for i in range(1,n+1))
    return points
