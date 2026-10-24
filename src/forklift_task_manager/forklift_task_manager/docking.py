# SPDX-License-Identifier: Apache-2.0
"""Estimate a known vertical box face from actual scan points, no world poses."""
import math
import itertools
import numpy as np


def detect_face(scan, expected, width=.24, expected_yaw=0.):
    clusters=[];current=[];last=None
    for i,r in enumerate(scan.ranges):
        a=scan.angle_min+i*scan.angle_increment
        point=np.array([r*math.cos(a),r*math.sin(a)]) if math.isfinite(r) and scan.range_min<r<scan.range_max else None
        # ROI is from the map estimate; raw points determine the fitted face.
        valid=point is not None and abs(point[0]-expected[0])<width/2+.25 and abs(point[1]-expected[1])<width/2+.25
        if not valid or (last is not None and np.linalg.norm(point-last)>.10):
            if len(current)>=6:clusters.append(np.asarray(current))
            current=[]
        if valid:current.append(point)
        last=point if valid else None
    if len(current)>=6:clusters.append(np.asarray(current))
    candidates=[]
    for points in clusters:
        # Corner/side returns belong to the same scan cluster. Fit the dominant
        # plane robustly before using its normal for the long-offset drop marker.
        best=None
        if len(points)>=8:
            indexes=sorted(set([0,len(points)//4,len(points)//2,3*len(points)//4,len(points)-1]))
            for i,j in itertools.combinations(indexes,2):
                delta=points[j]-points[i];length=np.linalg.norm(delta)
                if length<.08:continue
                n=np.array([-delta[1],delta[0]])/length
                mask=np.abs((points-points[i])@n)<.006
                count=int(mask.sum())
                if best is None or count>best[0]:best=(count,mask)
            if best is not None and best[0]>=6:points=points[best[1]]
        center=points.mean(axis=0);cov=(points-center).T@(points-center)
        _,vec=np.linalg.eigh(cov);normal=vec[:,0]
        if normal@np.array([math.cos(expected_yaw),math.sin(expected_yaw)])<0:normal=-normal
        tangent=np.array([-normal[1],normal[0]])
        projections=points@tangent;span=float(projections.max()-projections.min())
        residual=float(np.sqrt(np.mean(((points-center)@normal)**2)))
        angle=math.atan2(normal[1],normal[0])
        spacing=float(np.max(np.abs(np.diff(projections))))
        min_span=max(.8*width,width-.045-2*spacing)
        if abs(wrap(angle-expected_yaw))>.40 or not min_span<span<width+.035 or residual>.008:continue
        center=center+tangent*((projections.max()+projections.min())/2-center@tangent)
        candidates.append(dict(x=float(center[0]),y=float(center[1]),yaw=angle,
                               width=span,residual=residual,points=len(points)))
    return min(candidates,key=lambda f:math.hypot(f['x']-expected[0],f['y']-expected[1])) if candidates else None


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


def clamp(v,limit):return max(-limit,min(limit,v))


def can_advance(distance_error,lateral_error,yaw_error,lateral_tolerance,insert):
    # At long range angular sampling gives a coarser lateral estimate. Let the
    # robot approach while still OUTSIDE the insertion area, then tighten.
    if insert:
        return abs(lateral_error)<max(.014,2*lateral_tolerance) and abs(yaw_error)<.055
    tolerance=max(lateral_tolerance,.020) if distance_error>.10 else lateral_tolerance
    return abs(lateral_error)<tolerance and abs(yaw_error)<.025
