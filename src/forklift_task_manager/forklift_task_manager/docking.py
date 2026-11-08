# SPDX-License-Identifier: Apache-2.0
"""Estimate a known vertical box face from actual scan points, no world poses."""
import math
import itertools
import numpy as np


def detect_face(scan, expected, width=.24, expected_yaw=0., width_range=None, stats=None):
    # stats: dict tùy chọn để ĐẾM lý do loại (chẩn đoán). Trả về None không nói được là
    # không có cụm điểm nào, hay có mà sai bề rộng/góc/độ phẳng - ba thứ cần ba cách sửa.
    def note(key, value=None):
        if stats is None:
            return
        if value is None:
            stats[key] = stats.get(key, 0)+1
        else:
            stats.setdefault(key, []).append(round(value, 3))
    # width_range=(min,max): chấp nhận bề rộng ĐO ĐƯỢC trong khoảng này (xe thật,
    # mặt LiDAR cắt qua thay đổi theo góc nhìn). None: width ± dung sai như cũ.
    roi=(width_range[1] if width_range else width)/2+.25
    clusters=[];current=[];last=None;dropouts=0
    for i,r in enumerate(scan.ranges):
        a=scan.angle_min+i*scan.angle_increment
        point=np.array([r*math.cos(a),r*math.sin(a)]) if math.isfinite(r) and scan.range_min<r<scan.range_max else None
        # YDLIDAR X4 thật rớt tia lẻ (r=0) đều đặn ~14 tia/lần: 29/10/2026 mặt
        # pallet 0,30 m ở 0,94 m bị cắt thành 4+13+13+2 điểm, mẩu dài nhất 0,105 m
        # < pallet_face_min_width -> "No pallet face near A". Bỏ qua tối đa 2 tia
        # rớt liên tiếp thay vì cắt cụm; nối cụm vẫn do ngưỡng 0,10 m giữa hai điểm
        # hợp lệ kề nhau quyết định. SIM GPU LiDAR không rớt tia nên không đổi.
        if point is None and current and dropouts<2:
            dropouts+=1;continue
        dropouts=0
        # ROI is from the map estimate; raw points determine the fitted face.
        valid=point is not None and abs(point[0]-expected[0])<roi and abs(point[1]-expected[1])<roi
        if not valid or (last is not None and np.linalg.norm(point-last)>.10):
            if len(current)>=6:clusters.append(np.asarray(current))
            current=[]
        if valid:current.append(point)
        last=point if valid else None
    if len(current)>=6:clusters.append(np.asarray(current))
    if stats is not None:
        stats['quet']=stats.get('quet',0)+1
        stats['cum']=stats.get('cum',0)+len(clusters)
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
        if width_range:span_ok=width_range[0]<=span<=width_range[1]
        else:
            min_span=max(.8*width,width-.045-2*spacing)
            span_ok=min_span<span<width+.035
        if abs(wrap(angle-expected_yaw))>.40 or not span_ok or residual>.008:
            note('bo_goc' if abs(wrap(angle-expected_yaw))>.40 else
                 ('bo_berong' if not span_ok else 'bo_khong_phang'))
            note('span',span);note('residual',residual)
            continue
        center=center+tangent*((projections.max()+projections.min())/2-center@tangent)
        candidates.append(dict(x=float(center[0]),y=float(center[1]),yaw=angle,
                               width=span,residual=residual,points=len(points)))
    if not candidates:
        return None
    note('nhan')
    return min(candidates,key=lambda f:math.hypot(f['x']-expected[0],f['y']-expected[1]))


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


def clamp(v,limit):return max(-limit,min(limit,v))


def can_advance(distance_error,lateral_error,yaw_error,lateral_tolerance,insert,yaw_tolerance=.025):
    # At long range angular sampling gives a coarser lateral estimate. Let the
    # robot approach while still OUTSIDE the insertion area, then tighten.
    if insert:
        return abs(lateral_error)<max(.014,2*lateral_tolerance) and abs(yaw_error)<.055
    tolerance=max(lateral_tolerance,.020) if distance_error>.10 else lateral_tolerance
    # yaw_tolerance > 0,025 chỉ dùng cho xe thật (bánh không quay chính xác góc nhỏ).
    return abs(lateral_error)<tolerance and abs(yaw_error)<yaw_tolerance
