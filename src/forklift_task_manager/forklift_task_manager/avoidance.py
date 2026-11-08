# SPDX-License-Identifier: Apache-2.0
"""Bước 3 né vật (01/11/2026): quyết định theo loại vật cản trên đường (thuần Python).

Đầu vào là danh sách vật từ /obstacles/tracks (obstacle_tracker): dict id, label
(STATIC/DYNAMIC/UNKNOWN), x, y, vx, vy, width trong khung map; và đường còn lại của xe
[(x, y)] trong khung map. Hành lang của xe = nửa bề rộng xe + margin quanh đường.
"""
import math

STATIC, DYNAMIC, UNKNOWN = 'STATIC', 'DYNAMIC', 'UNKNOWN'


def ahead(points, max_distance):
    """Cắt đường còn lại tới max_distance (m) tính dọc theo đường."""
    if not points:
        return []
    out, total = [points[0]], 0.
    for a, b in zip(points, points[1:]):
        total += math.hypot(b[0]-a[0], b[1]-a[1])
        if total > max_distance:
            break
        out.append(b)
    return out


def distance_to_path(x, y, points):
    """Khoảng cách nhỏ nhất từ (x, y) tới đường gấp khúc."""
    if not points:
        return math.inf
    if len(points) == 1:
        return math.hypot(x-points[0][0], y-points[0][1])
    best = math.inf
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx-ax, by-ay
        n = dx*dx + dy*dy
        u = 0. if n < 1e-12 else max(0., min(1., ((x-ax)*dx + (y-ay)*dy)/n))
        best = min(best, math.hypot(x-(ax+u*dx), y-(ay+u*dy)))
    return best


def on_path(obstacle, points, corridor):
    """Vật (kể cả bề rộng của nó) có lấn vào hành lang của đường không."""
    return distance_to_path(obstacle['x'], obstacle['y'], points) <= corridor + .5*obstacle.get('width', 0.)


def blockers(obstacles, points, corridor):
    return [o for o in obstacles if on_path(o, points, corridor)]


def blocker_kind(found):
    """'dynamic' nếu có vật đang đi trên đường (nhường), 'static' nếu chỉ có vật đứng
    yên/chưa rõ (vòng ngay), None nếu không thấy vật nào (vật cản chỉ có trên costmap)."""
    if any(o['label'] == DYNAMIC for o in found):
        return 'dynamic'
    if found:
        return 'static'
    return None


def crossing_time(obstacle, points, corridor, horizon=1.5, dt=.1):
    """Thời điểm (s) vật ĐỘNG dự kiến đi vào hành lang đường, hoặc None.

    Dự báo vận tốc không đổi; vật đã nằm trong hành lang thì trả 0.
    """
    if obstacle['label'] != DYNAMIC:
        return None
    x, y = obstacle['x'], obstacle['y']
    vx, vy = obstacle.get('vx', 0.), obstacle.get('vy', 0.)
    reach = corridor + .5*obstacle.get('width', 0.)
    t = 0.
    while t <= horizon + 1e-9:
        if distance_to_path(x+vx*t, y+vy*t, points) <= reach:
            return t
        t += dt
    return None


def predicted_conflict(obstacles, points, corridor, horizon=1.5):
    """Vật động sớm nhất sẽ cắt đường trong horizon giây: (thời điểm, vật) hoặc None."""
    best = None
    for o in obstacles:
        t = crossing_time(o, points, corridor, horizon)
        if t is not None and (best is None or t < best[0]):
            best = (t, o)
    return best
