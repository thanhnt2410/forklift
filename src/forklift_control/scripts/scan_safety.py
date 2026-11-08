# SPDX-License-Identifier: Apache-2.0
"""Lớp an toàn theo LiDAR cho command_gate: dự báo va chạm của lệnh tốc độ.

Thuần Python (không phụ thuộc ROS) để kiểm thử trên máy tính. Mọi tọa độ trong
khung base_link: x tiến, y trái. Footprint là hình chữ nhật [xmin, xmax] x [-hw, hw].

Ý tưởng: mô phỏng lệnh (vx, vy, wz) trong horizon giây theo mô hình vận tốc không
đổi, mỗi bước kiểm tra footprint đã nới margin có chứa điểm LiDAR nào không. Thời
điểm và quãng đường tới lúc chạm quyết định DỪNG / GIẢM TỐC. Điểm nằm sẵn trong
thân xe (hoặc trong hàng đang chở) lúc t = 0 là chính xe nên bị bỏ trước.
"""
import math


def scan_points(ranges, angle_min, angle_increment, range_min, range_max,
                laser_x=0., laser_y=0., laser_yaw=0., keep_radius=math.inf):
    """Đổi tia LiDAR sang điểm (x, y) trong base_link; bỏ tia hỏng/NaN/ngoài tầm."""
    c, s = math.cos(laser_yaw), math.sin(laser_yaw)
    points = []
    for k, r in enumerate(ranges):
        if not (math.isfinite(r) and range_min < r < range_max):
            continue
        a = angle_min + k*angle_increment
        lx, ly = r*math.cos(a), r*math.sin(a)
        x, y = laser_x + c*lx - s*ly, laser_y + s*lx + c*ly
        if x*x + y*y <= keep_radius*keep_radius:
            points.append((x, y))
    return points


def outside_self(points, xmin, xmax, hw):
    """Bỏ điểm thuộc chính xe/hàng đang chở (nằm trong envelope lúc t = 0)."""
    return [(x, y) for x, y in points if not (xmin <= x <= xmax and abs(y) <= hw)]


def outside_corridor(points, x_from, half_width):
    """Bỏ hành lang phía trước mũi xe (pallet khi cắm càng): x > x_from và |y| < half_width."""
    return [(x, y) for x, y in points if not (x > x_from and abs(y) < half_width)]


def first_contact(points, command, xmin, xmax, hw, margin, horizon, dt=.05):
    """Thời điểm và quãng đường (m) tới lần chạm đầu tiên, hoặc None nếu không chạm.

    Quãng đường gồm tịnh tiến của tâm và cung quét của góc xe xa nhất khi quay,
    để lệnh quay tại chỗ (tịnh tiến 0) vẫn có khoảng cách dừng có nghĩa.
    """
    vx, vy, wz = command
    if not points or (abs(vx) < 1e-4 and abs(vy) < 1e-4 and abs(wz) < 1e-4):
        return None
    lo, hi, half = xmin - margin, xmax + margin, hw + margin
    # 02/11/2026: điểm ĐÃ nằm trong dải margin lúc này (xe sát vật < margin) chỉ được tính
    # khi chạm thân xe thật (margin 0). Trước đây chúng làm STOP t=0.10 d=0.00 với MỌI lệnh,
    # kể cả lùi ra -> xe kẹt cứng khi teleop/vẽ map sát tường. Đi tiếp vào vật vẫn dừng ngay.
    near = [(px, py) for px, py in points if lo <= px <= hi and abs(py) <= half]
    if near:
        points = [(px, py) for px, py in points if not (lo <= px <= hi and abs(py) <= half)]
        bare = first_contact(near, command, xmin, xmax, hw, 0., horizon, dt)
        rest = first_contact(points, command, xmin, xmax, hw, margin, horizon, dt)
        found = [c for c in (bare, rest) if c is not None]
        return min(found) if found else None
    corner = math.hypot(max(abs(xmin), abs(xmax)), hw)
    speed = math.hypot(vx, vy) + corner*abs(wz)
    x = y = th = t = 0.
    steps = max(1, int(math.ceil(horizon/dt)))
    for _ in range(steps):
        # Tích phân trong khung odom tạm (gốc = tư thế hiện tại).
        c, s = math.cos(th), math.sin(th)
        x += (c*vx - s*vy)*dt
        y += (s*vx + c*vy)*dt
        th += wz*dt
        t += dt
        c, s = math.cos(th), math.sin(th)
        for px, py in points:
            dx, dy = px - x, py - y
            u, v = c*dx + s*dy, -s*dx + c*dy   # điểm trong khung xe tại thời điểm t
            if lo <= u <= hi and abs(v) <= half:
                return t, speed*t
    return None


def decide(contact, autonomous, stop_time, stop_distance, slow_time, min_scale):
    """Trả (hệ số tốc độ, trạng thái). Teleop chỉ DỪNG, không giảm tốc."""
    if contact is None:
        return 1., 'CLEAR'
    t, d = contact
    if t <= stop_time or d <= stop_distance:
        return 0., 'STOP'
    if not autonomous or t >= slow_time:
        return 1., 'CLEAR'
    scale = min_scale + (1. - min_scale)*(t - stop_time)/(slow_time - stop_time)
    return max(min_scale, min(1., scale)), 'SLOW'
