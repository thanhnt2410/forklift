# SPDX-License-Identifier: Apache-2.0
"""Gom cụm điểm LiDAR và theo dõi vật cản, gắn nhãn TĨNH / ĐỘNG (thuần Python).

Dùng cho node obstacle_tracker; tách riêng để kiểm thử không cần ROS. Mọi tọa độ
trong khung map. Cách làm:
  1. Điểm LiDAR không trùng tường bản đồ được gom cụm theo thứ tự tia (đứt khi hai
     điểm liền nhau cách > gap), rồi gộp các cụm sát nhau (hai chân một người).
  2. Mỗi cụm ghép với vết (track) gần nhất trong cổng `gate`, có tính vị trí dự báo.
  3. Vận tốc lấy từ độ dời trong cửa sổ ~0,6 s (chống rung tâm cụm vài cm của LiDAR).
  4. Nhãn có trễ: ĐỘNG khi nhanh hơn dynamic_speed liên tục dynamic_time; về TĨNH khi
     chậm hơn static_speed liên tục static_time; vết mới là CHƯA RÕ tới khi đủ tuổi.
"""
import math

UNKNOWN, STATIC, DYNAMIC = 'UNKNOWN', 'STATIC', 'DYNAMIC'


def cluster_points(points, gap=.12, merge_distance=.35, merge_extent=.6, min_points=2,
                   ranges=None, occlusion_margin=.08, occluder_min_range=.6):
    """points: [(x, y)] hoặc [(x, y, k, r)] theo thứ tự góc quét (k = chỉ số tia, r = tầm).

    Trả [dict(x, y, width, n, occluded)]. Có `ranges` (toàn bộ tia của khung quét) thì
    đánh dấu cụm BỊ CHE: tia ngay ngoài mép cụm đo ngắn hơn mép > occlusion_margin,
    tức có vật đứng chắn phía trước một phần cụm -> tâm cụm không đáng tin để tính vận tốc.
    Tia ngắn hơn occluder_min_range là chính thân xe (cột, khung) nên không tính là vật che.
    """
    raw, cur = [], []
    for p in points:
        if cur and math.hypot(p[0]-cur[-1][0], p[1]-cur[-1][1]) > gap:
            raw.append(cur)
            cur = []
        cur.append(p)
    if cur:
        raw.append(cur)
    # Tia cuối và tia đầu kề nhau trên vòng 360°.
    if len(raw) > 1 and math.hypot(raw[0][0][0]-raw[-1][-1][0], raw[0][0][1]-raw[-1][-1][1]) <= gap:
        raw[0] = raw.pop() + raw[0]
    groups = [c for c in raw if len(c) >= min_points]
    merged = True
    while merged:
        merged = False
        for i in range(len(groups)):
            for j in range(i+1, len(groups)):
                a, b = groups[i], groups[j]
                ca, cb = _centroid(a), _centroid(b)
                if (math.hypot(ca[0]-cb[0], ca[1]-cb[1]) <= merge_distance
                        and _extent(a+b) <= merge_extent):
                    groups[i] = a+b
                    del groups[j]
                    merged = True
                    break
            if merged:
                break
    out = []
    for g in groups:
        cx, cy = _centroid(g)
        out.append(dict(x=cx, y=cy, width=_extent(g), n=len(g),
                        occluded=_occluded(g, ranges, occlusion_margin, occluder_min_range)))
    return out


def _occluded(group, ranges, margin, min_range):
    if not ranges or len(group[0]) < 4:
        return False
    n = len(ranges)
    ks = [p[2] for p in group]
    lo, hi = min(ks), max(ks)
    if hi - lo > n//2:          # cụm vắt qua tia 0 (hiếm): bỏ qua kiểm tra
        return False
    for edge, k in ((lo, lo-1), (hi, hi+1)):
        r_edge = next(p[3] for p in group if p[2] == edge)
        r = ranges[k % n]
        if math.isfinite(r) and min_range < r < r_edge - margin:
            return True
    return False


def _centroid(pts):
    return sum(p[0] for p in pts)/len(pts), sum(p[1] for p in pts)/len(pts)


def _extent(pts):
    return max(math.hypot(a[0]-b[0], a[1]-b[1]) for a in pts for b in pts) if len(pts) > 1 else 0.


class Track:
    def __init__(self, track_id, cluster, t):
        self.id = track_id
        self.x, self.y = cluster['x'], cluster['y']
        self.width = cluster['width']
        self.vx = self.vy = 0.
        self.first = self.last = t
        self.history = [(t, self.x, self.y)]
        self.label = UNKNOWN
        self.fast_since = None
        self.slow_since = None

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)

    def predict(self, t):
        dt = t - self.last
        return self.x + self.vx*dt, self.y + self.vy*dt

    def as_dict(self):
        return dict(id=self.id, label=self.label, x=round(self.x, 3), y=round(self.y, 3),
                    vx=round(self.vx, 3), vy=round(self.vy, 3), speed=round(self.speed, 3),
                    width=round(self.width, 3), age=round(self.last-self.first, 2))


class Tracker:
    def __init__(self, gate=.35, max_missing=1.0, velocity_window=.6, min_age=1.0,
                 dynamic_speed=.12, dynamic_time=.3, static_speed=.05, static_time=2.0):
        self.p = dict(gate=gate, max_missing=max_missing, velocity_window=velocity_window,
                      min_age=min_age, dynamic_speed=dynamic_speed, dynamic_time=dynamic_time,
                      static_speed=static_speed, static_time=static_time)
        self.tracks = []
        self.next_id = 1

    def update(self, clusters, t):
        p = self.p
        # Ghép tham lam theo khoảng cách tăng dần giữa vị trí dự báo và tâm cụm.
        pairs = []
        for ti, tr in enumerate(self.tracks):
            px, py = tr.predict(t)
            for ci, c in enumerate(clusters):
                d = math.hypot(c['x']-px, c['y']-py)
                if d <= p['gate']:
                    pairs.append((d, ti, ci))
        used_t, used_c = set(), set()
        for _, ti, ci in sorted(pairs):
            if ti in used_t or ci in used_c:
                continue
            used_t.add(ti)
            used_c.add(ci)
            self._correct(self.tracks[ti], clusters[ci], t)
        for ci, c in enumerate(clusters):
            if ci not in used_c:
                self.tracks.append(Track(self.next_id, c, t))
                self.next_id += 1
        self.tracks = [tr for tr in self.tracks if t - tr.last <= p['max_missing']]
        return self.tracks

    def _correct(self, tr, c, t):
        p = self.p
        # Bị che một phần (người đi qua trước thùng) hoặc bị gộp với vật bên cạnh: bề
        # rộng nhảy đột ngột và tâm cụm dời vài cm dù vật đứng yên. Không dùng lần đo
        # đó để tính vận tốc (SIM 01/11/2026: thùng bị che bị gán ĐỘNG ~2 s).
        jump = (c.get('occluded', False)
                or (tr.width > .15 and not (.6*tr.width <= c['width'] <= 1.67*tr.width)))
        tr.x, tr.y, tr.width, tr.last = c['x'], c['y'], c['width'], t
        if jump:
            tr.history = [(t, tr.x, tr.y)]
            tr.fast_since = None
            return
        tr.history.append((t, tr.x, tr.y))
        while len(tr.history) > 2 and t - tr.history[1][0] >= p['velocity_window']:
            tr.history.pop(0)
        t0, x0, y0 = tr.history[0]
        if t - t0 >= .5*p['velocity_window']:
            tr.vx, tr.vy = (tr.x-x0)/(t-t0), (tr.y-y0)/(t-t0)
        speed = tr.speed
        tr.fast_since = (tr.fast_since or t) if speed >= p['dynamic_speed'] else None
        tr.slow_since = (tr.slow_since or t) if speed <= p['static_speed'] else None
        # Vết mới sinh (thường do che khuất/tách cụm) phải đủ tuổi mới được gán ĐỘNG.
        if (tr.fast_since is not None and t - tr.fast_since >= p['dynamic_time']
                and t - tr.first >= .5*p['min_age']):
            tr.label = DYNAMIC
        elif tr.label == DYNAMIC:
            if tr.slow_since is not None and t - tr.slow_since >= p['static_time']:
                tr.label = STATIC
        elif t - tr.first >= p['min_age'] and tr.fast_since is None:
            tr.label = STATIC
