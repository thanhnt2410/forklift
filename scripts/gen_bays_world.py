# Dùng: python3 scripts/gen_bays_world.py ~/forklift  (rồi colcon build forklift_gazebo forklift_navigation)
# Sinh world Gazebo "bays" (kho có ô pallet hai bên, lối đi giữa) và bản đồ tĩnh tương ứng.
import math, sys
from pathlib import Path

repo = Path(sys.argv[1])
world_out = repo/'src/forklift_gazebo/worlds/bays.sdf'
map_dir = repo/'src/forklift_navigation/maps'

LX, LY, T, H = 6.0, 4.0, 0.10, 1.0          # trong lòng kho 6 x 4 m, tường dày 0,1 m, cao 1 m
BAY_X = (0.8, 1.6, 2.4)                      # ô có pallet (phía +x, xe nhìn thẳng tới)
DROP_X = (-2.2,)                             # ô trống để thả (phía -x)
FACE_Y = 1.45                                # mặt trước thùng hàng cách trục lối đi
# Kích thước người dùng đo (01/11/2026): pallet mặt cắm càng 20,5 cm x sâu 16 cm x cao 4,5 cm,
# một khe cắm càng rộng 20 cm (không thanh giữa); thùng 35x25x15 cm dựng đứng: cao 25 cm,
# mặt 35 cm quay về xe (chìa ra hai bên pallet 7,25 cm), sâu 15 cm theo chiều càng.
# Hai càng sải ngoài 16 cm: dư 2 cm mỗi bên khe.
PAL_L, PAL_W, PAL_H = 0.16, 0.205, 0.045    # sâu theo chiều càng, rộng mặt cắm, cao
SLOT_W = 0.20                                # bề rộng khe cắm càng
BOX_L, BOX_W, BOX_H = 0.15, 0.35, 0.25      # sâu theo chiều càng, mặt quay về xe, cao
DECK_T = 0.005                               # ván mặt mỏng 0,5 cm (người dùng) -> khe cao 4,0 cm
# 3 pallet + 3 thùng (người dùng có 3 bộ): cả dãy phải là chỗ lấy, dãy trái và hai ô
# cuối kho để trống làm chỗ thả.
OCCUPIED = {('R', 0.8), ('R', 1.6), ('R', 2.4)}

def mat(r, g, b):
    return f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse><specular>0.05 0.05 0.05 1</specular></material>'

def box(name, pose, size, color, collide=True):
    s = ' '.join(f'{v:g}' for v in size); p = ' '.join(f'{v:.4f}' for v in pose)
    col = f'<collision name="c"><geometry><box><size>{s}</size></box></geometry></collision>' if collide else ''
    return (f'    <model name="{name}"><static>true</static><pose>{p}</pose><link name="link">{col}'
            f'<visual name="v"><geometry><box><size>{s}</size></box></geometry>{mat(*color)}</visual></link></model>\n')

BRICK, MORTAR = (0.62, 0.24, 0.17), (0.78, 0.70, 0.62)
YELLOW, ORANGE = (0.98, 0.85, 0.10), (0.98, 0.50, 0.05)
parts = []

# Tường gạch: thân đỏ gạch + các vạch vữa nằm ngang (chỉ hình, không va chạm).
for name, x, y, sx, sy in [('wall_n', LX/2+T/2, 0, T, LY+2*T), ('wall_s', -LX/2-T/2, 0, T, LY+2*T),
                           ('wall_e', 0, LY/2+T/2, LX, T), ('wall_w', 0, -LY/2-T/2, LX, T)]:
    parts.append(box(name, (x, y, H/2, 0, 0, 0), (sx, sy, H), BRICK))
    for k in range(1, 8):
        z = k*H/8; inset = 0.003
        sxm, sym = (sx+2*inset, sy+2*inset)
        parts.append(box(f'{name}_mortar{k}', (x, y, z, 0, 0, 0), (sxm, sym, 0.012), MORTAR, collide=False))

# Vạch sơn trên sàn (không va chạm, LiDAR không thấy): dày 2 mm, nổi 1 mm.
def line(name, x0, y0, x1, y1, w, color):
    L = math.hypot(x1-x0, y1-y0); a = math.atan2(y1-y0, x1-x0)
    parts.append(box(name, ((x0+x1)/2, (y0+y1)/2, 0.001, 0, 0, a), (L, w, 0.002), color, collide=False))

def outline(name, cx, cy, sx, sy, w, color):
    line(f'{name}_a', cx-sx/2, cy-sy/2, cx+sx/2, cy-sy/2, w, color)
    line(f'{name}_b', cx-sx/2, cy+sy/2, cx+sx/2, cy+sy/2, w, color)
    line(f'{name}_c', cx-sx/2, cy-sy/2, cx-sx/2, cy+sy/2, w, color)
    line(f'{name}_d', cx+sx/2, cy-sy/2, cx+sx/2, cy+sy/2, w, color)

# Làn cam: hai mép lối đi, đoạn giữa (x -1.4..-0.2) bo cong phình ra ngoài như ảnh mẫu.
def lane(side):
    s = 1 if side == 'L' else -1; y0, y1 = 1.05*s, 1.30*s
    pts = [(-2.75, y0), (-1.6, y0)]
    for i in range(1, 9):                     # cung lồi ra ngoài bằng 8 đoạn
        t = i/8; pts.append((-1.6+0.4*t, y0+(y1-y0)*math.sin(t*math.pi/2)))
    pts += [(-0.4, y1)]
    for i in range(1, 9):
        t = i/8; pts.append((-0.4+0.4*t, y1-(y1-y0)*(1-math.cos(t*math.pi/2))))
    pts += [(2.75, y0)]
    for i in range(len(pts)-1):
        line(f'lane_{side}{i}', *pts[i], *pts[i+1], 0.05, ORANGE)
lane('L'); lane('R')
line('lane_end_e', 2.75, -1.05, 2.75, 1.05, 0.05, ORANGE)
line('lane_end_w', -2.75, -1.05, -2.75, 1.05, 0.05, ORANGE)

# Ô pallet viền vàng bao thùng 0,25 x 0,35 m, chừa ~5 cm mỗi phía.
for side, s in (('L', 1), ('R', -1)):
    for x in BAY_X + DROP_X:
        outline(f'bay_{side}_{x:+.1f}'.replace('.', 'p').replace('+', 'p').replace('-', 'm'),
                x, s*(FACE_Y+BOX_L/2), BOX_W+0.10, BOX_L+0.10, 0.03, YELLOW)

# Gốc model = mặt trước thùng (mặt LiDAR thấy), càng đi vào theo +x cục bộ như p6.sdf.
# Thùng dài hơn pallet nên chìa ra (BOX_L-PAL_L)/2 mỗi đầu. Pallet có hai thành bên dọc
# theo chiều càng, giữa là một khe SLOT_W: cả hai càng cùng lọt vào khe, dưới ván mặt.
def pallet(name, x, y, yaw):
    M = 0.3                                  # khối lượng pallet + thùng (kg, SIM)
    off = (BOX_L-PAL_L)/2
    def comp(n, pose, size, color):
        s = ' '.join(f'{v:g}' for v in size); p = ' '.join(f'{v:g}' for v in pose)
        return (f'<collision name="{n}_collision"><pose>{p}</pose><geometry><box><size>{s}</size></box></geometry>'
                f'<surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision>'
                f'<visual name="{n}_visual"><pose>{p}</pose><geometry><box><size>{s}</size></box></geometry>{mat(*color)}</visual>')
    body = comp('deck', (off+PAL_L/2, 0, PAL_H-DECK_T/2, 0, 0, 0), (PAL_L, PAL_W, DECK_T), (0.55, 0.32, 0.12))
    side = (PAL_W-SLOT_W)/2
    for i, sy in enumerate((-1, 1)):
        body += comp(f'side{i}', (off+PAL_L/2, sy*(SLOT_W+side)/2, (PAL_H-DECK_T)/2, 0, 0, 0),
                     (PAL_L, side, PAL_H-DECK_T), (0.45, 0.25, 0.10))
    body += comp('parcel', (BOX_L/2, 0, PAL_H+BOX_H/2, 0, 0, 0), (BOX_L, BOX_W, BOX_H), (0.72, 0.52, 0.30))
    return (f'    <model name="{name}"><pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.6f}</pose><link name="load">'
            f'<inertial><pose>{BOX_L/2:g} 0 {PAL_H+BOX_H/2:g} 0 0 0</pose><mass>{M}</mass><inertia><ixx>{M/12*(BOX_W**2+BOX_H**2):.5f}</ixx>'
            f'<iyy>{M/12*(BOX_L**2+BOX_H**2):.5f}</iyy><izz>{M/12*(BOX_L**2+BOX_W**2):.5f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>{body}</link></model>\n')

for side, x in sorted(OCCUPIED):
    s = 1 if side == 'L' else -1
    parts.append(pallet(f'pallet_{side}_{int(x*10)}', x, s*FACE_Y, s*math.pi/2))

header = f'''<?xml version='1.0' encoding='utf-8'?>
<!-- Kho mô phỏng "bays": lòng kho {LX:g} x {LY:g} m, tường gạch, ô pallet viền vàng hai bên,
     lối đi giữa có vạch làn cam (vạch chỉ để nhìn, không va chạm). Pallet mặt 20,5 x sâu 16 x cao 4,5 cm, khe càng 20 cm + thùng dựng đứng mặt 35 x sâu 15 x cao 25 cm.
     Xe sinh ở gốc, hướng +x. Sinh bằng script; bản đồ tĩnh: forklift_navigation/maps/bays.yaml
     (chỉ có tường, pallet là vật cản LiDAR thấy khi chạy). Chỉ dùng cho SIM. -->
<sdf version="1.6">
  <world name="forklift">
    <physics type="ignored"><real_time_update_rate>1000</real_time_update_rate><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <scene><ambient>0.45 0.45 0.45 1</ambient><background>0.6 0.6 0.65 1</background><shadows>true</shadows></scene>
    <light name="sun" type="directional"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>1 1 1 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.1 -0.9</direction></light>
    <model name="ground_plane"><static>true</static><pose>0 0 -0.005 0 0 0</pose><link name="link">
      <collision name="collision"><geometry><box><size>100 100 0.01</size></box></geometry><surface><friction><ode><mu>0.5</mu><mu2>0.5</mu2></ode></friction></surface></collision>
      <visual name="visual"><geometry><box><size>100 100 0.01</size></box></geometry>{mat(0.62, 0.63, 0.64)}</visual></link></model>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics" />
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands" />
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster" />
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu" />
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
'''
world_out.write_text(header + ''.join(parts) + '  </world>\n</sdf>\n')

# Bản đồ tĩnh 4 cm: chỉ tường (cùng quy ước map_server trinary: 0 = chiếm, 254 = trống, 205 = chưa biết).
res, margin = 0.04, 0.2
ox, oy = -LX/2-T-margin, -LY/2-T-margin
W, Hh = int(round((LX+2*T+2*margin)/res)), int(round((LY+2*T+2*margin)/res))
img = []
for j in range(Hh):
    y = oy+(Hh-1-j+0.5)*res; row = []
    for i in range(W):
        x = ox+(i+0.5)*res
        inside = abs(x) <= LX/2+T and abs(y) <= LY/2+T
        wall = inside and (abs(x) >= LX/2 or abs(y) >= LY/2)
        row.append(0 if wall else (254 if inside else 205))
    img.append(bytes(row))
(map_dir/'bays.pgm').write_bytes(f'P5\n{W} {Hh}\n255\n'.encode()+b''.join(img))
(map_dir/'bays.yaml').write_text(f'image: bays.pgm\nmode: trinary\nresolution: {res}\norigin: [{ox:g}, {oy:g}, 0]\n'
                                 'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
print(world_out, len(parts), 'model;', 'map', W, 'x', Hh)
