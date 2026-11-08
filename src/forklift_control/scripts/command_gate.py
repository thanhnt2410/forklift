#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Select/limit motion commands. Wall-clock expiry remains valid if SIM pauses."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

import scan_safety


class CommandGate(Node):
    def __init__(self):
        super().__init__('command_gate')
        defaults = dict(command_timeout=.3, state_timeout=.5, max_vx=.25,
                        max_vy=.2, max_wz=.8, max_ax=.5, max_ay=.5, max_aw=1.5,
                        max_wheel_speed=12., wheel_radius=.05,
                        kinematic_sum=.2275, transport_height=.02,
                        # Trần tốc độ bánh riêng khi docking (0 = tắt, dùng max_wheel_speed).
                        dock_max_wheel_speed=0.)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: float(self.get_parameter(key).value) for key in defaults}
        # dock_max_wheel_speed được phép bằng 0 (= tắt); các giới hạn còn lại phải dương.
        if not all(math.isfinite(v) and v > 0 for k, v in self.p.items() if k != 'dock_max_wheel_speed'):
            raise ValueError('All gate limits must be finite and positive')
        if not (math.isfinite(self.p['dock_max_wheel_speed']) and self.p['dock_max_wheel_speed'] >= 0):
            raise ValueError('dock_max_wheel_speed must be finite and >= 0')
        self.sources = {}
        self.declare_parameter('autonomous_enabled', False)
        self.autonomous = bool(self.get_parameter('autonomous_enabled').value)
        self.declare_parameter('require_navigation_ready', False)
        self.require_navigation_ready = bool(self.get_parameter('require_navigation_ready').value)
        # false = bánh cao su skid-steer (wheel_type:=rubber): xe không đi ngang được,
        # mọi vy bị bỏ ngay tại đầu vào. Bao tốc độ bánh giữ nguyên công thức với
        # vy = 0 và kinematic_sum = nửa vết bánh hiệu dụng.
        self.declare_parameter('holonomic', True)
        self.holonomic = bool(self.get_parameter('holonomic').value)
        self.last_strafe_warning = -math.inf
        self.declare_parameter('enforce_transport_height', True)
        self.enforce_transport_height = bool(self.get_parameter('enforce_transport_height').value)
        self.last_height_warning = -math.inf
        # Giữ hướng bằng gyro (xe thật, 30/10/2026): PI từng bánh chỉ bám encoder
        # của chính bánh đó, không tầng nào sửa hướng thân xe. Đi thẳng 6 rad/s
        # bên phải hụt ~15 % -> xe quay trái ~5°/s. Lệnh có tịnh tiến mà wz = 0
        # thì chốt góc gyro tích phân và thêm wz = kp·sai số. Gyro thay vì encoder
        # để thấy cả phần bánh trượt. SIM không bật (không có /imu/data_raw).
        self.declare_parameter('heading_hold', False)
        self.heading_hold = bool(self.get_parameter('heading_hold').value)
        hold_defaults = dict(heading_hold_kp=1.5, heading_hold_max_wz=.25,
                             heading_hold_min_speed=.03, imu_timeout=.2)
        for key, value in hold_defaults.items():
            self.declare_parameter(key, value)
            self.p[key] = float(self.get_parameter(key).value)
            if not (math.isfinite(self.p[key]) and self.p[key] > 0):
                raise ValueError(f'{key} must be finite and positive')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.yaw = 0.
        self.yaw_rate = 0.
        self.imu_stamp = None
        self.imu_time = 0.
        self.hold_ref = None
        self.last_turn = -math.inf
        self.navigation_ready = False
        self.navigation_stamp = 0.
        self.estop = False
        self.state = None
        self.lift_busy = True
        self.lift_stamp = 0.
        self.output = [0., 0., 0.]
        self.last_tick = time.monotonic()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.stop_pub = self.create_publisher(Bool, '/safety/estop', latched)
        self.stop_pub.publish(Bool(data=False))
        # Chế độ hiện tại (latched) để task_manager từ chối mission khi gate đang
        # thủ công: lúc đó lệnh nav/docking bị bỏ, xe đứng im tới DOCK_TIMEOUT.
        self.mode_pub = self.create_publisher(Bool, '~/autonomous', latched)
        self.mode_pub.publish(Bool(data=self.autonomous))
        for name in ('teleop', 'nav'):
            self.create_subscription(Twist, f'/cmd_vel_{name}',
                                     lambda m, n=name: self.receive(n, m), 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_subscription(Bool, '/lift/moving', self.lift, latched)
        self.create_subscription(Bool, '/navigation/ready', self.navigation, 10)
        if self.heading_hold:
            self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.imu,
                                     qos_profile_sensor_data)
        # Lớp an toàn LiDAR (01/11/2026): dự báo va chạm của lệnh đã chọn trong vài
        # giây tới; tự động thì GIẢM TỐC rồi DỪNG, teleop chỉ DỪNG (người dùng chọn).
        # Chạy trong gate (trên Pi ở xe thật) để không phụ thuộc trễ Wi-Fi của Nav2.
        self.declare_parameter('scan_safety', True)
        self.safety_enabled = bool(self.get_parameter('scan_safety').value)
        safety_defaults = dict(
            laser_x=0., laser_y=0., laser_yaw=0.,
            # Footprint khớp robot.yaml; khi chở hàng nới tới loaded_* để bỏ thùng/pallet.
            footprint_xmin=-.1751, footprint_xmax=.4513, footprint_halfwidth=.225145,
            loaded_xmax=.52, loaded_halfwidth=.20,
            margin=.05, stop_time=.4, stop_distance=.06, slow_time=1.5, min_scale=.3,
            step=.1, max_horizon=4., keep_radius=2.5, scan_timeout=.5,
            # Hành lang pallet khi cắm càng. 02/11/2026: 0,20 -> 0,26 m. Xe thật lệch
            # ngang ~5 cm lúc ALIGN (bánh cao su không đi ngang) nên mép pallet rộng
            # 0,34 m nằm ở y ≈ 0,22, lọt ra ngoài hành lang -> SCAN_SAFETY STOP d=0,04 m
            # suốt 90 s -> DOCK_TIMEOUT. 0,26 = nửa pallet 0,17 + lệch 0,06 + biên 0,03.
            # Đổi lại: dải 3,5 cm sát hông nửa trước thân xe (0,225..0,26) không được xét
            # khi docking; người/vật ở y ≥ 0,26 vẫn dừng xe.
            dock_corridor_halfwidth=.26, creep_speed=.08, creep_turn=.15)
        self.s = {}
        for key, value in safety_defaults.items():
            self.declare_parameter(key, value)
            self.s[key] = float(self.get_parameter(key).value)
            if not math.isfinite(self.s[key]):
                raise ValueError(f'{key} must be finite')
        if not (0 < self.s['stop_time'] < self.s['slow_time'] and 0 < self.s['min_scale'] <= 1
                and self.s['step'] > 0 and self.s['margin'] >= 0):
            raise ValueError('Invalid scan safety timing/scale parameters')
        self.scan = []
        self.scan_time = -math.inf
        self.mission_loaded = False
        self.motion_source = ''
        self.motion_source_time = -math.inf
        self.last_safety_kind = ''
        self.last_safety_pub = -math.inf
        self.last_safety_warning = -math.inf
        self.safety_pub = self.create_publisher(String, '/safety/scan_status', 10)
        if self.safety_enabled:
            for topic in ('/scan', '/scan_raw'):
                self.create_subscription(LaserScan, topic, self.laser, qos_profile_sensor_data)
            self.create_subscription(Bool, '/mission/loaded',
                                     lambda m: setattr(self, 'mission_loaded', m.data), latched)
        # Nguồn chuyển động (NAV2/DOCKING) cũng quyết định trần tốc độ bánh,
        # nên nghe luôn cả khi scan_safety tắt.
        self.create_subscription(String, '/mission_motion/source', self.motion, 10)
        self.create_service(SetBool, '~/estop', self.stop)
        self.create_service(SetBool, '~/set_autonomous', self.mode)
        self.create_timer(.02, self.tick, clock=rclpy.clock.Clock())

    def receive(self, source, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        unsupported = (msg.linear.z, msg.angular.x, msg.angular.y)
        if not all(math.isfinite(v) for v in values + unsupported) or any(unsupported):
            # Invalid input from the selected publisher is an explicit stop.
            self.sources[source] = ([0., 0., 0.], time.monotonic())
            return
        if not self.holonomic and values[1]:
            now = time.monotonic()
            if now-self.last_strafe_warning >= 5.:
                self.get_logger().warning(f'STRAFE_IGNORED: vy={values[1]:.3f} from {source}; '
                                          'skid-steer wheels cannot move sideways')
                self.last_strafe_warning = now
            values = (values[0], 0., values[2])
        if not self.estop:
            self.sources[source] = (values, time.monotonic())

    def joints(self, msg):
        if 'lift_joint' in msg.name:
            i = msg.name.index('lift_joint')
            if i < len(msg.position) and i < len(msg.velocity):
                pos, vel = msg.position[i], msg.velocity[i]
                if math.isfinite(pos) and math.isfinite(vel):
                    self.state = (pos, vel, time.monotonic())

    def lift(self, msg):
        self.lift_busy = msg.data
        self.lift_stamp = time.monotonic()

    def wheel_limit(self, now):
        # Đang docking (tin còn tươi <0,5 s) thì dùng trần thấp hơn nếu được cấu hình;
        # mất tin thì tự về trần thường, không kẹt ở mức thấp.
        limit = self.p['max_wheel_speed']
        dock = self.p['dock_max_wheel_speed']
        if dock > 0. and self.motion_source == 'DOCKING' and now-self.motion_source_time < .5:
            limit = min(limit, dock)
        return limit

    def motion(self, msg):
        self.motion_source = msg.data
        self.motion_source_time = time.monotonic()

    def imu(self, msg):
        rate = msg.angular_velocity.z
        if not math.isfinite(rate):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        # Tích phân theo stamp của driver (cùng máy Pi). Khoảng hở > 0,2 s (mất
        # gói) không tích phân bù; tick() sẽ thấy IMU cũ và nhả giữ hướng.
        if self.imu_stamp is not None and 0. < stamp-self.imu_stamp <= .2:
            self.yaw += .5*(rate+self.yaw_rate)*(stamp-self.imu_stamp)
        self.imu_stamp = stamp
        self.yaw_rate = rate
        self.imu_time = time.monotonic()

    def hold_heading(self, command, now):
        """Thêm wz giữ hướng khi lệnh tịnh tiến không kèm quay; ngoài ra nhả chốt."""
        vx, vy, wz = command
        if abs(wz) > 1e-3:
            self.last_turn = now
        if (not self.heading_hold or abs(wz) > 1e-3
                or math.hypot(vx, vy) < self.p['heading_hold_min_speed']
                or now-self.imu_time > self.p['imu_timeout']):
            self.hold_ref = None
            return command
        if self.hold_ref is None:
            self.hold_ref = self.yaw
        # Vừa nhả phím quay: thân xe còn trôi theo ramp. Chốt theo góc hiện tại tới
        # khi gyro gần đứng (tối đa 0,5 s), nếu không sẽ kéo ngược về góc lúc nhả.
        # Chỉ sau lệnh quay: đi thẳng từ đứng yên mà lệch ngay thì phải sửa luôn.
        if now-self.last_turn < .5 and abs(self.yaw_rate) > .05:
            self.hold_ref = self.yaw
        error = self.hold_ref-self.yaw
        limit = self.p['heading_hold_max_wz']
        return [vx, vy, max(-limit, min(limit, self.p['heading_hold_kp']*error))]

    def navigation(self, msg):
        self.navigation_ready = msg.data
        self.navigation_stamp = time.monotonic()

    def laser(self, msg):
        # Nhận cả /scan và /scan_raw (tùy launch, LiDAR thô có tên khác nhau); giữ
        # khung mới nhất. Gate tự bỏ thân xe/hàng nên không cần bản lọc của scan_relay
        # (chạy trên laptop, đi vòng qua Wi-Fi).
        s = self.s
        reach = s['keep_radius']
        self.scan = scan_safety.scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                            msg.range_min, msg.range_max, s['laser_x'],
                                            s['laser_y'], s['laser_yaw'], reach)
        self.scan_time = time.monotonic()

    def safety_scale(self, target, source, now):
        """Hệ số tốc độ 0..1 theo dự báo va chạm trên LiDAR; 1 khi tắt tính năng."""
        if not self.safety_enabled:
            return 1.
        s = self.s
        autonomous = source == 'nav'
        if now-self.scan_time > s['scan_timeout']:
            # Mất LiDAR: tự động thì dừng (không thấy đường); teleop vẫn cho lái để
            # người vận hành đưa xe ra, chỉ cảnh báo.
            self.safety_status('NO_SCAN', now)
            return 0. if autonomous else 1.
        loaded = self.mission_loaded or (self.state is not None and
                                         self.state[0] > self.p['transport_height'])
        xmax = max(s['footprint_xmax'], s['loaded_xmax']) if loaded else s['footprint_xmax']
        hw = max(s['footprint_halfwidth'], s['loaded_halfwidth']) if loaded else s['footprint_halfwidth']
        points = scan_safety.outside_self(self.scan, s['footprint_xmin'], xmax, hw)
        # Cắm càng: pallet/thùng nằm ngay trước mũi xe là đích chứ không phải vật cản.
        # Docking của mission, hoặc teleop bò chậm, thì chừa hành lang phía trước; hai
        # bên và phía sau vẫn được kiểm tra.
        docking = (autonomous and self.motion_source == 'DOCKING'
                   and now-self.motion_source_time < .5)
        creeping = (source == 'teleop' and abs(target[0]) <= s['creep_speed']
                    and abs(target[2]) <= s['creep_turn'])
        if docking or creeping:
            points = scan_safety.outside_corridor(points, 0., s['dock_corridor_halfwidth'])
        speed = math.hypot(target[0], target[1])
        horizon = min(s['max_horizon'], max(s['slow_time'], s['stop_distance']/max(speed, 1e-3)+s['step']))
        # Chỉ giữ điểm xe có thể chạm trong horizon (Pi: ít điểm thì vòng mô phỏng nhẹ).
        corner = math.hypot(max(abs(s['footprint_xmin']), xmax), hw)
        reach = speed*horizon + corner + s['margin']
        points = [(x, y) for x, y in points if x*x + y*y <= reach*reach]
        contact = scan_safety.first_contact(points, target, s['footprint_xmin'], xmax, hw,
                                            s['margin'], horizon, s['step'])
        # Quay tại chỗ (Spin của Nav2/mission): chỉ DỪNG, không giảm tốc. 02/11/2026 ROTATE_PICK
        # sát tường: SLOW co wz 0,3 rad/s xuống ~0,1 -> bánh cao su skid-steer không đủ mô-men
        # xoay, cả 4 bánh stall, Spin hết 16 s -> ACTION_FAILED_6. Dừng hẳn khi sắp chạm vẫn giữ.
        spinning = speed < .02 and abs(target[2]) > .05
        # Docking (ALIGN/INSERT) cũng chỉ DỪNG, không giảm tốc — cùng lý lẽ như quay tại chỗ.
        # 04/11/2026: ALIGN bò 0,12 m/s tới điểm chuẩn bị, pallet cách mũi footprint 0,135 m ->
        # SLOW hệ số ~0,49 -> lệnh còn 0,059 m/s, dưới mức bánh cao su bứt được ma sát tĩnh
        # (dock_min_linear 0,12 đặt vì xe chỉ chạy được từ ~0,06) -> xe đứng im 78 s, không lỗi,
        # tới khi DOCK_TIMEOUT. Vật cản thật vẫn STOP vì STOP được xét trước SLOW.
        scale, state = scan_safety.decide(contact, autonomous and not spinning and not docking,
                                          s['stop_time'], s['stop_distance'], s['slow_time'],
                                          s['min_scale'])
        detail = '' if contact is None else f' t={contact[0]:.2f}s d={contact[1]:.2f}m'
        self.safety_status(f'{state}{detail}' if state != 'CLEAR' else 'CLEAR', now,
                           warn=state in ('STOP', 'SLOW'), source=source)
        return scale

    def safety_status(self, status, now, warn=False, source=''):
        kind = status.split()[0]
        if kind != self.last_safety_kind or now-self.last_safety_pub >= .5:
            self.safety_pub.publish(String(data=status))
            self.last_safety_pub = now
        if warn and (kind != self.last_safety_kind or now-self.last_safety_warning >= 2.):
            self.get_logger().warning(f'SCAN_SAFETY {status} ({source}): obstacle in predicted path')
            self.last_safety_warning = now
        self.last_safety_kind = kind

    def stop(self, req, res):
        self.estop = req.data
        self.sources.clear()
        self.output = [0., 0., 0.]
        self.stop_pub.publish(Bool(data=self.estop))
        self.pub.publish(Twist())
        res.success = True
        res.message = 'Stopped' if self.estop else 'Released; send a fresh command'
        return res

    def mode(self, req, res):
        self.autonomous = req.data
        self.mode_pub.publish(Bool(data=self.autonomous))
        self.sources.clear()
        self.output = [0., 0., 0.]
        self.pub.publish(Twist())
        res.success = True
        res.message = 'Autonomous with teleop priority' if req.data else 'Manual only'
        return res

    def tick(self):
        now = time.monotonic()
        dt = min(now-self.last_tick, .1)
        self.last_tick = now
        selected = None
        selected_source = None
        for source in (('teleop', 'nav') if self.autonomous else ('teleop',)):
            if source in self.sources and now-self.sources[source][1] <= self.p['command_timeout']:
                selected = self.sources[source][0]
                selected_source = source
                break
        allowed = (not self.estop and not self.lift_busy and self.state is not None
                   and now-self.lift_stamp <= self.p['state_timeout']
                   and now-self.state[2] <= self.p['state_timeout']
                   and abs(self.state[1]) < .002)
        if self.state is not None and self.state[0] > self.p['transport_height']:
            if self.enforce_transport_height:
                allowed = False
            elif now-self.last_height_warning >= 5.:
                self.get_logger().warning(
                    f'LIFT_HEIGHT_WARNING: height={self.state[0]:.3f} m exceeds '
                    f'{self.p["transport_height"]:.3f} m; driving remains enabled')
                self.last_height_warning = now
        if selected_source == 'nav' and self.require_navigation_ready:
            # Sensor/localization interlocks bypass the normal command ramp.
            allowed = allowed and self.navigation_ready and now-self.navigation_stamp < .3
        if not allowed or selected is None:
            # Timeout/interlock bypasses the normal acceleration ramp.
            self.output = [0., 0., 0.]
            self.hold_ref = None
        else:
            selected = self.hold_heading(selected, now)
            target = [max(-self.p[k], min(self.p[k], v))
                      for k, v in zip(('max_vx', 'max_vy', 'max_wz'), selected)]
            norm = (abs(target[0])+abs(target[1])+self.p['kinematic_sum']*abs(target[2]))/self.p['wheel_radius']
            scale = min(1., self.wheel_limit(now)/max(norm, 1.e-9))
            target = [v*scale for v in target]
            speed_scale = self.safety_scale(target, selected_source, now)
            if speed_scale <= 0.:
                # Sắp chạm vật: dừng ngay, bỏ qua ramp như các khóa liên động khác.
                self.output = [0., 0., 0.]
                self.hold_ref = None
                out = Twist()
                self.pub.publish(out)
                return
            target = [v*speed_scale for v in target]
            self.output = [old + max(-self.p[a]*dt, min(self.p[a]*dt, new-old))
                           for old, new, a in zip(self.output, target,
                                                 ('max_ax', 'max_ay', 'max_aw'))]
            # Independent axis ramps can otherwise briefly leave the wheel
            # envelope while changing between two valid combined commands.
            norm = (abs(self.output[0])+abs(self.output[1])+
                    self.p['kinematic_sum']*abs(self.output[2]))/self.p['wheel_radius']
            scale = min(1., self.wheel_limit(now)/max(norm, 1.e-9))
            self.output = [v*scale for v in self.output]
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = self.output
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
