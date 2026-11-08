#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Hàng đợi ô pallet chọn trên RViz: ô 1..N lấy hàng, N+1..2N đặt hàng.

Bấm Run là chạy lần lượt từng cặp (ô 1 -> ô N+1, ô 2 -> ô N+2, ...) bằng đúng
nhiệm vụ `pick_and_place` cũ: mỗi chuyến là một goal riêng, goal sau chỉ gửi khi
goal trước báo SUCCEEDED. Xe chỉ mang được một pallet mỗi chuyến nên đây là
chuỗi tuần tự, không phải chở nhiều kiện cùng lúc.

Ngữ nghĩa pose gửi cho task_manager giữ nguyên như bản một cặp A/B:
  - pick: `pick_pose_mode=pallet` thì là mặt pallet, `staging` thì là pose xe đứng;
  - drop: luôn là pose xe lúc hạ hàng, không phải tâm pallet.
Phần "ô" chỉ là hiển thị: tâm ô được suy ra từ pose đó cộng khoảng cách tương
ứng, nên người vận hành thấy pallet sẽ nằm đâu mà hợp đồng action không đổi.
"""
import copy
import json
import math
import os
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from forklift_interfaces.action import RunMission
from forklift_task_manager.dynamic_mission import pose_values, compose

# Mã lỗi của task_manager -> câu giải thích cho người vận hành. Giữ nguyên bản cũ;
# chuỗi nhiều chuyến chỉ thêm số chuyến vào trước câu này.
EXPLANATIONS = {
    'PALLET_NOT_FOUND_AT_A': 'Khong thay mat pallet o o lay: bam dung mat truoc pallet, mui ten = huong cang; kiem tra pallet_face_width',
    'GATE_MANUAL': 'Command gate dang thu cong: ros2 service call /command_gate/set_autonomous std_srvs/srv/SetBool "{data: true}" (dong arrow_teleop)',
    'NO_VALID_PATH_NAV_DROP': 'O dat khong toi duoc: chon cho trong du rong cho xe co tai',
    'NO_VALID_PATH_REFINE_PICK': 'Thay pallet nhung khong co duong vao diem dung truoc no (da thu thang, cong, lui ra, 3 lan)',
    'STAGING_BLOCKED_NAV_PICK': 'Vat can truoc pallet (LiDAR thay lai khi toi noi): don lan cang roi Run',
    'ROUTE_BLOCKED_NAV_PICK': 'Vat can chan duong toi o lay va khong tim duoc duong vong: don duong',
    'ROUTE_BLOCKED_NAV_DROP': 'Vat can chan duong toi o dat va khong tim duoc duong vong: don duong',
    'ROUTE_BLOCKED_REFINE_PICK': 'Vat can chan loi vao pallet: don khu truoc pallet',
}

# Mã task_manager từ chối goal (topic /mission/reject) -> việc người vận hành phải làm.
# Phần "detail" do task_manager gửi kèm đã nói rõ số đo, nên ở đây chỉ là hành động.
REJECT_ACTIONS = {
    'FAULT_NOT_CLEARED': 'Ha cang xuong het (phim F/H) roi bam Run lai',
    'NOT_READY': 'Cho du lieu/navigation san sang (scan, odom, lift, /navigation/ready)',
    'MISSION_ALREADY_RUNNING': 'Doi chuyen dang chay xong hoac bam Stop',
    'PREVIEW_IN_PROGRESS': 'Cho do pallet xong (vai giay) roi Run',
    'PICK_DROP_TOO_CLOSE': 'Chon o dat xa o lay hon',
    'BAD_POSE': 'Bam lai o tren RViz',
    'BAD_MISSION_REQUEST': 'Kiem tra tham so timeout/mission cua mission_input',
    'LOAD_FEEDBACK_STALE': 'Kiem tra nguon /mission/load',
    'MISSION_NOT_SUPPORTED': 'Chi chay pick_and_place tren xe that',
}


def pose_message(values, stamp):
    """[x, y, yaw] -> PoseStamped trong hệ map (dùng khi nạp ô từ file)."""
    message = PoseStamped()
    message.header.frame_id = 'map'
    message.header.stamp = stamp
    message.pose.position.x, message.pose.position.y = float(values[0]), float(values[1])
    message.pose.orientation.z = math.sin(float(values[2]) / 2.)
    message.pose.orientation.w = math.cos(float(values[2]) / 2.)
    return message


class MissionInput(Node):
    def __init__(self):
        super().__init__('mission_input')
        self.picks = []
        self.drops = []
        self.queue = []
        self.trip = 0
        self.pending = False
        self.mission_handle = None
        self.declare_parameter('timeout', 600.)
        # Phải khớp task_manager: pallet = ô lấy bấm lên pallet, staging = pose xe đứng.
        self.declare_parameter('pick_pose_mode', 'staging')
        # Số cặp ô tối đa. 3 = ô 1,2,3 lấy và ô 4,5,6 đặt như quy ước của người vận hành.
        self.declare_parameter('slot_pairs', 3)
        # Chỉ để vẽ ô: khoảng cách từ pose bấm tới tâm pallet tương ứng. Phải lấy cùng
        # giá trị task_manager đang dùng, nếu không khung ô vẽ lệch so với chỗ xe sẽ đặt.
        self.declare_parameter('staging_distance', .65)
        self.declare_parameter('inserted_distance', .28)
        self.declare_parameter('laser_x', 0.)
        self.declare_parameter('slot_length', .30)
        self.declare_parameter('slot_width', .24)
        # Rỗng = ~/.ros/forklift_slots.yaml. Không ghi vào install/ vì symlink-install
        # sẽ sửa thẳng vào source.
        self.declare_parameter('slots_file', '')
        # Cửa sổ coi tin /mission/reject là "của lần Run này" (s).
        self.declare_parameter('reject_grace', 3.)
        self.declare_parameter('autoload_slots', False)
        self.pallet_mode = self.get_parameter('pick_pose_mode').value == 'pallet'
        self.pairs = max(1, int(self.get_parameter('slot_pairs').value))
        self.staging_distance = float(self.get_parameter('staging_distance').value)
        self.inserted_distance = float(self.get_parameter('inserted_distance').value)
        self.laser_x = float(self.get_parameter('laser_x').value)
        self.slot_length = float(self.get_parameter('slot_length').value)
        self.slot_width = float(self.get_parameter('slot_width').value)
        self.reject_grace = max(.5, float(self.get_parameter('reject_grace').value))
        self.slots_file = self.get_parameter('slots_file').value or os.path.join(
            os.path.expanduser('~'), '.ros', 'forklift_slots.yaml')
        self.message = 'Chon %d o lay (1..%d) va %d o dat (%d..%d) tren RViz' % (
            self.pairs, self.pairs, self.pairs, self.pairs + 1, 2 * self.pairs)
        self.client = ActionClient(self, RunMission, '/mission/run', callback_group=ReentrantCallbackGroup())
        self.markers = self.create_publisher(MarkerArray, '/mission/input_markers',
                                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status = self.create_publisher(String, '/mission/input_status', 10)
        for key in ('pick', 'drop'):
            self.create_subscription(PoseStamped, '/mission/' + key + '_pose',
                                     lambda msg, k=key: self.receive(k, msg), 10)
        # task_manager dò pallet ngay khi bấm ô lấy (chế độ pallet) và báo kết quả ở đây.
        self.create_subscription(String, '/mission/pick_preview', self.preview,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Lý do task_manager từ chối goal. Latched nên có thể tới trước hoặc sau phản hồi
        # reject của action; giữ cả hai chiều: dispatch dùng tin vừa nhận, còn tin tới muộn
        # thì ghi đè lại câu hiển thị (trong reject_grace giây).
        self.reject = None
        self.reject_pending_trip = None
        self.reject_pending_at = 0.
        self.create_subscription(String, '/mission/reject', self.rejected,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '/mission/start', self.start)
        self.create_service(Trigger, '/mission/clear', self.clear)
        self.create_service(Trigger, '/mission/save_slots', self.save_slots)
        self.create_service(Trigger, '/mission/load_slots', self.load_slots)
        self.create_timer(.5, self.publish)
        if bool(self.get_parameter('autoload_slots').value):
            self.load_slots(None, Trigger.Response())

    # --- trạng thái chung -------------------------------------------------
    def busy(self):
        return self.pending or self.mission_handle is not None

    def slots(self, key):
        return self.picks if key == 'pick' else self.drops

    def label(self, key, index):
        """Số ô hiển thị: lấy là 1..N, đặt là N+1..2N."""
        return index + 1 if key == 'pick' else self.pairs + index + 1

    def slot_centre(self, key, pose):
        """Tâm pallet suy ra từ pose đã bấm, chỉ dùng để vẽ ô."""
        values = pose_values(pose)
        if key == 'drop':
            return compose(values, [self.inserted_distance + self.laser_x, 0., 0.])
        if self.pallet_mode:
            return values
        return compose(values, [self.staging_distance + self.laser_x, 0., 0.])

    # --- nhận ô từ RViz ---------------------------------------------------
    def receive(self, key, msg):
        if self.busy():
            self.message = 'Dang chay: khong doi o'
            return
        try:
            pose_values(msg)
        except ValueError as e:
            self.message = str(e)
            return
        slots = self.slots(key)
        if len(slots) >= self.pairs:
            self.message = 'Da du %d o %s; bam Clear de chon lai' % (self.pairs, key)
            self.publish()
            return
        slots.append(copy.deepcopy(msg))
        number = self.label(key, len(slots) - 1)
        self.message = ('Da nhan o %d - dang do pallet...' if key == 'pick' and self.pallet_mode
                        else 'Da nhan o %d') % number
        self.publish()

    def rejected(self, msg):
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(info, dict) or 'code' not in info:
            return
        self.reject = (time.monotonic(), info)
        # Goal vua bi tu choi ma luc do chua co ly do: cap nhat lai cau hien thi.
        if (self.reject_pending_trip is not None and
                time.monotonic() - self.reject_pending_at <= self.reject_grace):
            self.message = self.reject_text(self.reject_pending_trip, info)
            self.reject_pending_trip = None
            self.publish()

    def reject_text(self, trip, info):
        code = str(info.get('code', 'MISSION_REJECTED'))
        detail = str(info.get('detail', '')).strip()
        action = REJECT_ACTIONS.get(code, '')
        parts = [p for p in (detail, action) if p]
        return 'Chuyen %d bi tu choi (%s)%s' % (trip, code, ': ' + '. '.join(parts) if parts else '')

    def fresh_reject(self, trip):
        """Câu giải thích nếu task_manager vừa gửi lý do, None nếu chưa thấy."""
        if self.reject is None:
            return None
        at, info = self.reject
        if time.monotonic() - at > self.reject_grace:
            return None
        return self.reject_text(trip, info)

    def preview(self, msg):
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        # Chỉ hiện kết quả cho đúng ô lấy vừa bấm (bỏ kết quả cũ của lần bấm trước).
        if not self.picks or self.busy():
            return
        pick = self.picks[-1]
        if (abs(pick.pose.position.x - info['click'][0]) > 1e-3 or
                abs(pick.pose.position.y - info['click'][1]) > 1e-3):
            return
        number = self.label('pick', len(self.picks) - 1)
        self.message = ('O %d OK - ' % number if info.get('found') else 'O %d FAIL - ' % number) + info.get('message', '')
        self.publish()

    # --- chạy chuỗi -------------------------------------------------------
    async def start(self, req, res):
        if self.busy():
            res.success = False
            res.message = 'MISSION_BUSY'
            return res
        if not self.picks or len(self.picks) != len(self.drops):
            res.success = False
            res.message = 'Can so o lay bang so o dat (hien %d/%d)' % (len(self.picks), len(self.drops))
            self.message = res.message
            return res
        if not self.client.server_is_ready():
            res.success = False
            res.message = 'Mission action server chua san sang'
            self.message = res.message
            return res
        self.queue = list(zip(self.picks, self.drops))
        self.trip = 0
        accepted, reason = await self.dispatch()
        res.success = accepted
        res.message = 'MISSION_ACCEPTED chuyen 1/%d' % len(self.queue) if accepted else reason
        self.message = res.message
        return res

    async def dispatch(self):
        """Gửi goal cho chuyến hiện tại; trả (accepted, lý do)."""
        self.pending = True
        try:
            pick, drop = self.queue[self.trip]
            goal = RunMission.Goal(mission='pick_and_place',
                                   timeout=float(self.get_parameter('timeout').value),
                                   pick_pose=copy.deepcopy(pick), drop_pose=copy.deepcopy(drop))
            handle = await self.client.send_goal_async(goal)
            if not handle.accepted:
                trip = self.trip + 1
                text = self.fresh_reject(trip)
                if text is not None:
                    return False, text
                # Lý do chưa tới (hai luồng DDS khác nhau): hiện tạm, callback sẽ ghi đè.
                self.reject_pending_trip = trip
                self.reject_pending_at = time.monotonic()
                return False, 'Chuyen %d bi tu choi; dang lay ly do tu task_manager...' % trip
            self.mission_handle = handle
            handle.get_result_async().add_done_callback(self.finished)
            return True, ''
        except Exception as e:
            return False, str(e)
        finally:
            self.pending = False

    def finished(self, future):
        self.mission_handle = None
        trip = self.trip + 1
        total = len(self.queue)
        try:
            code = future.result().result.code
        except Exception as e:
            self.message = 'Chuyen %d/%d loi: %s' % (trip, total, e)
            self.publish()
            return
        if code != 'SUCCEEDED':
            self.message = 'Chuyen %d/%d dung (%s): %s' % (trip, total, code, self.explain(code))
            self.publish()
            return
        if trip >= total:
            self.message = 'Hoan tat %d/%d chuyen' % (trip, total)
            self.publish()
            return
        # Chuyến trước đã xong và xe đang đứng tự do (return_to_pick=false thì dừng ngay
        # sau RETREAT_DROP): gửi chuyến kế từ chính chỗ đó, không cần về home.
        self.trip += 1
        self.message = 'Chuyen %d/%d xong, bat dau chuyen %d/%d' % (trip, total, self.trip + 1, total)
        self.publish()
        self.executor.create_task(self.continue_queue())

    async def continue_queue(self):
        accepted, reason = await self.dispatch()
        if not accepted:
            self.message = 'Chuyen %d/%d khong khoi dong duoc: %s' % (self.trip + 1, len(self.queue), reason)
        self.publish()

    def explain(self, code):
        if code.startswith('START_IN_COLLISION_'):
            return ('footprint xe cham o vat can ngay cho dang dung (xem log task_manager); '
                    'lui vai cm khoi tuong hoac dinh vi lai roi Run')
        if code == 'NO_VALID_PATH_NAV_PICK':
            return ('O lay khong toi duoc: khong co loi vao truoc mat pallet' if self.pallet_mode
                    else 'O lay khong toi duoc: lui diem dung ra (~0,65 m truoc mat pallet)')
        return EXPLANATIONS.get(code, code)

    # --- lưu / nạp / xóa --------------------------------------------------
    def clear(self, req, res):
        res.success = not self.busy()
        if res.success:
            self.picks.clear()
            self.drops.clear()
            self.queue = []
            self.trip = 0
            self.message = 'Chon %d o lay (1..%d) va %d o dat (%d..%d) tren RViz' % (
                self.pairs, self.pairs, self.pairs, self.pairs + 1, 2 * self.pairs)
            self.publish()
        res.message = self.message
        return res

    def save_slots(self, req, res):
        data = {key: [pose_values(p) for p in self.slots(key)] for key in ('pick', 'drop')}
        data['pick_pose_mode'] = 'pallet' if self.pallet_mode else 'staging'
        try:
            os.makedirs(os.path.dirname(self.slots_file), exist_ok=True)
            with open(self.slots_file, 'w') as handle:
                yaml.safe_dump(data, handle, default_flow_style=False)
            res.success = True
            res.message = 'Da luu %d o lay, %d o dat vao %s' % (len(data['pick']), len(data['drop']), self.slots_file)
        except OSError as e:
            res.success = False
            res.message = str(e)
        self.message = res.message
        self.publish()
        return res

    def load_slots(self, req, res):
        if self.busy():
            res.success = False
            res.message = 'MISSION_BUSY'
            return res
        try:
            with open(self.slots_file) as handle:
                data = yaml.safe_load(handle) or {}
        except OSError as e:
            res.success = False
            res.message = str(e)
            self.message = res.message
            return res
        # Ô lưu ở chế độ khác thì tâm ô vẽ ra sẽ sai và pose gửi đi cũng khác nghĩa.
        mode = 'pallet' if self.pallet_mode else 'staging'
        if data.get('pick_pose_mode', mode) != mode:
            res.success = False
            res.message = 'File luu o che do %s, node dang chay %s' % (data.get('pick_pose_mode'), mode)
            self.message = res.message
            return res
        stamp = self.get_clock().now().to_msg()
        try:
            loaded = {key: [pose_message(v, stamp) for v in (data.get(key) or [])[:self.pairs]]
                      for key in ('pick', 'drop')}
            for poses in loaded.values():
                for pose in poses:
                    pose_values(pose)
        except (ValueError, TypeError, IndexError) as e:
            res.success = False
            res.message = 'File o khong hop le: %s' % e
            self.message = res.message
            return res
        self.picks, self.drops = loaded['pick'], loaded['drop']
        res.success = True
        res.message = 'Da nap %d o lay, %d o dat tu %s' % (len(self.picks), len(self.drops), self.slots_file)
        self.message = res.message
        self.publish()
        return res

    # --- hiển thị ---------------------------------------------------------
    def slot_markers(self, key, index, pose, markers):
        number = self.label(key, index)
        pick = key == 'pick'
        # id tách theo loại để hai dãy ô không ghi đè nhau khi số ô hai bên lệch.
        base = (index if pick else self.pairs + index) * 3
        colour = (.25, .6, 1.) if pick else (.3, 1., .4)
        arrow = Marker()
        arrow.header = pose.header
        arrow.header.frame_id = 'map'
        arrow.ns = 'mission_inputs'
        arrow.id = base
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = copy.deepcopy(pose.pose)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = .35, .06, .08
        arrow.color.a = 1.
        arrow.color.r, arrow.color.g, arrow.color.b = colour
        markers.markers.append(arrow)
        centre = self.slot_centre(key, pose)
        box = Marker()
        box.header = arrow.header
        box.ns = 'mission_slots'
        box.id = base + 1
        box.type = Marker.LINE_STRIP
        box.action = Marker.ADD
        box.scale.x = .015
        box.color.a = 1.
        box.color.r, box.color.g, box.color.b = colour
        box.pose.orientation.w = 1.
        half_l, half_w = self.slot_length / 2., self.slot_width / 2.
        corners = [(-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w), (-half_l, -half_w)]
        for u, v in corners:
            x, y, _ = compose(centre, [u, v, 0.])
            box.points.append(Point(x=x, y=y, z=.01))
        markers.markers.append(box)
        text = Marker()
        text.header = arrow.header
        text.ns = 'mission_slots'
        text.id = base + 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x, text.pose.position.y, text.pose.position.z = centre[0], centre[1], .35
        text.pose.orientation.w = 1.
        text.scale.z = .16
        text.color.a = 1.
        text.color.r, text.color.g, text.color.b = colour
        text.text = str(number)
        markers.markers.append(text)

    def publish(self):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for key in ('pick', 'drop'):
            for index, pose in enumerate(self.slots(key)):
                try:
                    self.slot_markers(key, index, pose, markers)
                except ValueError:
                    continue
        self.markers.publish(markers)
        self.status.publish(String(data=json.dumps(dict(
            picks=len(self.picks), drops=len(self.drops), pairs=self.pairs,
            trip=self.trip + 1 if self.queue and self.busy() else 0, trips=len(self.queue),
            busy=self.busy(), mode='pallet' if self.pallet_mode else 'staging',
            message=self.message))))


def main():
    rclpy.init()
    node = MissionInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
