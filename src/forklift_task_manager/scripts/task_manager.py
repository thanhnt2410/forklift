#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Pallet missions: legacy SIM route or dynamic A/B with LiDAR refinement.
No Gazebo world pose subscription, teleport or attach/detach in the controller.
Real deployments must provide calibrated lift and external load feedback.
"""
import json
import os
import math
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, PoseStamped, Point, Polygon, Point32
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from nav2_msgs.action import NavigateToPose, Spin, ComputePathToPose, FollowPath
from nav2_msgs.srv import IsPathValid, ClearEntireCostmap
from nav2_msgs.msg import SpeedLimit
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.srv import CancelGoal
# ros_gz chỉ có trên máy chạy Gazebo; xe thật (external/none) không cần contact sensor.
try:from ros_gz_interfaces.msg import Contacts
except ImportError:Contacts=None
from tf2_ros import Buffer, TransformListener
from forklift_interfaces.action import RunMission, MoveLift, HomeLift
from forklift_interfaces.msg import LiftState
from forklift_task_manager.docking import detect_face, clamp, wrap, can_advance
from forklift_task_manager import avoidance as av
from forklift_task_manager.mission_route import (station_poses, straight_samples, load_approved_route,
    DYNAMIC_PLANNERS, path_statistics, choose_navigation_candidate)
from forklift_task_manager.dynamic_mission import pose_values, station_from_approach, compose, inverse, stable_face, face_spread, refined_approach, needs_pick_replan, alignment_samples


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class MissionError(Exception):pass

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.declare_parameter('stations',str(Path(get_package_share_directory('forklift_task_manager'))/'config/stations.yaml'))
        self.p=yaml.safe_load(Path(self.get_parameter('stations').value).read_text())['mission']
        self.g=yaml.safe_load((Path(get_package_share_directory('forklift_description'))/'config/robot.yaml').read_text())
        self.p['laser_x']=self.g['laser_xyz'][0]
        self.declare_parameter('approved_route_file',str(Path(get_package_share_directory('forklift_task_manager'))/'config/approved_route.json'))
        filename=self.get_parameter('approved_route_file').value
        self.approved_file=Path(filename) if filename else None
        self.approved_sha256=''
        self.declare_parameter('scan_topic','/scan_raw')
        self.declare_parameter('load_feedback','contacts')
        self.load_feedback=self.get_parameter('load_feedback').value
        if self.load_feedback not in ('contacts','external','none'):raise ValueError('Invalid load_feedback')
        # none: xe thật chưa có cảm biến tải. Bỏ mọi kiểm tra có/mất/nhả tải; người
        # vận hành phải tự quan sát pallet. Không giả lập /pallet/load_detected.
        if self.load_feedback=='none':
            self.get_logger().warning('load_feedback=none: load presence is NOT verified; operator must watch the pallet')
        # <0: dùng stations.yaml. REAL pulse mode truyền đúng goal preset của bridge
        # (lift_raise_steps/lift_steps_per_m), là mã lệnh nâng chứ chưa phải mét đã đo.
        self.declare_parameter('carry_joint_position',-1.)
        carry=float(self.get_parameter('carry_joint_position').value)
        if carry>=0.:self.p['carry_joint_position']=carry
        if not 0.<=self.p['carry_joint_position']<=self.g['lift_travel']:raise ValueError('Invalid carry_joint_position')
        # Khóa liên động khi xe chạy: càng không được cao hơn mức mang hàng. SIM
        # (carry 0.016) giữ nguyên ngưỡng 0.02 như trước.
        self.carry_limit=max(.02,self.p['carry_joint_position']+.005)
        self.declare_parameter('staging_distance',.65)
        self.declare_parameter('pallet_face_width',.24)
        self.staging_distance=float(self.get_parameter('staging_distance').value)
        self.pallet_width=float(self.get_parameter('pallet_face_width').value)
        # staging: A là pose robot đứng trước pallet (SIM/P6). pallet: A bấm lên
        # pallet, mũi tên = hướng cắm càng; LiDAR tự tìm mặt và tính pose đứng.
        # true (SIM): NAV_DROP phải lập được trước khi xe chạy. false (xe thật, chỗ
        # chật): pallet còn trên sàn chặn đường lúc lập trước; cho phép hoãn tới
        # sau khi nhấc (costmap đã xóa vết pallet). Lỗi khi đó xảy ra lúc đang chở.
        # Xe thật: tốc độ tối thiểu khi docking/lùi và dung sai khoảng cách docking.
        # 0 = giữ hành vi SIM.
        for name in ('dock_min_linear','dock_min_angular','dock_distance_tolerance','dock_yaw_tol','orient_min_yaw','spin_overshoot'):
            self.declare_parameter(name,0.);setattr(self,name,float(self.get_parameter(name).value))
        # spin_overshoot: góc trôi sau Spin (rad), xe thật; 0 = SIM, không bù/quay sửa.
        if not 0.<=self.spin_overshoot<=.5:raise ValueError('spin_overshoot must be in [0, 0.5] rad')
        self.spin_compensation=self.spin_overshoot>0.   # bật theo tham số lúc khởi động, không theo giá trị đã học
        # Dung sai xe thật nới theo pallet tự làm (khe càng rộng); SIM để 0 = mặc định cũ.
        if not (0.<=self.dock_min_linear<=.15 and 0.<=self.dock_min_angular<=.5 and 0.<=self.dock_distance_tolerance<=.05
                and 0.<=self.dock_yaw_tol<=.15 and 0.<=self.orient_min_yaw<=.3):
            raise ValueError('Invalid docking minimums/tolerance')
        # Xe thật: gate thủ công bỏ mọi lệnh nav/docking -> xe đứng im rồi
        # DOCK_TIMEOUT sau 45 s. Kiểm tra ngay từ CHECK_READY. False = SIM.
        self.declare_parameter('require_gate_autonomous',False)
        self.require_gate_autonomous=bool(self.get_parameter('require_gate_autonomous').value)
        # Xe thật: bám pallet theo lần đo LiDAR trước; tốc độ tối thiểu đi ngang
        # riêng (mecanum đi ngang cần mô-men lớn hơn đi thẳng); dung sai ngang ALIGN.
        # False/0 = SIM.
        self.declare_parameter('track_face',False)
        self.track_face=bool(self.get_parameter('track_face').value);self.face_anchor=None
        for name in ('dock_min_lateral','dock_lateral_tol','dock_stop_accel_angular','dock_stop_accel_lateral'):
            self.declare_parameter(name,0.);setattr(self,name,float(self.get_parameter(name).value))
        # Xe thật có IMU + EKF: xung quay dừng theo góc /odom (gyro) thay vì LiDAR.
        self.declare_parameter('pulse_yaw_from_odom',False)
        # Xe thật: căn chỉnh không đi ngang (tiến + bẻ lái, lùi rồi vào lại). False = SIM.
        self.declare_parameter('dock_no_strafe',False)
        # Xe thật: NAV_PICK luôn lập đường cong (lùi ra nếu quá sát), chỉ bỏ khi đã
        # đứng đúng điểm đứng. False = SIM (bỏ NAV_PICK khi near_staging).
        self.declare_parameter('nav_pick_curved',False)
        # Xe thật: NAV_DROP thử planner đoạn thẳng cuối 0,7/0,5 m trước khi quay tại chỗ.
        self.declare_parameter('drop_short_terminal',False)
        self.drop_short_terminal=bool(self.get_parameter('drop_short_terminal').value)
        self.nav_pick_curved=bool(self.get_parameter('nav_pick_curved').value)
        self.dock_no_strafe=bool(self.get_parameter('dock_no_strafe').value)
        # dock_steer (không đi ngang): còn lệch ngang ở điểm chuẩn bị thì lùi thẳng
        # dock_reapproach_distance rồi vào lại vừa tiến vừa bẻ lái. 30/10/2026 bánh cao
        # su, lùi 0,20 m chỉ sửa ~2 mm/lượt (38 -> 32 mm) và hết 45 s -> DOCK_TIMEOUT.
        # Quãng dài hơn cho nhiều đường nhập tâm hơn; thời gian đủ 4-5 lượt dài.
        # Khe pallet 205 mm, càng 140 mm -> hở ~32 mm mỗi bên. Chưa thử lại trên xe.
        self.declare_parameter('dock_reapproach_distance',.35)
        self.declare_parameter('dock_steer_timeout',90.)
        self.dock_reapproach_distance=float(self.get_parameter('dock_reapproach_distance').value)
        self.dock_steer_timeout=float(self.get_parameter('dock_steer_timeout').value)
        # Xe thật: đang gửi lệnh tiến mà khoảng cách tới mặt pallet không giảm quá bao lâu
        # thì coi là không tiến được (gate co tốc độ, bánh trượt, chạm vật). 0 = SIM, bỏ qua.
        self.declare_parameter('dock_no_motion',0.)
        self.dock_no_motion=float(self.get_parameter('dock_no_motion').value)
        if not 0.<=self.dock_no_motion<=10.:raise ValueError('dock_no_motion must be in [0, 10] s')
        # Xe thật: ALIGN chờ odom hết trôi rồi ĐO LẠI mặt pallet trước khi giao cho INSERT.
        # 0 = SIM (xe dừng ngay, giữ hành vi cũ).
        self.declare_parameter('dock_settle',0.)
        self.dock_settle=float(self.get_parameter('dock_settle').value)
        if not 0.<=self.dock_settle<=5.:raise ValueError('dock_settle must be in [0, 5] s')
        if not (.1<=self.dock_reapproach_distance<=.6 and 20.<=self.dock_steer_timeout<=300.):
            raise ValueError('Invalid dock_reapproach_distance/dock_steer_timeout')
        self.pulse_yaw_from_odom=bool(self.get_parameter('pulse_yaw_from_odom').value)
        if not (0.<=self.dock_min_lateral<=.1 and 0.<=self.dock_lateral_tol<=.05
                and 0.<=self.dock_stop_accel_angular<=5. and 0.<=self.dock_stop_accel_lateral<=2.):
            raise ValueError('Invalid dock_min_lateral/dock_lateral_tol')
        # Xe thật: NAV_PICK bị hủy khi đã gần điểm đứng thì chuyển sang ALIGN. False = SIM.
        self.declare_parameter('nav_pick_fallback',False)
        # A/B động: thả xong thì quay về điểm đứng trước A (trống vì pallet đã đi).
        self.declare_parameter('return_to_pick',False)
        self.return_to_pick=bool(self.get_parameter('return_to_pick').value)
        # Xe thật qua Wi-Fi: /scan_raw 4-5 Hz thỉnh thoảng hở > 0,6 s, ready tụt một
        # tin -> FEEDBACK_OR_NAVIGATION_FAULT giữa mission (31/10/2026, 3 lần). Chỉ báo
        # lỗi khi chưa sẵn sàng liên tục quá ready_grace giây. 0 = như cũ (SIM). Khi xe
        # chạy theo Nav2, navigation_guard vẫn tự chốt và cắt lệnh ngay khi mất phản hồi.
        self.declare_parameter('ready_grace',0.)
        # Tránh vật bằng costmap + replan (xe thật). Trong lúc bám đường, kiểm phần đường
        # còn lại với global costmap mỗi 0,5 s theo footprint đúng trạng thái; bị chặn liên
        # tục replan_confirm giây thì dừng, chờ replan_wait (người đi ngang tự hết), vẫn
        # chặn thì lập đường mới từ pose thật vòng qua vật, tối đa replan_attempts lần.
        # false = như cũ (SIM): chỉ dừng khi controller thấy chặn sát rồi báo lỗi.
        # Đường "quay -> đi thẳng -> quay" thử TRƯỚC Reeds-Shepp (xe skid-steer). false = SIM.
        self.declare_parameter('pivot_paths',False)
        self.pivot_paths=bool(self.get_parameter('pivot_paths').value)
        # Quay cuối tối đa tại đích (rad); lớn hơn thì đi hai chặng qua điểm phụ cách đích
        # pivot_approach phía sau (m).
        self.declare_parameter('pivot_final_max',.6)
        self.declare_parameter('pivot_approach',.35)
        self.pivot_final_max=float(self.get_parameter('pivot_final_max').value)
        self.pivot_approach=float(self.get_parameter('pivot_approach').value)
        self.declare_parameter('replan_on_block',False)
        self.declare_parameter('replan_confirm',.6)
        self.declare_parameter('replan_wait',2.5)
        self.declare_parameter('replan_attempts',3)
        self.replan_on_block=bool(self.get_parameter('replan_on_block').value)
        self.replan_confirm=float(self.get_parameter('replan_confirm').value)
        self.replan_wait=float(self.get_parameter('replan_wait').value)
        # Bước 3 né vật (01/11/2026), chỉ có tác dụng khi replan_on_block: dùng nhãn tĩnh/động
        # của obstacle_tracker (/obstacles/tracks). Vật TĨNH chắn đường -> lập đường vòng ngay
        # (không chờ replan_wait). Vật ĐỘNG (người đi) -> đứng nhường tối đa dynamic_wait giây,
        # đường thông thì đi tiếp ĐƯỜNG CŨ; người đứng lại thì coi là tĩnh và vòng. Người sắp
        # cắt đường trong predict_horizon giây -> giảm tốc Nav2 còn slow_percent % qua
        # /speed_limit. Không có dữ liệu tracker thì như trước. Phanh LiDAR ở command_gate
        # vẫn là lớp bảo vệ cuối.
        for key,value in dict(avoidance=True,dynamic_wait=8.,slow_percent=40.,predict_horizon=1.5,
                              avoid_margin=.10,avoid_lookahead=2.5).items():
            self.declare_parameter(key,value)
        self.avoidance=bool(self.get_parameter('avoidance').value)
        self.av={k:float(self.get_parameter(k).value) for k in
                 ('dynamic_wait','slow_percent','predict_horizon','avoid_margin','avoid_lookahead')}
        self.speed_limit_now=0.;self.last_block_kind=None
        self.avoid_log=Path.home()/'.ros'/'forklift_avoidance_events.csv'
        self.replan_attempts=int(self.get_parameter('replan_attempts').value)
        if not (0.<=self.replan_confirm<=3. and 0.<=self.replan_wait<=10. and 1<=self.replan_attempts<=10):
            raise ValueError('Invalid replan_confirm/replan_wait/replan_attempts')
        self.ready_grace=float(self.get_parameter('ready_grace').value)
        if not 0.<=self.ready_grace<=2.:raise ValueError('ready_grace must be within [0, 2] s')
        self.not_ready_since=None
        self.nav_pick_fallback=bool(self.get_parameter('nav_pick_fallback').value)
        self.declare_parameter('require_drop_preflight',True)
        self.require_drop_preflight=bool(self.get_parameter('require_drop_preflight').value)
        # >0 cả hai: nhận mặt pallet khi bề rộng LiDAR đo được nằm trong [min,max]
        # (xe thật). 0: dùng pallet_face_width ± dung sai như SIM.
        self.declare_parameter('pallet_face_min_width',0.)
        self.declare_parameter('pallet_face_max_width',0.)
        lo,hi=(float(self.get_parameter(k).value) for k in ('pallet_face_min_width','pallet_face_max_width'))
        self.width_range=(lo,hi) if lo>0. and hi>0. else None
        if self.width_range and not .05<=lo<hi<=1.5:raise ValueError('Invalid pallet face width range')
        self.previewing=False;self.preview_face=None
        self.declare_parameter('pick_pose_mode','staging')
        self.pick_pose_mode=self.get_parameter('pick_pose_mode').value
        if self.pick_pose_mode not in ('staging','pallet'):raise ValueError('Invalid pick_pose_mode')
        # Xe thật (02/11/2026), vật cản giữa xe và pallet: dò pallet lúc xuất phát chỉ thấy
        # vật (che mặt pallet) -> PALLET_NOT_FOUND_AT_A. true = bỏ dò lúc đầu; điểm ảo X tính
        # thẳng từ A bấm (lùi staging_distance theo mũi tên, X nằm SAU vật, sát pallet), xe đi
        # tới X bằng planner PalletX (Smac thuần, không đoạn thẳng cuối, phạt cong thấp nên
        # vòng gắt quanh vật được), vừa đi vừa replan khi LiDAR thấy thêm mặt bên/sau của vật.
        # Tới X mới dò mặt pallet (DETECT_PICK) và căn chỉnh như cũ. false = như trước.
        self.declare_parameter('pick_via_x',False)
        self.pick_via_x=bool(self.get_parameter('pick_via_x').value)
        self.declare_parameter('initial_detection_timeout',2.)
        self.initial_detection_timeout=float(self.get_parameter('initial_detection_timeout').value)
        if not 0.<=self.initial_detection_timeout<=5.:raise ValueError('Invalid initial_detection_timeout')
        if not .60<=self.staging_distance<=1.2 or not .10<=self.pallet_width<=1.5:raise ValueError('Invalid pallet geometry')
        self.dynamic=False;self.refinement=None;self.initial_detection=None;self.dynamic_segments={}
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.data={};self.stamps={};self.sleepers=[];self.active=False;self.failed=False
        self.state='IDLE';self.code='';self.mission_goal=None;self.client_goal=None
        self.stop_requested=False;self.load_required=False;self.contact_time=0.;self.face=None
        self.contact_debug=[];self.contact_messages=0;self.started=0.;self.deadline=0.;self.events=[];self.stable=None
        self.velocity=self.create_publisher(Twist,'/mission/cmd_vel',10)
        self.status=self.create_publisher(String,'/mission/status',10)
        self.loaded=self.create_publisher(Bool,'/mission/loaded',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for cls,topic,key,qos in [(LaserScan,self.get_parameter('scan_topic').value,'scan',qos_profile_sensor_data),
                (Odometry,'/odom','odom',10),(LiftState,'/lift/state','lift',latched),
                (Bool,'/navigation/ready','ready',10),(Bool,'/safety/estop','estop',latched),
                (Bool,'/command_gate/autonomous','gate',latched)]:
            self.create_subscription(cls,topic,lambda m,k=key:self.receive(k,m),qos)
        if self.load_feedback=='contacts':
            if Contacts is None:raise RuntimeError('load_feedback=contacts needs ros_gz_interfaces (SIM only)')
            self.create_subscription(Contacts,'/p6/contacts',self.contacts,10)
        self.create_subscription(Bool,'/pallet/load_detected',lambda m:self.receive('load',m),10)
        # Costmap chỉ dùng để kiểm tra trước vùng quét khi quay tại chỗ ở NAV_DROP.
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:self.receive('costmap',m),latched)
        # Bước 3 né vật: danh sách vật tĩnh/động và giới hạn tốc độ Nav2 (controller_server).
        self.create_subscription(String,'/obstacles/tracks',lambda m:self.receive('obstacles',m),10)
        self.speed_limit_pub=self.create_publisher(SpeedLimit,'/speed_limit',10)
        self.detected_pose=self.create_publisher(PoseStamped,'/mission/pallet_pose',latched)
        self.corrected_pose=self.create_publisher(PoseStamped,'/mission/refined_pick_pose',latched)
        self.pick_preview=self.create_publisher(String,'/mission/pick_preview',latched)
        # Lý do từ chối goal: action không có chỗ trả thông điệp khi reject, nên đẩy ra
        # topic latched để mission_input/RViz hiện thẳng câu tiếng Việt thay vì bắt người
        # vận hành đi đọc /mission/status và /navigation/status.
        self.reject_info=self.create_publisher(String,'/mission/reject',latched)
        self.create_subscription(PoseStamped,'/mission/pick_pose',self.preview_pick_pose,10)
        self.nav_cancel=self.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
        self.route_segments=[];self.route_revision=0
        self.route_path=self.create_publisher(NavPath,'/mission/route',latched)
        self.route_markers=self.create_publisher(MarkerArray,'/mission/route_markers',latched)
        self.route_info=self.create_publisher(String,'/mission/route_info',latched)
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.active_path=self.create_publisher(NavPath,'/mission/active_path',latched)
        self.footprints=[self.create_publisher(Polygon,'/'+name+'/footprint',10)
                         for name in ('local_costmap','global_costmap')]
        self.follow=ActionClient(self,FollowPath,'/follow_path')
        self.path_valid=self.create_client(IsPathValid,'/is_path_valid')
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.spin_action=ActionClient(self,Spin,'/spin')
        self.clear_costmaps=[self.create_client(ClearEntireCostmap,'/%s/clear_entirely_%s'%(name,name))
                             for name in ('global_costmap','local_costmap')]
        self.lift=ActionClient(self,MoveLift,'/lift/move')
        self.home=ActionClient(self,HomeLift,'/lift/home')
        self.mux=self.create_client(SetBool,'/mission_motion/select_docking')
        self.lift_stop=self.create_client(Trigger,'/lift/stop')
        self.create_service(Trigger,'/mission/stop',self.stop)
        self.create_service(Trigger,'/mission/reset',self.reset)
        self.server=ActionServer(self,RunMission,'/mission/run',execute_callback=self.execute,
            goal_callback=self.accept,cancel_callback=lambda h:CancelResponse.ACCEPT)
        self.create_timer(.02,self.tick,clock=rclpy.clock.Clock())

    def receive(self,key,msg):self.data[key]=msg;self.stamps[key]=time.monotonic()
    def contacts(self,msg):
        self.receive('contacts',msg)
        self.contact_messages+=1
        if msg.contacts:self.contact_debug=[dict(a=c.collision1.name,b=c.collision2.name,aid=c.collision1.id,bid=c.collision2.id) for c in msg.contacts]
        # Names are supplied by Gazebo contact sensor, not a geometric pose oracle.
        if any('forklift' in c.collision1.name+c.collision2.name and
               ('fork_' in c.collision1.name+c.collision2.name or 'lift_carriage' in c.collision1.name+c.collision2.name)
               for c in msg.contacts):self.contact_time=time.monotonic()
    def has_load(self):
        # none: không đo được nên báo không tải; LOWER_RELEASE/reset không bị chặn.
        if getattr(self,'load_feedback','contacts')=='none':return False
        if getattr(self,'load_feedback','contacts')=='external':
            return self.fresh('load') and self.data['load'].data
        return time.monotonic()-self.contact_time<.3
    def fresh(self,key,age=.6):return time.monotonic()-self.stamps.get(key,0.)<age
    def ready(self):
        return (all(self.fresh(k) for k in ['scan','odom','lift','ready']) and self.data['ready'].data
                and not self.data.get('estop',Bool()).data and not self.data['lift'].fault_code)
    def not_ready_reason(self):
        now=time.monotonic();parts=['%s age %.2f s'%(k,now-self.stamps.get(k,0.)) for k in ['scan','odom','lift','ready'] if not self.fresh(k)]
        if 'ready' in self.data and not self.data['ready'].data:parts.append('/navigation/ready=false')
        if self.data.get('estop',Bool()).data:parts.append('estop')
        if 'lift' in self.data and self.data['lift'].fault_code:parts.append('lift fault '+self.data['lift'].fault_code)
        return ', '.join(parts) or 'recovered'
    def accept(self,g):
        reason=self.reject_reason(g)
        if reason is not None:
            code,detail=reason
            self.get_logger().error('Mission rejected: %s (%s)'%(code,detail))
            self.reject_info.publish(String(data=json.dumps(dict(code=code,detail=detail,
                mission=g.mission or 'a_to_b',wall_time=time.time()))))
            return GoalResponse.REJECT
        # Bấm Run là thao tác chủ động của người vận hành: xóa FAULT cũ chỉ khi
        # đạt đúng điều kiện của /mission/reset (sẵn sàng, càng thấp, không tải).
        if self.failed:
            self.get_logger().warning('Previous mission '+(self.code or 'FAULT')+' acknowledged by new Run')
            self.clear_fault()
        self.active=True;self.stop_requested=False;return GoalResponse.ACCEPT
    def reject_reason(self,g):
        # Trả (code, detail) nếu phải từ chối goal, None nếu nhận. Thứ tự kiểm tra giữ
        # nguyên như bản gộp một dòng trước đây; chỉ tách ra để nói được lý do.
        # Đang dò thử A thì chờ xong (vài giây) để không dùng chung bộ lọc mặt pallet.
        if getattr(self,'previewing',False):
            return ('PREVIEW_IN_PROGRESS','dang do mat pallet cho o lay vua bam')
        if self.active:
            return ('MISSION_ALREADY_RUNNING','mission dang chay, buoc %s'%self.state)
        if g.mission not in ('','a_to_b','pick_and_place'):
            return ('BAD_MISSION_REQUEST','mission="%s" khong hop le'%g.mission)
        if not math.isfinite(g.timeout) or not 0<=g.timeout<=900:
            return ('BAD_MISSION_REQUEST','timeout=%s ngoai khoang [0, 900] s'%g.timeout)
        if not self.ready():
            return ('NOT_READY',self.not_ready_reason())
        if g.mission=='pick_and_place':
            try:a,b=pose_values(g.pick_pose),pose_values(g.drop_pose)
            except ValueError as error:return ('BAD_POSE',str(error))
            distance=math.hypot(a[0]-b[0],a[1]-b[1])
            if distance<.30:
                return ('PICK_DROP_TOO_CLOSE','o lay va o dat cach nhau %.2f m, can >= 0,30 m'%distance)
        feedback=getattr(self,'load_feedback','contacts')
        if feedback=='external' and not self.fresh('load'):
            return ('LOAD_FEEDBACK_STALE','/pallet/load_detected cu hon %.2f s'%(time.monotonic()-self.stamps.get('load',0.)))
        # Trạm A/B cố định chỉ có nghĩa trong world SIM; xe thật chỉ chạy pick_and_place.
        if feedback in ('external','none') and g.mission!='pick_and_place':
            return ('MISSION_NOT_SUPPORTED','load_feedback=%s chi chay pick_and_place'%feedback)
        if self.failed and not self.recoverable():
            return ('FAULT_NOT_CLEARED',self.not_recoverable_reason())
        return None
    def not_recoverable_reason(self):
        # recoverable(): khong active, ready(), cang < 0,02 m, khong con tai.
        parts=[]
        if not self.ready():parts.append(self.not_ready_reason())
        lift=self.data.get('lift')
        if lift is None:parts.append('chua nhan /lift/state')
        else:
            height=lift.height+lift.home_offset
            if height>=.02:parts.append('cang dang cao %.3f m, phai ha xuong < 0,020 m'%height)
        if self.has_load():parts.append('cam bien bao con tai tren cang')
        return '%s chua xoa duoc: %s'%(self.code or 'FAULT',', '.join(parts) or 'chua ro')
    def stop(self,req,res):
        self.stop_requested=True;self.command();res.success=True;res.message='Stopping mission; load position held';return res
    def recoverable(self):
        low='lift' in self.data and self.data['lift'].height+self.data['lift'].home_offset<.02
        return not self.active and self.ready() and low and not self.has_load()
    def clear_fault(self):
        self.failed=False;self.code='';self.state='IDLE';self.stop_requested=False
        self.load_required=False;self.loaded.publish(Bool(data=False))
    def reset(self,req,res):
        res.success=self.recoverable()
        if res.success:self.clear_fault()
        res.message='Reset; send a new mission' if res.success else 'Not ready, active or carrying load; recover explicitly';return res
    def check(self):
        if self.stop_requested or (self.mission_goal and self.mission_goal.is_cancel_requested):raise MissionError('CANCELED')
        if time.monotonic()>self.deadline:raise MissionError('MISSION_TIMEOUT')
        if self.ready():self.not_ready_since=None
        else:
            now=time.monotonic()
            if self.not_ready_since is None:self.not_ready_since=now
            if now-self.not_ready_since>=getattr(self,'ready_grace',0.):
                self.get_logger().error('Not ready for %.2f s: %s'%(now-self.not_ready_since,self.not_ready_reason()))
                raise MissionError('FEEDBACK_OR_NAVIGATION_FAULT')
        # Kiểm tra ở CHECK_READY và cả giữa mission (vd. mở arrow_teleop, nó tự xin
        # chế độ thủ công). Topic latched: chưa có tin = gate chưa chạy/khác domain.
        if getattr(self,'require_gate_autonomous',False) and not ('gate' in self.data and self.data['gate'].data):
            raise MissionError('GATE_MANUAL')
        if getattr(self,'load_feedback','contacts')=='external' and not self.fresh('load'):raise MissionError('LOAD_FEEDBACK_STALE')
        if self.load_required and getattr(self,'load_feedback','contacts')!='none' and not self.has_load():raise MissionError('LOAD_CONTACT_LOST')
        if self.state not in ('CHECK_READY','HOME_LIFT','LOWER_FOR_PICK','LIFT_PICK','LOWER_RELEASE'):
            lift=self.data['lift']
            # Ghi rõ cờ nào vi phạm: chỉ mã LIFT_INTERLOCK_CHANGED thì không biết là càng cao,
            # chưa homing, đang chạy, hay bridge/MCU đang DISARMED (inhibited) - bốn nguyên
            # nhân khác hẳn nhau khi chạy xe thật qua Wi-Fi.
            tripped=[name for name,bad in (('moving',lift.moving),('inhibited',lift.inhibited),
                ('not_homed',not lift.homed),
                ('height',lift.height+lift.home_offset>getattr(self,'carry_limit',.02))) if bad]
            if tripped:
                self.get_logger().error('%s: lift interlock [%s]; height=%.4f offset=%.4f limit=%.3f state=%s fault=%s'%(
                    self.state,','.join(tripped),lift.height,lift.home_offset,
                    getattr(self,'carry_limit',.02),lift.state,lift.fault_code or 'none'))
                raise MissionError('LIFT_INTERLOCK_CHANGED')
    async def pause(self,seconds=.05,check=True):
        f=Future();self.sleepers.append((time.monotonic()+seconds,f));await f
        if check:self.check()
    async def wait(self,future,timeout):
        end=time.monotonic()+timeout
        while not future.done():
            await self.pause()
            if time.monotonic()>end:raise MissionError('ACTION_OR_SERVICE_TIMEOUT')
        if future.exception():raise MissionError(str(future.exception()))
        return future.result()
    def stage(self,state):
        self.state=state;self.events.append(dict(state=state,wall_time=time.monotonic()))
        self.get_logger().info('P6 '+state)
    def command(self,vx=0.,vy=0.,wz=0.):
        m=Twist();m.linear.x=float(vx);m.linear.y=float(vy);m.angular.z=float(wz);self.velocity.publish(m)
    def dock_yaw_tolerance(self):
        # Dung sai góc để được tiến khi ALIGN; 0 = 0,025 rad của SIM. Ngưỡng hủy
        # INSERT_ALIGNMENT_LOST của dock_steer nới theo giá trị này.
        return max(.025,getattr(self,'dock_yaw_tol',0.))
    def drive(self,vx=0.,vy=0.,wz=0.):
        # Docking/lùi trên xe thật: bánh không lăn đều dưới ~0,6 rad/s (đo bàn),
        # nên lệnh nhỏ được nâng lên mức tối thiểu, giữ chiều. Lệnh tinh chỉnh
        # rất nhỏ thì về 0 để không giật qua lại quanh đích. 0 = như SIM.
        lin=getattr(self,'dock_min_linear',0.);ang=getattr(self,'dock_min_angular',0.)
        def lift(v,minimum,deadband):
            if minimum<=0.:return v
            if abs(v)<deadband:return 0.
            return math.copysign(max(abs(v),minimum),v)
        # Chỉnh góc nhỏ hơn nửa dung sai góc thì bỏ: nâng lên 0,3 rad/s sẽ lắc qua lại.
        self.command(lift(vx,lin,.003),lift(vy,lin,.003),lift(wz,ang,max(.008,.5*self.dock_yaw_tolerance())))
    async def select(self,docking):
        self.command();r=await self.wait(self.mux.call_async(SetBool.Request(data=docking)),3.)
        if not r.success:raise MissionError('SOURCE_SWITCH_FAILED')
        await self.pause(.3)
    async def action(self,client,goal,timeout):
        if not client.server_is_ready():raise MissionError('ACTION_SERVER_UNAVAILABLE')
        pending=client.send_goal_async(goal)
        # Retain handle even if cancel arrives while goal acknowledgement is pending.
        owner=self.mission_goal
        def acknowledged(f):
            if f.exception() or not f.result() or not f.result().accepted:return
            h=f.result()
            if not self.active or self.stop_requested or self.mission_goal is not owner:
                h.cancel_goal_async()
            else:self.client_goal=h
        pending.add_done_callback(acknowledged)
        h=await self.wait(pending,5.)
        if not h.accepted:raise MissionError('ACTION_REJECTED')
        self.client_goal=h
        result=await self.wait(h.get_result_async(),timeout)
        self.client_goal=None
        if result.status!=4 or (hasattr(result.result,'success') and not result.result.success):
            raise MissionError('ACTION_FAILED_'+str(getattr(result.result,'code',result.status)))
        return result.result
    async def nav_to(self,pose,state):
        self.stage(state)
        # Stop/preempt any external RViz goal while the docking source is held at
        # zero. Do not enable Nav2 with an old goal during a source transition.
        await self.select(True)
        await self.wait(self.nav_cancel.call_async(CancelGoal.Request()),5.)
        await self.pause(.2)
        if self.dynamic:
            if state=='NAV_PICK' and self.dynamic_segments.get('NAV_PICK',{}).get('skip_navigation'):return
            if state!='NAV_PICK':
                await self.update_footprint()
                start=self.current_map_pose()
                if state=='NAV_DROP':
                    if not getattr(self,'require_drop_preflight',True):await self.clear_carried_pallet_marks()
                    segment=await self.compute_drop_segment(start,pose)
                    if segment.get('kind')=='pivot':
                        self.dynamic_segments[state]=segment;await self.start_pivot(state,pose,segment)
                        segment=self.dynamic_segments[state]
                    elif 'pre_rotation' in segment:
                        await self.rotate_in_place(segment['pre_rotation'],'ROTATE_DROP')
                        # Lập lại từ pose thật sau khi quay; không bám đường tính
                        # cho góc lý tưởng nếu Spin dừng lệch vài độ.
                        self.stage(state);start=self.current_map_pose()
                        segment=await self.compute_dynamic_segment(start,pose,state)
                    self.dynamic_segments[state]=segment
                else:
                    pivot=self.pivot_segment(start,pose,state,self.load_required) if getattr(self,'pivot_paths',False) else None
                    if pivot is not None:
                        self.dynamic_segments[state]=pivot;await self.start_pivot(state,pose,pivot)
                    elif state=='REFINE_PICK' and getattr(self,'nav_pick_curved',False):
                        self.dynamic_segments[state]=await self.refine_segment(start,pose)
                    else:self.dynamic_segments[state]=await self.compute_dynamic_segment(start,pose,state)
                self.publish_dynamic_route()
            if state=='NAV_PICK':await self.pick_maneuver()
            if getattr(self,'replan_on_block',False):
                while True:
                    # Đang đi tới điểm tạm trước X (via_x_proxy_segment): không giao ALIGN giữa
                    # chừng, điểm tạm còn cách pallet xa hơn tầm ALIGN.
                    proxy=state=='NAV_PICK' and getattr(self,'via_x_target',None) is not None
                    handover=(lambda current:self.align_reach(current,pose)) if (state=='NAV_PICK'
                        and getattr(self,'nav_pick_fallback',False) and not proxy) else None
                    while True:
                        await self.follow_monitored(state,lambda current:self.replan_segment(state,current,pose),handover)
                        # Đã lấy pose hiện tại làm X, hoặc đã thấy pallet (bỏ X): không đi tiếp
                        # chặng pivot/quay về hướng X cũ.
                        if state=='NAV_PICK' and (getattr(self,'via_x_taken',False) or getattr(self,'pallet_seen',None) is not None):break
                        if not await self.finish_pivot(state,pose):break
                    if not proxy or getattr(self,'pallet_seen',None) is not None or getattr(self,'via_x_taken',False):
                        self.via_x_target=None;return
                    # Tới điểm tạm: thấy pallet thì đi thẳng tới pallet; chưa thấy (bị che)
                    # mới nhìn lại vùng X, chọn X thật rồi đi tiếp vào X.
                    if await self.look_for_pallet(state) is not None:
                        self.via_x_target=None;return
                    pose=await self.recheck_via_x(state)
            try:await self.follow_approved_route(state)
            except MissionError as error:
                # Xe thật: đích NAV_PICK lập trong khung map; AMCL lệch vài cm là
                # footprint ở cuối đường chạm chính pallet -> MPPI "approved reference
                # blocked" (29/10/2026: hở còn 4 cm thay vì 0,19 m). Đã vào vùng
                # near_staging thì giao cho dò LiDAR/ALIGN như khi bỏ qua NAV_PICK.
                if not (state=='NAV_PICK' and getattr(self,'nav_pick_fallback',False)
                        and str(error).startswith('ACTION_FAILED_')):raise
                # 30/10/2026: MPPI hủy ở cuối đường khi quay tại chỗ chỉnh góc chạm pallet
                # ("endpoint heading recovery footprint collision") hoặc đường bị chặn.
                # Trong tầm ALIGN (tiến + bẻ lái tự sửa góc/ngang) thì giao cho ALIGN;
                # xa hơn thì lập lại đường cong từ pose thật, tối đa 2 lần.
                for attempt in range(3):
                    await self.select(True);await self.stationary()
                    current=self.current_map_pose()
                    if self.align_reach(current,pose):
                        self.get_logger().warning('NAV_PICK %s within LiDAR alignment reach; ALIGN takes over'%error);return
                    if attempt==2 or not getattr(self,'nav_pick_curved',False):raise error
                    # Đang giữa đường: chỉ lập lại đường cong/lùi, không thêm quay tại chỗ.
                    segment=await self.curved_pick_segment(current,self.dynamic_pick,rotate=False)
                    if segment is None:raise error
                    self.get_logger().warning('NAV_PICK %s; replanning curved path (attempt %d)'%(error,attempt+1))
                    self.dynamic_segments['NAV_PICK']=segment;self.publish_dynamic_route();self.stage(state)
                    try:await self.follow_approved_route(state);return
                    except MissionError as again:
                        if not str(again).startswith('ACTION_FAILED_'):raise
                        error=again
            return
        if self.approved_file:
            await self.follow_approved_route(state)
            return
        await self.select(False)
        await self.update_footprint()
        g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.header.stamp=self.get_clock().now().to_msg()
        g.pose.pose.position.x=float(pose[0]);g.pose.pose.position.y=float(pose[1]);g.pose.pose.orientation.z=math.sin(pose[2]/2);g.pose.pose.orientation.w=math.cos(pose[2]/2)
        await self.action(self.nav,g,120.);await self.select(True);await self.stationary()
    async def replan_segment(self,state,current,pose):
        # Lập lại đoạn state từ pose thật trên global costmap mới nhất (đã có vật cản).
        # pick_via_x (02/11/2026): tới gần, LiDAR thấy thêm vật sát trước pallet (ô lethal ở góc
        # trước khung xe tại X) -> đường còn lại bị chặn ở chính X, xe đứng ở cửa rồi
        # ROUTE_BLOCKED_NAV_PICK. X là điểm ảo nên lùi X ra chỗ trống rồi mới lập lại; mặt pallet
        # giữ nguyên, dò ở X rồi ALIGN tiến lên.
        if state=='NAV_PICK' and (self.initial_detection or {}).get('outcome')=='SKIPPED':
            moved,back=self.clear_virtual_x(self.dynamic_pick['approach'])
            if back:
                self.get_logger().warning('pick_via_x: X now blocked on costmap; X moved back %.2f m to (%.2f, %.2f)'%(back,moved[0],moved[1]))
                self.dynamic_pick['approach']=moved;self.initial_detection['initial']=list(moved)
                self.corrected_pose.publish(self.map_pose(moved))
        end=self.dynamic_pick['approach'] if state in ('NAV_PICK','RETURN_HOME') else pose
        pivot=self.pivot_segment(current,end,state,self.load_required)
        if pivot is not None:
            self.dynamic_segments[state]=pivot;self.stage(state)
            await self.start_pivot(state,end,pivot);return False
        if state=='NAV_PICK':
            # Giữa đường: chỉ đường cong/lùi thẳng, không quay tại chỗ (pick_maneuver đã chạy).
            segment=await self.curved_pick_segment(current,self.dynamic_pick,rotate=False)
            if segment is None:raise MissionError('NO_VALID_PATH_NAV_PICK')
        elif state=='NAV_DROP':
            segment=await self.compute_drop_segment(current,pose)
            if 'pre_rotation' in segment:
                await self.rotate_in_place(segment['pre_rotation'],'ROTATE_DROP')
                self.stage(state);segment=await self.compute_dynamic_segment(self.current_map_pose(),pose,state)
        elif state=='RETURN_HOME':
            segment=dict(await self.pick_segment_from(current,self.dynamic_pick),stage='RETURN_HOME')
        else:segment=await self.compute_dynamic_segment(current,pose,state)
        self.dynamic_segments[state]=segment;self.publish_dynamic_route();self.stage(state)
        return False
    async def update_footprint(self,loaded=None):
        # The carried pallet extends beyond the forks. After release it is an
        # obstacle, not part of the robot: restore the CAD unloaded footprint.
        xmax=self.g['footprint_xmax'];halfwidth=self.g['footprint_halfwidth']
        # Nhớ footprint planner đang dùng để kiểm điểm đầu đúng hình (start_footprint_clear).
        self.footprint_loaded=bool(self.load_required if loaded is None else loaded)
        if self.footprint_loaded:
            xmax=max(xmax,self.g['loaded_xmax'])
            halfwidth=max(halfwidth,self.g['loaded_halfwidth'])
        polygon=Polygon(points=[Point32(x=float(x),y=float(y)) for x,y in
            [(self.g['footprint_xmin'],-halfwidth),(xmax,-halfwidth),
             (xmax,halfwidth),(self.g['footprint_xmin'],halfwidth)]])
        for publisher in self.footprints:publisher.publish(polygon)
        await self.pause(.3)
    async def prepare_route_path(self,state):
        await TaskManager.update_footprint(self)
        segment=next((s for s in self.route_segments if s['stage']==state),None)
        if segment is None:raise MissionError('PLANNED_SEGMENT_MISSING' if getattr(self,'dynamic',False) else 'APPROVED_SEGMENT_MISSING')
        path=NavPath();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        path.poses=[self.map_pose(p) for p in segment['poses']]
        transform=self.tf.lookup_transform('map','base_link',rclpy.time.Time()).transform
        closest=0;best=float('inf');travelled=0.
        for i,p in enumerate(path.poses):
            if i:travelled+=math.hypot(p.pose.position.x-path.poses[i-1].pose.position.x,p.pose.position.y-path.poses[i-1].pose.position.y)
            if travelled>.25:break
            distance=math.hypot(p.pose.position.x-transform.translation.x,p.pose.position.y-transform.translation.y)
            if distance<best:best=distance;closest=i
        if best>.25:raise MissionError('PLANNED_ROUTE_START_MISMATCH' if self.dynamic else 'APPROVED_ROUTE_START_MISMATCH')
        # Preserve original vertices/tangents: never invent a cusp by anchoring
        # the first vertex to a slightly off-path localization estimate.
        path.poses=path.poses[closest:]
        if not self.path_valid.service_is_ready():raise MissionError('PATH_VALIDATION_UNAVAILABLE')
        if not await self.path_valid_now(state,path,[transform.translation.x,transform.translation.y]):
            raise MissionError('PLANNED_ROUTE_BLOCKED' if self.dynamic else 'APPROVED_ROUTE_BLOCKED')
        return path
    async def path_valid_now(self,state,path,current):
        check=path
        if self.via_x(state) and not self.near_x(current[0],current[1],.05):
            # pick_via_x: còn xa X thì không kiểm vùng quanh X (02/11/2026: IsPathValid báo chặn
            # ở X ngay lúc xuất phát -> 2 lần "NAV_PICK blocked" trước khi xe lăn bánh). Tới gần,
            # drive_monitored/resolve_blocked_x xử lý X.
            check=NavPath();check.header=path.header
            check.poses=[p for p in path.poses if not self.near_x(p.pose.position.x,p.pose.position.y)]
        valid=await self.wait(self.path_valid.call_async(IsPathValid.Request(path=check)),5.)
        if not valid.is_valid:
            # Ghi ô đầu tiên làm IsPathValid từ chối (tâm xe trên ô inscribed/lethal) để biết
            # là vật cản thật hay đường planner đi sát tường; trước đây log chỉ có "blocked".
            cm=self.data.get('costmap');bad=None
            if cm is not None:
                info=cm.info;res=info.resolution;ox,oy=info.origin.position.x,info.origin.position.y
                for p in check.poses:
                    mx,my=int((p.pose.position.x-ox)/res),int((p.pose.position.y-oy)/res)
                    c=cm.data[my*info.width+mx] if 0<=mx<info.width and 0<=my<info.height else -1
                    if not 0<=c<99:bad=(p.pose.position.x,p.pose.position.y,c);break
            self.get_logger().warning('%s: IsPathValid rejected path%s'%(
                state,' at (%.2f, %.2f) cost %d'%bad if bad else ''))
        return valid.is_valid
    def route_checker(self,state):
        if self.dynamic and state=='NAV_PICK':return 'pickup_goal_checker'
        # B động: hạ pallet ngay tại điểm dừng, dung sai rộng hơn (navigation.launch.py).
        if self.dynamic and state in ('NAV_DROP','RETURN_HOME'):return 'drop_goal_checker'
        return 'general_goal_checker'
    async def follow_approved_route(self,state):
        path=await self.prepare_route_path(state)
        self.active_path.publish(path)
        self.get_logger().info('Following '+('planned ' if self.dynamic else 'approved ')+state+' path; sha256='+self.approved_sha256)
        await self.select(False)
        goal=FollowPath.Goal(path=path,controller_id='FollowPath',goal_checker_id=self.route_checker(state))
        await self.action(self.follow,goal,120.)
        await self.select(True);await self.stationary()
    def route_blocked(self,poses,loaded):
        # Cùng quy tắc TerminalApproachPlanner/rotation_clear trên global costmap (planner
        # dùng chính costmap này): tâm < inscribed (99), viền footprint không lethal/unknown.
        # Đường vừa lập luôn đạt, nên "chặn" nghĩa là có vật MỚI trên đoạn còn lại.
        # Không có costmap mới thì không kết luận chặn; controller vẫn tự dừng khi chặn sát.
        if self.data.get('costmap') is None or not self.fresh('costmap',3.):return False
        return not self.footprint_path_clear([(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)) for p in poses],loaded)
    def footprint_path_clear(self,points,loaded,interior=False):
        # points: [(x,y,yaw)]. Không có costmap mới thì coi là KHÔNG trống (dùng khi lập đường).
        cm=self.data.get('costmap')
        if cm is None or not self.fresh('costmap',3.):return False
        last=None
        for x,y,a in points:
            # Lấy mẫu mỗi ~5 cm (ô costmap 4-5 cm) để vòng 0,5 s nhẹ trên Python.
            if last is not None and math.hypot(x-last[0],y-last[1])<.05:continue
            last=(x,y)
            if self.footprint_blocker(cm,(x,y,a),loaded,interior) is not None:return False
        return True
    def footprint_blocker(self,cm,pose,loaded,interior=False):
        # Cùng quy tắc planner (Smac/TerminalApproach): tâm < inscribed (99), viền không
        # lethal (100)/unknown (-1). Trả về (x, y, cost, 'center'|'edge'|'interior') ô chặn đầu
        # tiên, None = trống. interior=True: xét thêm ô lethal LỌT TRONG khung, dùng khi chọn
        # ĐÍCH X (02/11/2026: vật/mép pallet nằm gọn trong khung xe đặt tại X, viền và tâm trống
        # -> X bị coi là trống, không lùi X, mà mọi planner vẫn "approach corridor blocked").
        info=cm.info;W,H,res=info.width,info.height,info.resolution;ox,oy=info.origin.position.x,info.origin.position.y
        def cost(px,py):
            mx,my=int((px-ox)/res),int((py-oy)/res)
            return cm.data[my*W+mx] if 0<=mx<W and 0<=my<H else -1
        x,y,a=pose
        center=cost(x,y)
        if not 0<=center<99:return (x,y,center,'center')
        xmin=self.g['footprint_xmin'];xmax=self.g['footprint_xmax'];hw=self.g['footprint_halfwidth']
        if loaded:xmax=max(xmax,self.g['loaded_xmax']);hw=max(hw,self.g['loaded_halfwidth'])
        # Bước mẫu viền không lớn hơn một ô, kể cả khi bản đồ vẽ lại ở 2 cm.
        step=min(.04,res)
        n=max(2,int(math.ceil((xmax-xmin)/step))+1);m=max(2,int(math.ceil(2*hw/step))+1)
        edge=[(xmin+(xmax-xmin)*i/(n-1),sg*hw) for i in range(n) for sg in (-1,1)]+[(u,-hw+2*hw*j/(m-1)) for j in range(m) for u in (xmin,xmax)]
        c,sn=math.cos(a),math.sin(a)
        for u,v in edge:
            px,py=x+u*c-v*sn,y+u*sn+v*c;value=cost(px,py)
            if not 0<=value<100:return (px,py,value,'edge')
        if interior:
            # Chỉ ô lethal (vật thật/vết đã đánh dấu); unknown bên trong khung thì bỏ qua.
            for i in range(1,n-1):
                for j in range(1,m-1):
                    u=xmin+(xmax-xmin)*i/(n-1);v=-hw+2*hw*j/(m-1)
                    px,py=x+u*c-v*sn,y+u*sn+v*c;value=cost(px,py)
                    if value>=100:return (px,py,value,'interior')
        return None
    async def start_footprint_clear(self,start):
        # 02/11/2026 xe thật: xe đứng cách tường bản đồ 2-3 cm (bản đồ/AMCL lệch 3-5 cm, ô 4 cm)
        # -> mọi planner "Starting point in lethal space" trong 2 s, mission chỉ thấy
        # NO_VALID_PATH_NAV_PICK ("no free approach in front of the pallet face") dù đích trống.
        # Nhiễu X4/AMCL thường tự hết sau vài lần cập nhật costmap (2 Hz): chờ tối đa
        # start_clear_wait; vẫn chạm thì ghi start_blocked để execute báo START_IN_COLLISION.
        loaded=getattr(self,'footprint_loaded',self.load_required)
        end=time.monotonic()+float(self.p.get('start_clear_wait',3.))
        while True:
            cm=self.data.get('costmap')
            # Không có costmap mới: để planner tự quyết như trước.
            if cm is None or not self.fresh('costmap',3.):return True
            blocker=self.footprint_blocker(cm,start,loaded)
            if blocker is None:
                self.start_blocked=None;return True
            if time.monotonic()>=end:break
            await self.pause(.1)
        x,y,value,where=blocker
        self.start_blocked=dict(x=round(x,3),y=round(y,3),cost=value,where=where,loaded=loaded)
        self.get_logger().warning('START_IN_COLLISION: robot footprint (%s) %s touches costmap cell (%.2f, %.2f) cost %d at (%.2f, %.2f, %.0f deg); '
            'drive away from the wall a few cm or relocalize (2D Pose Estimate)'%(
            'loaded' if loaded else 'unloaded',where,x,y,value,start[0],start[1],math.degrees(start[2])))
        return False
    def pivot_segment(self,start,end,state,loaded):
        # Xe skid-steer quay tại chỗ được: QUAY hướng về đích -> ĐI THẲNG -> QUAY đúng hướng đích.
        # Planner Reeds-Shepp (ô tô) trong phòng hẹp hay lùi vào chỗ chật rồi kẹt khi quay
        # ở điểm đảo chiều (31/10/2026: đích ngay trước mặt 0,74 m mà vẫn lùi vào hành lang,
        # "endpoint heading recovery footprint collision"). Kiểm costmap cả vùng quét hai lần
        # quay (rotation_clear) và đoạn thẳng (footprint_path_clear). None = không dùng được.
        dx,dy=end[0]-start[0],end[1]-start[1];d=math.hypot(dx,dy)
        if not .10<=d<=2.5:return None
        options=[]
        for reverse in (False,True):
            # Lùi chỉ khi ngắn: LiDAR vẫn thấy sau nhưng đuôi xe là phía ít kiểm soát hơn.
            if reverse and d>.6:continue
            h=wrap(math.atan2(dy,dx)+(math.pi if reverse else 0.))
            d1=wrap(h-start[2]);d2=wrap(end[2]-h)
            options.append((abs(d1)+abs(d2)+(1. if reverse else 0.),reverse,h,d1,d2))
        for _,reverse,h,d1,d2 in sorted(options):
            # Quay cuối lớn ngay tại đích làm tâm xe trượt (31/10/2026: quay 123° khi chở,
            # tâm xe trôi ~25 cm, thả lệch B 32 cm) -> dùng bản hai chặng bên dưới.
            if abs(d2)>self.pivot_final_max:continue
            if abs(d1)>.06 and not self.rotation_clear(start[0],start[1],start[2],d1,loaded):continue
            line=straight_samples([start[0],start[1],h],[end[0],end[1],h])
            if not self.footprint_path_clear(line,loaded):continue
            if abs(d2)>.06 and not self.rotation_clear(end[0],end[1],h,d2,loaded):continue
            self.get_logger().info('%s: pivot path (rotate %+.0f deg, %s %.2f m, rotate %+.0f deg)'%(
                state,math.degrees(d1),'reverse' if reverse else 'forward',d,math.degrees(d2)))
            return dict(stage=state,kind='pivot',collision_checked=True,reverse=reverse,travel_yaw=h,via=None,
                        pivot_pre=d1,pivot_post=d2,end=list(end),poses=line,statistics=path_statistics(line))
        # Hai chặng: tới điểm phụ P cách đích pivot_approach phía SAU theo hướng đích, quay ở P
        # cho thẳng hàng, rồi đi thẳng vào đích. Chặng cuối lập lại từ pose thật sau khi quay
        # nên phần trượt lúc quay được sửa; ở đích chỉ còn chỉnh vài độ.
        L=self.pivot_approach;P=[end[0]-L*math.cos(end[2]),end[1]-L*math.sin(end[2]),end[2]]
        dpx,dpy=P[0]-start[0],P[1]-start[1];dp=math.hypot(dpx,dpy)
        if dp<.10:return None
        final=straight_samples(P,end)
        if not self.footprint_path_clear(final,loaded):return None
        options=[]
        for reverse in (False,True):
            if reverse and dp>.6:continue
            h=wrap(math.atan2(dpy,dpx)+(math.pi if reverse else 0.))
            d1=wrap(h-start[2]);dm=wrap(end[2]-h)
            options.append((abs(d1)+abs(dm)+(1. if reverse else 0.),reverse,h,d1,dm))
        for _,reverse,h,d1,dm in sorted(options):
            if abs(d1)>.06 and not self.rotation_clear(start[0],start[1],start[2],d1,loaded):continue
            line=straight_samples([start[0],start[1],h],[P[0],P[1],h])
            if not self.footprint_path_clear(line,loaded):continue
            if abs(dm)>.06 and not self.rotation_clear(P[0],P[1],h,dm,loaded):continue
            self.get_logger().info('%s: two-leg pivot (rotate %+.0f deg, %s %.2f m, rotate %+.0f deg, forward %.2f m)'%(
                state,math.degrees(d1),'reverse' if reverse else 'forward',dp,math.degrees(dm),L))
            poses=line+final[1:]
            return dict(stage=state,kind='pivot',collision_checked=True,reverse=reverse,travel_yaw=h,via=P,
                        pivot_pre=d1,pivot_post=dm,end=list(end),poses=poses,statistics=path_statistics(poses))
        return None
    async def start_pivot(self,state,end,segment):
        # Quay tại chỗ (Spin tự kiểm va chạm) rồi lập lại đoạn thẳng từ pose THẬT:
        # Spin dừng lệch vài độ, không bám đường tính cho góc lý tưởng.
        d1=wrap(segment['travel_yaw']-self.current_map_pose()[2])
        if abs(d1)>.06:
            await TaskManager.update_footprint(self)
            try:await self.rotate_in_place(d1,'ROTATE_'+state.split('_')[-1])
            except MissionError as error:
                # Spin kiểm va chạm trên LOCAL costmap (dày hơn global nơi rotation_clear
                # kiểm): bị chặn thì bỏ pivot, lập đường cong từ pose thật như trước khi có pivot.
                if not str(error).startswith('ACTION_FAILED_'):raise
                await self.select(True);await self.stationary();self.stage(state)
                self.get_logger().warning('%s: pivot rotation blocked (%s); using curved path'%(state,error))
                self.dynamic_segments[state]=await self.curved_segment(state,self.current_map_pose(),end)
                self.publish_dynamic_route();return
            self.stage(state)
        cur=self.current_map_pose();goal=segment.get('via') or end
        h=wrap(math.atan2(goal[1]-cur[1],goal[0]-cur[0])+(math.pi if segment['reverse'] else 0.))
        line=straight_samples([cur[0],cur[1],h],[goal[0],goal[1],h])
        self.dynamic_segments[state]=dict(segment,travel_yaw=h,poses=line,statistics=path_statistics(line))
        self.publish_dynamic_route()
    async def finish_pivot(self,state,end):
        # Đoạn pivot kết thúc theo hướng đi thẳng: quay nốt cho đúng hướng đích.
        # True = vừa khởi động chặng cuối (hai chặng), người gọi phải bám đường thêm lần nữa.
        seg=self.dynamic_segments.get(state,{})
        if seg.get('kind')!='pivot':return False
        if seg.get('via') is not None:
            leg=dict(seg,via=None,reverse=False,travel_yaw=end[2])
            self.dynamic_segments[state]=leg;await self.start_pivot(state,end,leg);return True
        d2=wrap(end[2]-self.current_map_pose()[2])
        if abs(d2)>.06:
            await TaskManager.update_footprint(self)
            try:await self.rotate_in_place(d2,'ROTATE_'+state.split('_')[-1])
            except MissionError as error:
                # 31/10/2026: tới B lệch 14°, Spin "Collision Ahead" (hành lang hẹp, đang chở)
                # -> FAULT dù thả lệch vài độ vẫn được. Lệch <= 0,35 rad (~20°): đi tiếp (thả
                # pallet / ALIGN tự sửa góc tới 0,35 rad); lệch hơn mới là lỗi.
                if not str(error).startswith('ACTION_FAILED_'):raise
                await self.select(True);await self.stationary()
                left=wrap(end[2]-self.current_map_pose()[2])
                if abs(left)>.35:raise
                self.get_logger().warning('%s: final rotation blocked (%s); continuing %.0f deg off'%(state,error,math.degrees(left)))
            self.stage(state)
        return False
    async def curved_segment(self,state,current,end):
        # Đường cong Reeds-Shepp (cách cũ trước khi có pivot) cho từng đoạn.
        if state in ('NAV_PICK','RETURN_HOME'):
            return dict(await self.pick_segment_from(current,self.dynamic_pick),stage=state)
        if state=='NAV_DROP':
            try:return await self.compute_dynamic_segment(current,end,'NAV_DROP')
            except MissionError as error:
                if str(error)!='NO_VALID_PATH_NAV_DROP' or not getattr(self,'drop_short_terminal',False):raise
            return await self.compute_dynamic_segment(current,end,'NAV_DROP',('Pallet07','Pallet05'))
        return await self.compute_dynamic_segment(current,end,state)
    def progress_index(self,poses,idx):
        try:x,y,_=self.current_map_pose()
        except Exception:return idx
        best=idx;bd=float('inf')
        for i in range(idx,min(len(poses),idx+80)):
            d=math.hypot(poses[i].pose.position.x-x,poses[i].pose.position.y-y)
            if d<bd:bd=d;best=i
        return best
    def tracked_obstacles(self):
        # None = không có dữ liệu tracker mới (node tắt/mất): dùng hành vi cũ.
        if not self.avoidance or not self.fresh('obstacles',.6):return None
        try:return json.loads(self.data['obstacles'].data).get('obstacles',[])
        except (ValueError,AttributeError):return None
    def avoid_corridor(self):
        hw=self.g['footprint_halfwidth']
        if self.load_required:hw=max(hw,self.g['loaded_halfwidth'])
        return hw+self.av['avoid_margin']
    def path_ahead(self,poses):
        return av.ahead([(p.pose.position.x,p.pose.position.y) for p in poses],self.av['avoid_lookahead'])
    def set_speed_limit(self,percent):
        # 0 = bỏ giới hạn (quy ước nav2_msgs/SpeedLimit). Chỉ gửi khi đổi.
        if percent==self.speed_limit_now:return
        self.speed_limit_now=percent
        self.speed_limit_pub.publish(SpeedLimit(percentage=True,speed_limit=float(percent)))
    def log_avoidance(self,state,kind,action,found=()):
        # Một dòng/sự kiện để làm số liệu báo cáo: thời gian, bước, loại vật, hành động, vật.
        objs=';'.join('#%s %s (%.2f,%.2f) v=%.2f'%(o['id'],o['label'],o['x'],o['y'],o.get('speed',0.)) for o in found)
        self.get_logger().info('AVOID %s: %s -> %s %s'%(state,kind,action,objs))
        try:
            new=not self.avoid_log.exists()
            with open(self.avoid_log,'a') as f:
                if new:f.write('wall_time,state,kind,action,obstacles\n')
                f.write('%.3f,%s,%s,%s,"%s"\n'%(time.time(),state,kind,action,objs))
        except OSError:pass
    async def yield_to_dynamic(self,state,path):
        # Người đang đi chắn đường: đứng yên nhường. True = đường thông, đi tiếp đường cũ;
        # False = hết dynamic_wait hoặc vật đã đứng yên chắn -> người gọi lập đường vòng ngay.
        start=time.monotonic();still=None
        while time.monotonic()-start<self.av['dynamic_wait']:
            await self.pause(.25)
            rest=path.poses[self.progress_index(path.poses,0)+1:]
            if not self.route_blocked(rest,self.load_required):
                self.log_avoidance(state,'dynamic','resume same route after %.1f s'%(time.monotonic()-start))
                return True
            obs=self.tracked_obstacles()
            found=av.blockers(obs or [],self.path_ahead(rest),self.avoid_corridor())
            if av.blocker_kind(found)=='static':
                still=still or time.monotonic()
                if time.monotonic()-still>=1.:
                    self.log_avoidance(state,'static','person stopped on route; detour',found)
                    return False
            else:still=None
        self.log_avoidance(state,'dynamic','waited %.0f s; detour'%self.av['dynamic_wait'])
        return False
    async def drive_monitored(self,state,path):
        try:return await self.drive_monitored_inner(state,path)
        finally:self.set_speed_limit(0.)
    async def drive_monitored_inner(self,state,path):
        # Như follow_approved_route nhưng vừa chạy vừa kiểm đường còn lại với costmap.
        # 'done' = tới đích; 'blocked' = vật mới chắn đường (đã hủy FollowPath);
        # 'failed' = Nav2 tự hủy (vd. controller thấy chặn sát trên local costmap trước).
        self.active_path.publish(path)
        self.get_logger().info('Following planned %s path (costmap monitored)'%state)
        await self.select(False)
        if not self.follow.server_is_ready():raise MissionError('ACTION_SERVER_UNAVAILABLE')
        goal=FollowPath.Goal(path=path,controller_id='FollowPath',goal_checker_id=self.route_checker(state))
        h=await self.wait(self.follow.send_goal_async(goal),5.)
        if not h.accepted:raise MissionError('ACTION_REJECTED')
        # Giữ handle: check() ném lỗi giữa chừng thì nhánh FAULT tự hủy goal này.
        self.client_goal=h;result=h.get_result_async()
        end=time.monotonic()+120.;next_check=0.;since=None;idx=0;loaded=self.load_required
        while not result.done():
            await self.pause(.05)
            now=time.monotonic()
            if now>end:raise MissionError('ACTION_OR_SERVICE_TIMEOUT')
            # pick_via_x: X chỉ là điểm vẽ đường. Vừa chạy vừa dò mỗi scan mới; thấy mặt pallet
            # ổn định thì dừng, bỏ X, để DETECT_PICK/REFINE_PICK lập đường thẳng tới pallet.
            if self.via_x(state) and self.watch_pallet() is not None:
                self.get_logger().warning('%s: pallet face seen at (%.2f, %.2f, %.0f deg) while driving to X; '
                    'skipping X, planning straight to the pallet'%(state,self.pallet_seen[0],self.pallet_seen[1],
                    math.degrees(self.pallet_seen[2])))
                try:await self.wait(h.cancel_goal_async(),3.)
                except MissionError:pass
                self.client_goal=None
                await self.select(True);await self.stationary()
                return 'pallet_seen'
            if now<next_check:continue
            next_check=now+.5
            idx=self.progress_index(path.poses,idx)
            obs=self.tracked_obstacles()
            pts=self.path_ahead(path.poses[idx+1:]) if obs is not None else []
            if obs is not None:
                # Người sắp cắt đường: giảm tốc sớm, trước khi vào sát (phanh LiDAR mới dừng).
                hit=av.predicted_conflict(obs,pts,self.avoid_corridor(),self.av['predict_horizon'])
                if hit and not self.speed_limit_now:
                    self.log_avoidance(state,'dynamic','slow to %.0f%% (crossing in %.1f s)'%(self.av['slow_percent'],hit[0]),[hit[1]])
                self.set_speed_limit(self.av['slow_percent'] if hit else 0.)
            rest=path.poses[idx+1:]
            if self.via_x(state):
                # pick_via_x: vật cản quanh X (điểm ảo) khi còn xa thường là vết nhìn từ xa hoặc
                # vật sát pallet; KHÔNG dừng, đi tiếp tới gần X. Vào vùng X mà vẫn chặn thì dừng
                # để follow_monitored quyết định (lùi X / lấy pose hiện tại làm X / hủy).
                near=[q for q in rest if self.near_x(q.pose.position.x,q.pose.position.y)]
                rest=[q for q in rest if not self.near_x(q.pose.position.x,q.pose.position.y)]
                cur=self.current_map_pose()
                if near and self.near_x(cur[0],cur[1],.05) and self.route_blocked(near,loaded):
                    self.get_logger().warning('%s: reached X zone but X is still blocked on costmap; stopping'%state)
                    try:await self.wait(h.cancel_goal_async(),3.)
                    except MissionError:pass
                    self.client_goal=None
                    return 'x_blocked'
            if self.route_blocked(rest,loaded):
                since=since or now
                if now-since>=self.replan_confirm:
                    found=av.blockers(obs or [],pts,self.avoid_corridor())
                    self.last_block_kind=av.blocker_kind(found) if obs is not None else None
                    self.get_logger().warning('%s: costmap obstacle on remaining route; stopping (%s)'%(
                        state,self.last_block_kind or 'unclassified'))
                    if self.last_block_kind:self.log_avoidance(state,self.last_block_kind,'stop',found)
                    try:await self.wait(h.cancel_goal_async(),3.)
                    except MissionError:pass
                    self.client_goal=None
                    return 'blocked_dynamic' if self.last_block_kind=='dynamic' else 'blocked'
            else:since=None
        self.client_goal=None;r=result.result()
        if r.status==4:
            await self.select(True);await self.stationary();return 'done'
        self.last_follow_error='ACTION_FAILED_'+str(getattr(r.result,'code',r.status))
        return 'failed'
    async def follow_monitored(self,state,replan,handover=None):
        # replan(current): lập đoạn mới cho state vào dynamic_segments + publish_dynamic_route;
        # ném NO_VALID_PATH_* nếu chưa có đường (giữ đường cũ, chờ rồi thử lại).
        # handover(current): True = giao cho bước sau (vd. ALIGN LiDAR khi đã đủ gần pallet).
        events=0;error='ROUTE_BLOCKED_'+state
        while True:
            self.last_block_kind=None
            try:path=await self.prepare_route_path(state)
            except MissionError as e:
                if str(e) not in ('PLANNED_ROUTE_BLOCKED','PLANNED_ROUTE_START_MISMATCH'):raise
                path=None;outcome='blocked' if str(e)=='PLANNED_ROUTE_BLOCKED' else 'failed'
            if path is not None:
                outcome=await self.drive_monitored(state,path)
                if outcome in ('done','pallet_seen'):return
            await self.select(True);await self.stationary()
            # Xóa đường đang bám khỏi RViz ngay khi dừng: đường này đã bị chặn/Nav2 hủy, nếu giữ
            # lại thì suốt thời gian chờ replan (hoặc khi replan chưa ra đường) trông như xe
            # vẫn dùng đường cũ.
            self.active_path.publish(NavPath(header=self.map_pose([0.,0.,0.]).header))
            # pick_via_x: bị chặn/MPPI hủy thì đứng yên dò pallet trước khi cứu X; thấy thì
            # bỏ X, đi thẳng tới pallet (DETECT_PICK/REFINE_PICK) thay vì replan X.
            if self.via_x(state) and outcome in ('x_blocked','failed','blocked') and await self.look_for_pallet(state) is not None:
                return
            # Bước 3: người đang đi chắn -> nhường, đường thông thì đi tiếp đường cũ (không
            # tính vào replan_attempts); người đứng lại/hết giờ chờ -> xử lý như vật tĩnh.
            if outcome=='blocked_dynamic':
                if await self.yield_to_dynamic(state,path):continue
                outcome='blocked';self.last_block_kind='static'
            if self.via_x(state) and outcome in ('x_blocked','failed'):
                # MPPI cũng tự hủy khi đoạn 0,45 m trước mũi chạm ô chặn ở X.
                if await self.resolve_blocked_x(state,outcome):
                    if self.via_x_taken:return
                    continue
                if outcome=='x_blocked':outcome='blocked'
            # Không giao ALIGN ngay: 31/10/2026 người đứng chắn khi xe đã trong tầm ALIGN ->
            # giao luôn, DETECT_PICK bị chân người che -> PALLET_DETECTION_NOT_STABLE. Chờ
            # replan_wait trước (Nav2 không cho biết lý do hủy, chờ cả khi không phải vật cản).
            events+=1
            if outcome=='failed':error=getattr(self,'last_follow_error','ACTION_FAILED')
            if events>self.replan_attempts:
                self.get_logger().error('%s: gave up after %d blocked/failed attempts'%(state,events-1))
                raise MissionError(error if outcome=='failed' else 'ROUTE_BLOCKED_'+state)
            # Chờ trước: người đi ngang thì costmap tự xóa và đi tiếp đường cũ, không vòng.
            # Bước 3: tracker xác nhận vật TĨNH thì không chờ, chỉ để costmap cập nhật 0,3 s.
            wait=.3 if outcome=='blocked' and self.last_block_kind=='static' else self.replan_wait
            if wait<self.replan_wait:self.log_avoidance(state,'static','detour now')
            self.get_logger().warning('%s %s; waiting %.1f s before replanning (%d/%d)'%(
                state,outcome,wait,events,self.replan_attempts))
            await self.pause(wait)
            current=self.current_map_pose()
            if path is not None and outcome=='blocked':
                rest=path.poses[self.progress_index(path.poses,0)+1:]
                if not self.route_blocked(rest,self.load_required):
                    self.get_logger().info('%s: obstacle cleared; resuming the same route'%state);continue
            if handover is not None and handover(current):
                self.get_logger().warning('%s %s within LiDAR alignment reach; ALIGN takes over'%(state,outcome));return
            try:
                if await replan(current):return
                self.get_logger().warning('%s: replanned from current pose (%s)'%(
                    state,'around costmap obstacle' if outcome=='blocked' else 'after Nav2 abort'))
            except MissionError as e:
                if not str(e).startswith('NO_VALID_PATH_'):raise
                self.get_logger().warning('%s: no detour yet (%s); keep waiting'%(state,e))
    def current_map_pose(self):
        t=self.tf.lookup_transform('map','base_link',rclpy.time.Time()).transform
        return [t.translation.x,t.translation.y,yaw(t.rotation)]
    async def compute_dynamic_segment(self,start,end,state,planners=None):
        # Chỉ kiểm điểm đầu là chỗ xe ĐANG đứng; điểm đầu giả định (sau lùi/quay) planner tự
        # loại như cũ. Điểm đầu chạm thì không planner nào lập được: báo ngay, khỏi gọi 7 planner.
        try:here=self.current_map_pose()
        except Exception:here=None
        if (here is not None and math.hypot(start[0]-here[0],start[1]-here[1])<.03
                and abs(wrap(start[2]-here[2]))<.05 and not await self.start_footprint_clear(start)):
            raise MissionError('NO_VALID_PATH_'+state)
        if planners is None:
            # pick_via_x: tới X thử PalletX trước (đường cong tự do quanh vật); không có
            # đường mới dùng các planner có đoạn thẳng cuối như cũ.
            if getattr(self,'pick_via_x',False) and state in ('NAV_PICK','REFINE_PICK'):
                try:return await self.compute_dynamic_segment(start,end,state,('PalletX',))
                except MissionError as error:
                    if str(error)!='NO_VALID_PATH_'+state:raise
            planners=DYNAMIC_PLANNERS
        candidates=[]
        for planner in planners:
            goal=ComputePathToPose.Goal(start=self.map_pose(start),goal=self.map_pose(end),use_start=True,planner_id=planner)
            try:result=await self.action(self.planner,goal,12.)
            except MissionError as error:
                # Only a completed planner failure is a rejected candidate.
                # Cancellation, timeouts and sensor faults must abort the mission.
                if not str(error).startswith('ACTION_FAILED_'):raise
                continue
            points=[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in result.path.poses]
            if len(points)<2 or any(not all(math.isfinite(v) for v in p) for p in points):continue
            if math.hypot(points[-1][0]-end[0],points[-1][1]-end[1])>.06 or abs(wrap(points[-1][2]-end[2]))>.06:continue
            if state=='NAV_PICK' and self.path_valid.service_is_ready():
                # 02/11/2026: PalletX (Smac thuần) cho tâm xe đi qua ô inscribed (99) miễn viền
                # không chạm lethal, nhưng preflight IsPathValid từ chối đúng đường đó -> lập lại
                # ra cùng đường -> 3 lần "NAV_PICK blocked" -> ROUTE_BLOCKED_NAV_PICK khi xe
                # chưa lăn bánh. Loại ứng viên ngay tại đây để thử planner kế tiếp, hoặc báo
                # NO_VALID_PATH (follow_monitored chờ rồi thử lại, không tính là bị chặn).
                path=NavPath();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
                path.poses=[self.map_pose(p) for p in points]
                if not await self.path_valid_now(state,path,start):
                    self.get_logger().warning('%s: %s candidate fails IsPathValid; trying next planner'%(state,planner))
                    continue
            candidates.append(dict(stage=state,kind='navigation',planner=planner,collision_checked=True,
                poses=points,statistics=path_statistics(points)))
        if not candidates:raise MissionError('NO_VALID_PATH_'+state)
        return choose_navigation_candidate(candidates)
    def rotation_clear(self,x,y,start_yaw,delta,loaded=True):
        # Planner Reeds-Shepp không quay tại chỗ; xe mecanum thì quay được. Kiểm
        # tra vùng quét theo đúng chiều quay với footprint chở hàng, cùng quy tắc
        # TerminalApproachPlanner: tâm < inscribed (99), viền không lethal/unknown.
        cm=self.data.get('costmap')
        if cm is None or not self.fresh('costmap',5.):return False
        info=cm.info;W,H,res=info.width,info.height,info.resolution;ox,oy=info.origin.position.x,info.origin.position.y
        def cost(px,py):
            mx,my=int((px-ox)/res),int((py-oy)/res)
            return cm.data[my*W+mx] if 0<=mx<W and 0<=my<H else -1
        if not 0<=cost(x,y)<99:return False
        xmin=self.g['footprint_xmin'];xmax=self.g['footprint_xmax'];hw=self.g['footprint_halfwidth']
        # NAV_PICK quay khi chưa chở: footprint chở dài hơn sẽ "chạm" chính pallet.
        if loaded:xmax=max(xmax,self.g['loaded_xmax']);hw=max(hw,self.g['loaded_halfwidth'])
        n=max(2,int(math.ceil((xmax-xmin)/.02))+1);m=max(2,int(math.ceil(2*hw/.02))+1)
        edge=[(xmin+(xmax-xmin)*i/(n-1),s*hw) for i in range(n) for s in (-1,1)]+[(u,-hw+2*hw*j/(m-1)) for j in range(m) for u in (xmin,xmax)]
        # Bước 2,5°: góc xa nhất (~0,57 m) dịch ~2,5 cm < một ô costmap 4 cm.
        steps=max(1,int(math.ceil(abs(delta)/math.radians(2.5))))
        for k in range(steps+1):
            a=start_yaw+delta*k/steps;c,s=math.cos(a),math.sin(a)
            if any(not 0<=cost(x+u*c-v*s,y+u*s+v*c)<100 for u,v in edge):return False
        return True
    async def compute_drop_segment(self,start,end):
        # Chỉ khi không có đường trực tiếp mới thử quay tại chỗ trước NAV_DROP;
        # góc nhỏ trước, mỗi góc phải sạch vùng quét rồi mới hỏi planner.
        if getattr(self,'pivot_paths',False):
            pivot=self.pivot_segment(start,end,'NAV_DROP',True)
            if pivot is not None:return pivot
        try:return await self.compute_dynamic_segment(start,end,'NAV_DROP')
        except MissionError as error:
            if str(error)!='NO_VALID_PATH_NAV_DROP':raise
        if getattr(self,'drop_short_terminal',False):
            # Xe thật: ưu tiên đường cong có đoạn thẳng cuối ngắn hơn trước khi quay tại chỗ.
            try:
                segment=await self.compute_dynamic_segment(start,end,'NAV_DROP',('Pallet07','Pallet05'))
                self.get_logger().info('NAV_DROP uses a short terminal approach (%s)'%segment.get('planner'))
                return segment
            except MissionError as error:
                if str(error)!='NO_VALID_PATH_NAV_DROP':raise
        for degrees in (30,-30,60,-60,90,-90,120,-120,150,-150,180,-180):
            delta=math.radians(degrees)
            if not self.rotation_clear(start[0],start[1],start[2],delta):continue
            try:segment=await self.compute_dynamic_segment([start[0],start[1],wrap(start[2]+delta)],end,'NAV_DROP')
            except MissionError as error:
                if str(error)!='NO_VALID_PATH_NAV_DROP':raise
                continue
            self.get_logger().info('NAV_DROP needs in-place rotation %+.0f deg before path'%degrees)
            return {**segment,'pre_rotation':delta}
        raise MissionError('NO_VALID_PATH_NAV_DROP')
    async def clear_carried_pallet_marks(self):
        # Pallet đang chở bị scan_relay lọc (NaN) nên tia không xóa được vết pallet
        # ở chỗ cũ -> planner thấy "pallet ảo". Xóa costmap rồi chờ ~5 lần quét để
        # vật cản thật được đánh dấu lại. Map tĩnh không chứa pallet nên không mất.
        if not getattr(self,'clear_costmaps',None):return
        for client in self.clear_costmaps:
            if client.service_is_ready():await self.wait(client.call_async(ClearEntireCostmap.Request()),5.)
        await self.pause(1.2)
    async def rotate_in_place(self,delta,state):
        # Spin của Nav2 tự kiểm tra va chạm footprint (đang là footprint chở
        # hàng) trong lúc quay; nguồn vận tốc chuyển sang Nav2 như ORIENT_PICK.
        # Xe thật (spin_overshoot > 0): Spin chạy trên laptop, odom/lệnh qua Wi-Fi trễ nên lúc
        # Spin báo đủ góc xe còn quay và trôi thêm (đo 03/11: 0,16-0,20 rad ở 0,5 rad/s; mission
        # 00:48 quay 90° lố ~35°, phải quay bù 1-2 lượt). Đặt Spin ngắn đi đúng mức trôi ước tính,
        # đo góc thật theo odom sau khi đứng yên, còn lệch > spin_tolerance thì quay sửa (tối đa 2
        # lần), và cập nhật mức trôi theo lần đo (trung bình trượt) cho các lần quay sau.
        self.stage(state);await self.stationary()
        start=self.odom_yaw() if getattr(self,'spin_compensation',False) else None
        # 03/11/2026 18:44: pivot "quay -4°" -> Spin ở sàn 0,35 rad/s quay -12°. Góc nhỏ hơn độ
        # trôi + dung sai thì Spin nào cũng lố hơn chính góc đó: bỏ qua, ALIGN/MPPI tự chỉnh khi tiến.
        tolerance=float(self.p.get('spin_tolerance',.06))
        if start is not None and abs(delta)<=self.spin_overshoot+tolerance:
            self.get_logger().info('%s: skip Spin %+.2f rad (<= overshoot %.2f + tolerance %.2f)'%(
                state,float(delta),self.spin_overshoot,tolerance))
            return
        remaining=float(delta);previous=math.inf
        for attempt in range(3 if start is not None else 1):
            if attempt and abs(remaining)<=float(self.p.get('spin_tolerance',.06)):break
            # Góc còn thiếu nhỏ hơn độ trôi thì Spin nào cũng lố hơn chính góc đó: chấp nhận, để
            # MPPI/ALIGN tự chỉnh khi tiến. Sai số không giảm sau lượt sửa cũng dừng (không lắc).
            if attempt and abs(remaining)<=self.spin_overshoot:break
            if abs(remaining)>=previous:break
            previous=abs(remaining)
            # Giữ lại ít nhất 0,05 rad lệnh để Spin vẫn quay khi góc cần quay nhỏ.
            comp=min(self.spin_overshoot,max(0.,abs(remaining)-.05)) if start is not None else 0.
            command=remaining-math.copysign(comp,remaining)
            before=self.odom_yaw() if start is not None else None
            await self.select(False)
            # 08/11/2026: 10+|góc|/0,15 -> 4+|góc|/0,1. Góc nhỏ không phải đợi 10 s cố định khi
            # bánh kẹt; góc lớn giữ ~21 s cho 1,7 rad (đo: Spin thật 1,7 rad mất 14 s, ~0,12 rad/s).
            goal=Spin.Goal();goal.target_yaw=float(command);goal.time_allowance.sec=int(4+abs(command)/.1)
            await self.action(self.spin_action,goal,goal.time_allowance.sec+5.)
            await self.select(True);await self.stationary()
            if start is None or before is None:break
            after=self.odom_yaw()
            if after is None:break
            turned=wrap(after-before);remaining=wrap(float(delta)-wrap(after-start))
            # Chỉ học từ lần quay đủ dài và đúng chiều; kẹp 0..0,5 rad để một lần đo lỗi không phá.
            if abs(command)>=.2 and turned*command>0:
                observed=min(.5,max(0.,math.copysign(1.,command)*(turned-command)))
                self.spin_overshoot=.5*self.spin_overshoot+.5*observed
            self.get_logger().info('%s: asked %+.2f rad, Spin %+.2f, turned %+.2f, remaining %+.2f; overshoot estimate %.2f rad'%(
                state,float(delta),command,turned,remaining,self.spin_overshoot))
    def odom_yaw(self):
        # Góc thân xe trong khung odom (EKF trên Pi): liên tục, không nhảy theo AMCL như map.
        try:return yaw(self.tf.lookup_transform('odom','base_link',rclpy.time.Time()).transform.rotation)
        except Exception:return None
    def dynamic_pick_geometry(self):
        station=self.dynamic_pick;face=[*station['face'],station['approach'][2]]
        pre=refined_approach(face,self.p['preinsert_distance'],self.p['laser_x'])
        inserted=refined_approach(face,self.p['inserted_distance'],self.p['laser_x'])
        retreat=compose(inserted,[-self.p['retreat_distance'],0.,0.])
        return pre,inserted,retreat
    async def plan_dynamic_mission(self,pick,drop):
        self.stage('PLAN_MISSION');await self.select(True);await self.stationary()
        await self.wait(self.nav_cancel.call_async(CancelGoal.Request()),5.)
        self.dynamic_pick=pick;self.dynamic_drop=drop;self.dynamic_segments={};self.approved_sha256='';self.face_anchor=None;self.track_lost_since=None
        self.via_x_target=None;self.pallet_seen=None;self.watch_samples=[]
        self.clear_route_display()
        # Preflight both legs with the conservative carried footprint, as in
        # the old route preview. No wheel commands are enabled while planning.
        current=self.current_map_pose()
        if getattr(self,'nav_pick_curved',False):
            # Xe thật không đi ngang được (30/10/2026): luôn lập đường cong (Reeds-Shepp,
            # có thể lùi) tới điểm đứng đã thẳng hàng pallet; ALIGN chỉ còn sửa vài cm.
            # Chỉ bỏ NAV_PICK khi xe đã gần như đứng đúng điểm đứng.
            segment=None
            if not self.at_staging(current,pick['approach']):
                # NAV_PICK chạy khi CHƯA chở: footprint chở dài hơn sẽ chạm chính pallet.
                await self.update_footprint(loaded=False)
                if getattr(self,'pivot_paths',False):segment=self.pivot_segment(current,pick['approach'],'NAV_PICK',False)
                # pick_via_x xe thật: thử điểm tạm TRƯỚC khi quay tại chỗ. 02/11/2026: "lùi 0,50 m rồi
                # quay +30°" -> Spin 0,52 rad hết 13 s chưa xong (quay tại chỗ trên sàn kẹt bánh),
                # ACTION_FAILED_6; điểm tạm chỉ cần đi tới gần rồi dò pallet.
                via=self.via_x('NAV_PICK') and getattr(self,'replan_on_block',False)
                if segment is None:segment=await self.curved_pick_segment(current,pick,rotate=not via)
                if segment is None:segment=await self.via_x_proxy_segment(current,pick)
                if segment is None and via:
                    found=await self.rotated_pick_segment(current,pick)
                    if found is not None:
                        degrees,delta,rotated=found
                        self.get_logger().info('NAV_PICK needs in-place rotation %+.0f deg before path'%degrees)
                        segment={**rotated,'pre_rotation':delta}
            if segment is None:
                near=self.near_staging(current,pick['approach'])
                if not near and not self.at_staging(current,pick['approach']):
                    await self.update_footprint(loaded=True);raise MissionError('NO_VALID_PATH_NAV_PICK')
                self.get_logger().info('Robot at staging pose; skipping NAV_PICK, LiDAR alignment takes over')
                segment=dict(stage='NAV_PICK',kind='lidar_docking',collision_checked=False,
                             skip_navigation=True,poses=straight_samples(current,pick['approach']))
            self.dynamic_segments['NAV_PICK']=segment
            await self.update_footprint(loaded=True)
        else:
            near=self.near_staging(current,pick['approach'])
            if near:
                # Đoạn căn chỉnh tới pallet chạy khi CHƯA chở: kiểm tra với footprint
                # trống. Footprint chở dài hơn sẽ "chạm" chính mặt pallet ở pre-insert.
                await self.update_footprint(loaded=False)
                near=await self.docking_join_valid(current,pick)
            await self.update_footprint(loaded=True)
            if near:
                # Xe đã sát điểm đứng: hành lang thẳng 0,9 m phía sau nó có thể nằm
                # trong tường, planner kiểu ô tô không nối được vài cm lệch ngang.
                # ALIGN (LiDAR, mecanum đi ngang/xoay) xử lý phần này; đường join đã
                # kiểm tra bằng IsPathValid như REFINE_PICK.
                self.get_logger().info('Robot already near staging pose; skipping NAV_PICK, LiDAR alignment takes over')
                self.dynamic_segments['NAV_PICK']=dict(stage='NAV_PICK',kind='lidar_docking',collision_checked=False,
                    skip_navigation=True,poses=straight_samples(current,pick['approach']))
            else:self.dynamic_segments['NAV_PICK']=await self.compute_dynamic_segment(current,pick['approach'],'NAV_PICK')
        _,_,retreat=self.dynamic_pick_geometry()
        try:self.dynamic_segments['NAV_DROP']=await self.compute_drop_segment(retreat,drop['approach'])
        except MissionError as error:
            if str(error)!='NO_VALID_PATH_NAV_DROP' or getattr(self,'require_drop_preflight',True):raise
            self.get_logger().warning('NAV_DROP preflight blocked (pallet still on floor?); replanning after pick')
            # Chỉ để hiển thị; NAV_DROP luôn lập lại từ pose thật sau khi nhấc.
            self.dynamic_segments['NAV_DROP']=dict(stage='NAV_DROP',kind='deferred',collision_checked=False,
                poses=straight_samples(retreat,drop['approach']))
        self.publish_dynamic_route()
        await self.update_footprint()
    def clear_route_display(self):
        empty=NavPath();empty.header=self.map_pose([0.,0.,0.]).header
        self.active_path.publish(empty);self.route_path.publish(empty)
        clear=Marker();clear.action=Marker.DELETEALL
        self.route_markers.publish(MarkerArray(markers=[clear]))
        self.route_info.publish(String(data=json.dumps(dict(route_source='dynamic_planned',
            planned_before_motion=False,segments=[],status='PLANNING'))))
    def publish_dynamic_route(self):
        pre,inserted,retreat=self.dynamic_pick_geometry();segments=[self.dynamic_segments['NAV_PICK']]
        if 'REFINE_PICK' in self.dynamic_segments:segments.append(self.dynamic_segments['REFINE_PICK'])
        def line(name,start,end):
            segments.append(dict(stage=name,kind='lidar_docking',collision_checked=False,poses=straight_samples(start,end)))
        line('ALIGN_PICK',self.dynamic_pick['approach'],pre);line('INSERT_PICK',pre,inserted)
        line('RETREAT_PICK',inserted,retreat);segments.append(self.dynamic_segments['NAV_DROP'])
        release=self.dynamic_drop['approach'];withdrawn=compose(release,[-self.p['retreat_distance'],0.,0.])
        line('RETREAT_DROP',release,withdrawn)
        if 'RETURN_HOME' in self.dynamic_segments:segments.append(self.dynamic_segments['RETURN_HOME'])
        self.publish_route(segments,segments[0]['poses'][0],inserted,release)
    def map_pose(self, values):
        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=self.get_clock().now().to_msg()
        pose.pose.position.x=float(values[0]);pose.pose.position.y=float(values[1])
        pose.pose.orientation.z=math.sin(values[2]/2);pose.pose.orientation.w=math.cos(values[2]/2)
        return pose
    async def plan_mission(self):
        self.stage('PLAN_MISSION');await self.select(True);await self.stationary()
        await self.wait(self.nav_cancel.call_async(CancelGoal.Request()),5.)
        await self.pause(.2)
        transform=self.tf.lookup_transform('map','base_link',rclpy.time.Time()).transform
        start=[transform.translation.x,transform.translation.y,yaw(transform.rotation)]
        ap,ai,ar=station_poses(self.p,self.p['a']);bp,bi,br=station_poses(self.p,self.p['b'])
        if self.approved_file:
            approved=load_approved_route(self.approved_file,self.p)
            self.approved_sha256=approved['sha256']
            self.publish_route(approved['segments'],start,approved['lift_at'],approved['release_at'])
            return
        segments=[]
        async def navigation(name,begin,end):
            goal=ComputePathToPose.Goal(start=self.map_pose(begin),goal=self.map_pose(end),use_start=True,planner_id='GridBased')
            result=await self.action(self.planner,goal,12.)
            if len(result.path.poses)<2:raise MissionError('MISSION_ROUTE_EMPTY_'+name)
            segments.append(dict(stage=name,kind='navigation',collision_checked=True,
                poses=[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in result.path.poses]))
        def line(name,begin,end):
            # Intentional contact/docking legs use the existing LiDAR/sweep
            # interlocks at execution. They are not claimed as Nav2 free space.
            segments.append(dict(stage=name,kind='lidar_docking',collision_checked=False,poses=straight_samples(begin,end)))
        await navigation('NAV_PICK',start,self.p['a']['approach'])
        line('ALIGN_PICK',self.p['a']['approach'],ap);line('INSERT_PICK',ap,ai)
        line('RETREAT_PICK',ai,ar)
        await navigation('NAV_DROP',ar,self.p['b']['approach'])
        line('ALIGN_DROP',self.p['b']['approach'],bp);line('INSERT_DROP',bp,bi)
        line('RETREAT_DROP',bi,br)
        await navigation('RETURN_HOME',br,self.p['home'])
        self.publish_route(segments,start,ai,bi)
    def publish_route(self,segments,start,ai,bi):
        self.route_segments=segments;self.route_revision+=1
        combined=NavPath();combined.header=self.map_pose(start).header
        markers=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
        palette={'NAV_PICK':(.2,.65,1.),'REFINE_PICK':(1.,.9,.1),'NAV_DROP':(.2,1.,.3),'RETURN_HOME':(.8,.3,1.)}
        for index,segment in enumerate(segments):
            poses=segment['poses'];combined.poses.extend(self.map_pose(p) for p in poses)
            color=palette.get(segment['stage'],(1.,.5,.05) if 'RETREAT' in segment['stage'] else (.15,.9,.9))
            marker=Marker();marker.header=combined.header;marker.ns='mission_route';marker.id=index
            marker.type=Marker.LINE_STRIP;marker.action=Marker.ADD;marker.pose.orientation.w=1.;marker.scale.x=.025
            marker.color.r,marker.color.g,marker.color.b=color;marker.color.a=.85
            marker.points=[Point(x=float(p[0]),y=float(p[1]),z=.035) for p in poses];markers.markers.append(marker)
            label=Marker();label.header=combined.header;label.ns='mission_labels';label.id=index
            label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.orientation.w=1.
            label.pose.position=Point(x=float(poses[len(poses)//2][0]),y=float(poses[len(poses)//2][1]),z=.18+.04*(index%3))
            label.scale.z=.075;label.color.r=label.color.g=label.color.b=label.color.a=1.;label.text=f'{index+1}. '+segment['stage'];markers.markers.append(label)
        self.route_path.publish(combined);self.route_markers.publish(markers)
        self.route_info.publish(String(data=json.dumps(dict(revision=self.route_revision,planned_before_motion=self.state=='PLAN_MISSION',
            route_source='dynamic_planned' if self.dynamic else ('approved_preview' if self.approved_file else 'live_planner'),approved_sha256=self.approved_sha256,withdrawal_clearance=self.p['retreat_distance'],home=None if self.dynamic else self.p['home'],docking_uses_live_lidar=True,terminal_distance=self.p.get('terminal_distance',.5),retreat_minimum=self.p['retreat_distance'],turn_setup_distance=self.p.get('turn_setup_distance',0.),lift_at=ai,release_at=bi,segments=segments))))
        self.get_logger().info('Full mission route planned before NAV_PICK: '+str(len(segments))+' segments')
    async def stationary(self):
        self.command();end=time.monotonic()+4.;stable=None
        while True:
            t=self.data['odom'].twist.twist
            if max(abs(t.linear.x),abs(t.linear.y),abs(t.angular.z))<.008:
                stable=stable or time.monotonic()
                if time.monotonic()-stable>.4:return
            else:stable=None
            if time.monotonic()>end:raise MissionError('BASE_NOT_STATIONARY')
            await self.pause()
    async def move_lift(self,q,state):
        self.stage(state);await self.stationary();offset=self.data['lift'].home_offset
        if not .0<=q-offset<=self.g['lift_travel']:raise MissionError('LIFT_REFERENCE_INVALID')
        await self.action(self.lift,MoveLift.Goal(height=q-offset,timeout=15.),20.)
        await self.pause(.35)
        await self.wait_lift_settled(state)
    async def wait_lift_settled(self,state,timeout=3.):
        # 03/11/2026 xe thật: xong homing (và hạ về công tắc) bridge gửi D, ESP32 về DISARMED rồi
        # ~0,24 s sau mới ACTIVE lại; /lift/state báo inhibited trong khoảng đó. Bước kế tiếp
        # (PLAN_MISSION, RETREAT_DROP...) không còn nằm trong danh sách miễn của check() nên
        # báo LIFT_INTERLOCK_CHANGED sau 60 ms. Giữ nguyên state nâng/hạ (được miễn) và chờ
        # cơ cấu nâng hết moving/inhibited, tối đa timeout giây.
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            lift=self.data.get('lift')
            if lift is not None and self.fresh('lift') and not lift.moving and not lift.inhibited:return
            await self.pause(.05)
        self.get_logger().error('%s: lift still moving/inhibited %.1f s after the lift action'%(state,timeout))
        raise MissionError('LIFT_NOT_READY_AFTER_'+state)
    def observation(self,station):
        if self.dynamic or getattr(self,'previewing',False):return self.dynamic_observation(station)
        try:t=self.tf.lookup_transform('map','base_link',rclpy.time.Time()).transform
        except Exception:return None
        a=yaw(t.rotation);dx=station['face'][0]+station['marker_offset_x']-t.translation.x;dy=station['face'][1]+station['marker_offset_y']-t.translation.y
        expected=(math.cos(a)*dx+math.sin(a)*dy-self.p['laser_x'],-math.sin(a)*dx+math.cos(a)*dy)
        f=detect_face(self.data['scan'],expected,station['width'],wrap(station['marker_yaw']-a),getattr(self,'width_range',None))
        if f is not None:
            f['yaw']=wrap(f['yaw']-station['marker_yaw'])
            stamp=(self.data['scan'].header.stamp.sec,self.data['scan'].header.stamp.nanosec)
            previous=getattr(self,'filtered_face',None)
            if stamp!=getattr(self,'fit_stamp',None):
                if previous is not None:
                    for key in ('x','y'):f[key]=.5*f[key]+.5*previous[key]
                    f['yaw']=wrap(previous['yaw']+.5*wrap(f['yaw']-previous['yaw']))
                self.filtered_face=f;self.fit_stamp=stamp
            f=self.filtered_face
        self.face=f
        if f is None:
            self.debug_saved=True
            scan=self.data['scan']
            debug=dict(expected=expected,map_yaw=a,station=station,angle_min=scan.angle_min,
                angle_increment=scan.angle_increment,range_min=scan.range_min,range_max=scan.range_max,ranges=list(scan.ranges))
            try:Path('/tmp/forklift_p6_docking_'+os.environ.get('ROS_DOMAIN_ID','0')+'.json').write_text(json.dumps(debug))
            except OSError:pass
        return f
    def dynamic_observation(self,station):
        # Use the actual scan frame, including sensor translation and yaw.
        scan=self.data['scan']
        stats=getattr(self,'fit_stats',None)
        try:
            t=self.tf.lookup_transform('map',scan.header.frame_id,
                rclpy.time.Time.from_msg(scan.header.stamp)).transform
            base=self.tf.lookup_transform('base_link',scan.header.frame_id,rclpy.time.Time()).transform
        except Exception as error:
            # Không có TF map->laser_frame ở đúng stamp của scan (AMCL chậm/Wi-Fi) thì KHÔNG
            # quét nào fit được, nhưng trước đây im lặng y như "không thấy pallet".
            if stats is not None:
                stats['tf']=stats.get('tf',0)+1;stats['tf_loi']=str(error)[:80]
            return None
        sensor=[t.translation.x,t.translation.y,yaw(t.rotation)]
        # Xe thật: bám theo lần đo LiDAR gần nhất thay vì vị trí pallet lập từ đầu.
        # Bánh mecanum trượt làm odom/map lệch dần (29/10/2026: odom báo quay 19°,
        # LiDAR chỉ thấy ~4°) -> cửa sổ dò lệch khỏi pallet thật, PARCEL_FACE_NOT_OBSERVABLE.
        # Mỗi lần đo tính trong cùng khung TF hiện tại nên neo trôi theo nhất quán.
        anchor=getattr(self,'face_anchor',None)
        tracking=getattr(self,'track_face',False) and anchor is not None and anchor[3] is station
        # Neo không khớp liên tục > 0,7 s (30/10/2026: mất dấu khi đứng yên sau xung
        # quay lố 9°): dò lại theo vị trí lập ban đầu và nhận lần đo đó làm neo mới,
        # thay vì kẹt với một neo sai tới PARCEL_FACE_NOT_OBSERVABLE.
        lost=getattr(self,'track_lost_since',None)
        reacquire=tracking and lost is not None and time.monotonic()-lost>.7
        ref=list(anchor[:3]) if tracking and not reacquire else [*station['face'],station['approach'][2]]
        expected=compose(inverse(sensor),ref)
        if stats is not None:
            # Cửa sổ tìm đặt ở đâu (khung map), nguồn của nó, và pose LiDAR lúc đó: nếu cửa sổ
            # lệch khỏi pallet thật > 0,46 m thì detect_face không có cụm nào để xét.
            stats['cua_so']=[round(v,3) for v in ref]
            stats['nguon_cua_so']='neo LiDAR' if tracking and not reacquire else 'mat pallet cua station'
            stats['lidar']=[round(v,3) for v in sensor]
        f=detect_face(scan,expected[:2],station['width'],expected[2],getattr(self,'width_range',None),stats)
        map_face=compose(sensor,[f['x'],f['y'],f['yaw']]) if f is not None else None
        # Nhảy quá 10 cm / 0,2 rad giữa hai lần đo liền nhau: nhiều khả năng bám
        # nhầm vật khác, bỏ mẫu này.
        if f is not None and tracking and not reacquire and (
                math.hypot(map_face[0]-ref[0],map_face[1]-ref[1])>.10 or abs(wrap(map_face[2]-ref[2]))>.20):
            if stats is not None:
                stats['bo_nhay_neo']=stats.get('bo_nhay_neo',0)+1
                stats['nhay']=stats.get('nhay',[])+[round(math.hypot(map_face[0]-ref[0],map_face[1]-ref[1]),3)]
            f=None
        if f is None:
            if tracking and lost is None:self.track_lost_since=time.monotonic()
            self.face=None;return None
        if reacquire:self.get_logger().warning('Pallet face re-acquired after %.1f s without a tracked match'%(time.monotonic()-lost))
        self.track_lost_since=None
        if getattr(self,'track_face',False):self.face_anchor=(map_face[0],map_face[1],map_face[2],station)
        local=compose([base.translation.x,base.translation.y,yaw(base.rotation)],[f['x'],f['y'],f['yaw']])
        f.update(x=local[0]-self.p['laser_x'],y=local[1],yaw=local[2],map_pose=map_face)
        stamp=(scan.header.stamp.sec,scan.header.stamp.nanosec)
        previous=getattr(self,'filtered_face',None)
        if stamp!=getattr(self,'fit_stamp',None):
            if previous is not None:
                for key in ('x','y'):f[key]=.5*f[key]+.5*previous[key]
                f['yaw']=wrap(previous['yaw']+.5*wrap(f['yaw']-previous['yaw']))
            self.filtered_face=f;self.fit_stamp=stamp
        else:f=self.filtered_face
        self.face=f
        return f
    async def collect_pick_face(self,station,timeout,check=True):
        self.filtered_face=None;self.fit_stamp=None;self.fit_stats={}
        samples=[];sample_times=[]
        stamp=self.data['scan'].header.stamp
        last_stamp=(stamp.sec,stamp.nanosec);end=time.monotonic()+timeout
        # 04/11/2026: PALLET_DETECTION_NOT_STABLE không cho biết thiếu gì, mà ba nguyên nhân
        # cần ba cách xử lý khác nhau: (a) không thấy mặt pallet (bị che/ngoài tầm fit),
        # (b) /scan về quá chậm nên không đủ 5 quét KHÁC NHAU trong cửa sổ 1,5 s (X4 5 Hz qua
        # Wi-Fi, mất gói là hết), (c) có đủ mẫu nhưng rung quá 0,025 m / 0,05 rad (fit LiDAR
        # hoặc TF map->base_link của AMCL nhảy). Đếm và đo để log nói thẳng.
        seen=last_stamp;scans=0;hits=0;full=0;best=None
        while time.monotonic()<end:
            f=self.observation(station)
            stamp=self.data['scan'].header.stamp
            stamp=(stamp.sec,stamp.nanosec)
            if stamp>seen:seen=stamp;scans+=1
            # A scan callback can arrive before its corresponding odom TF.
            # Retry that scan on the next tick instead of consuming its stamp
            # and discarding otherwise valid observations on every TF delay.
            if f is not None and stamp>last_stamp:
                last_stamp=stamp;hits+=1
                samples.append(f['map_pose']);sample_times.append(time.monotonic())
                while sample_times and sample_times[0]<time.monotonic()-1.5:
                    sample_times.pop(0);samples.pop(0)
                samples=samples[-5:];sample_times=sample_times[-5:]
                if len(samples)==5:
                    full+=1;spread=face_spread(samples)
                    if best is None or spread[0]+spread[1]<best[0]+best[1]:best=spread
                    try:return stable_face(samples),len(samples)
                    except ValueError:pass
            # Dò thử khi bấm A chạy ngoài mission: không kiểm tra deadline/hủy mission.
            await self.pause(check=check)
        window=(sample_times[-1]-sample_times[0]) if len(sample_times)>1 else 0.
        stats=self.fit_stats;self.fit_stats=None
        if hits==0:
            # Nói rõ dừng ở bước nào: TF, không có cụm điểm, hay cụm bị loại vì gì.
            spans=stats.get('span',[])
            self.get_logger().warning(
                'collect_pick_face: khong fit duoc mat - TF loi %d (%s), %d quet qua detect_face, '
                '%d cum diem, bo: goc %d, be rong %d, khong phang %d, nhay neo %d; be rong do duoc %s; '
                'cua so tim quanh mat du kien cua station'%(
                    stats.get('tf',0),stats.get('tf_loi','-'),stats.get('quet',0),stats.get('cum',0),
                    stats.get('bo_goc',0),stats.get('bo_berong',0),stats.get('bo_khong_phang',0),
                    stats.get('bo_nhay_neo',0),
                    '%s (can %s)'%(sorted(spans)[:1]+sorted(spans)[-1:],getattr(self,'width_range',None)) if spans else '-'))
            self.get_logger().warning(
                'collect_pick_face: cua so tim quanh %s (%s), LiDAR o %s, mat pallet station %s'%(
                    stats.get('cua_so','-'),stats.get('nguon_cua_so','-'),stats.get('lidar','-'),
                    [round(v,3) for v in station['face']]))
        self.get_logger().warning(
            'collect_pick_face: %.1f s, %d quet moi, %d lan fit duoc mat, %d lan du 5 mau '
            '(cua so 1,5 s), rung tot nhat %s; %s'%(
                timeout,scans,hits,full,
                'chua du mau' if best is None else '%.3f m / %.3f rad (nguong 0,025 / 0,050)'%best,
                'khong thay mat pallet (bi che? ngoai tam fit?)' if hits==0 else
                ('/scan ve qua cham: %d mau trong %.2f s, can 5 mau khac nhau trong 1,5 s'%(len(samples),window)
                 if full==0 else 'do duoc nhung rung qua nguong: xem fit LiDAR va TF map->base_link (AMCL)')))
        return None
    def pick_measurement(self,station,face,samples):
        if math.hypot(face[0]-station['face'][0],face[1]-station['face'][1])>.25 or abs(wrap(face[2]-station['approach'][2]))>.35:
            raise MissionError('PALLET_CORRECTION_OUT_OF_RANGE')
        corrected=refined_approach(face,self.staging_distance,self.p['laser_x'])
        return dict(initial=list(station['approach']),corrected=corrected,face=face,samples=samples)
    def apply_pick_measurement(self,station,measurement):
        face=measurement['face'];corrected=measurement['corrected']
        self.detected_pose.publish(self.map_pose(face));self.corrected_pose.publish(self.map_pose(corrected))
        station['face']=face[:2];station['approach']=corrected;station['marker_yaw']=face[2]
    async def prepare_dynamic_pick(self,station):
        self.stage('DETECT_PICK_INITIAL');await self.stationary()
        result=await self.collect_pick_face(station,self.initial_detection_timeout)
        if result is None:
            self.initial_detection=dict(outcome='UNAVAILABLE',initial=list(station['approach']))
            return
        try:measurement=self.pick_measurement(station,*result)
        except MissionError as error:
            if str(error)!='PALLET_CORRECTION_OUT_OF_RANGE':raise
            self.initial_detection=dict(outcome='OUT_OF_RANGE',initial=list(station['approach']))
            return
        self.initial_detection=dict(outcome='DETECTED',**measurement)
        self.apply_pick_measurement(station,measurement)
    async def find_pallet_near(self,click,check=True):
        # Cửa sổ detect_face chỉ ±(width/2+0,25 m) quanh điểm kỳ vọng. Người dùng
        # có thể bấm giữa pallet thay vì mặt trước, nên lùi dần về phía robot.
        timeout=max(2.,self.initial_detection_timeout)
        for offset in (0.,.2,.4):
            guess=station_from_approach(compose(click,[-offset-self.staging_distance-self.p['laser_x'],0.,0.]),
                                        self.staging_distance,self.pallet_width,self.p['laser_x'])
            result=await self.collect_pick_face(guess,timeout,check)
            if result is not None:return result[0],result[1],offset
        return None
    def preview_pick_pose(self,msg):
        # Bấm A trên RViz: dò pallet ngay (chỉ đọc LiDAR, không ra lệnh bánh) để
        # người vận hành biết A có dùng được trước khi bấm Run.
        if getattr(self,'pick_pose_mode','staging')!='pallet' or self.active or self.previewing:return
        try:click=pose_values(msg)
        except ValueError:return
        self.executor.create_task(self.preview_pick(click))
    async def preview_pick(self,click):
        self.previewing=True;result=None
        try:
            if 'scan' in self.data:result=await self.find_pallet_near(click,check=False)
        except Exception as error:self.get_logger().warning('Pick preview failed: %s'%error)
        finally:self.previewing=False
        if result is None:
            info=dict(found=False,click=list(click),message='No pallet face near A (measured width outside range or not visible)')
            # pick_via_x: không thấy từ xa là bình thường khi vật che; xe sẽ dò lại ở X.
            if getattr(self,'pick_via_x',False):
                info.update(found=True,message='Pallet not visible from here (obstacle?); will scan at X %.2f m before A'%self.staging_distance)
        else:
            face,samples,offset=result;approach=refined_approach(face,self.staging_distance,self.p['laser_x'])
            self.preview_face=dict(click=list(click),face=list(face),stamp=time.monotonic())
            self.detected_pose.publish(self.map_pose(face));self.corrected_pose.publish(self.map_pose(approach))
            width=(self.filtered_face or {}).get('width')
            info=dict(found=True,click=list(click),face=face,staging=approach,width=width,samples=samples,search_offset=offset,
                message='Pallet found: face (%.2f, %.2f, %.0f deg)%s, staging (%.2f, %.2f)'%(face[0],face[1],math.degrees(face[2]),
                    ', width %.2f m'%width if width else '',approach[0],approach[1]))
        self.get_logger().info('Pick preview: '+info['message'])
        self.pick_preview.publish(String(data=json.dumps(info)))
    X_ZONE=.40
    def via_x(self,state):
        return state=='NAV_PICK' and getattr(self,'pick_via_x',False) and (self.initial_detection or {}).get('outcome')=='SKIPPED'
    def near_x(self,x,y,extra=0.):
        g=self.dynamic_pick['approach']
        return math.hypot(x-g[0],y-g[1])<=self.X_ZONE+extra
    async def resolve_blocked_x(self,state,outcome):
        # Xe đã tới gần X mà X vẫn chặn. True = đã xử lý (đi tiếp hoặc dùng pose hiện tại).
        await self.select(True);await self.stationary()
        cur=self.current_map_pose();goal=self.dynamic_pick['approach']
        if not self.near_x(cur[0],cur[1],.10):return False
        if self.footprint_path_clear([goal],False,True):return False   # X trống: lỗi khác, xử lý như cũ
        # 1) Lùi X dọc hướng pallet tới chỗ trống, nếu X mới vẫn còn ở PHÍA TRƯỚC xe >= 10 cm.
        moved,back=self.clear_virtual_x(goal)
        if back and compose(inverse(cur),moved)[0]>=.10:
            self.get_logger().warning('%s: X blocked near pallet; X moved back %.2f m, short replan'%(state,back))
            self.dynamic_pick['approach']=moved;self.initial_detection['initial']=list(moved)
            self.corrected_pose.publish(self.map_pose(moved))
            self.dynamic_segments[state]=await self.compute_dynamic_segment(cur,moved,state)
            self.publish_dynamic_route();self.stage(state)
            return True
        # 2) Lấy pose hiện tại làm X nếu từ đây vẫn dò pallet + ALIGN được (cùng tầm REFINE_PICK
        # chấp nhận: sau X gốc tới 0,45 m, vượt 5 cm, ngang 12 cm, góc 0,25 rad).
        if self.near_staging(cur,self.via_x_origin):
            self.get_logger().warning('%s: X still blocked near pallet; using current pose (%.2f, %.2f, %.0f deg) as X'%(
                state,cur[0],cur[1],math.degrees(cur[2])))
            self.dynamic_pick['approach']=list(cur);self.initial_detection['initial']=list(cur)
            self.corrected_pose.publish(self.map_pose(cur));self.via_x_taken=True
            return True
        error=compose(inverse(self.via_x_origin),cur)
        self.get_logger().error('%s: X blocked and current pose not usable as X (behind %.2f m, lateral %.2f m, yaw %.0f deg)'%(
            state,-error[0],error[1],math.degrees(error[2])))
        return False
    def watch_pallet(self):
        # 02/11/2026 (yêu cầu người dùng): X chỉ là điểm vẽ đường tới pallet bị che. Đã thấy
        # mặt pallet thì không cần tới X: trả mặt đo ổn định [x, y, yaw] (khung map) và ghi vào
        # pallet_seen. Gọi mỗi vòng drive_monitored nên chỉ xử lý scan MỚI, và chỉ khi pallet dự
        # kiến trong pallet_watch_range (X4 thấy mặt 0,3 m rõ trong ~2 m). Cùng tiêu chí với
        # collect_pick_face (5 mẫu/1,5 s, stable_face) và pick_measurement (lệch <=0,25 m/0,35 rad
        # so với mặt dự kiến) để không bám nhầm vật khác.
        station=self.dynamic_pick;scan=self.data.get('scan')
        if scan is None:return None
        stamp=(scan.header.stamp.sec,scan.header.stamp.nanosec)
        if stamp==getattr(self,'watch_stamp',None):return None
        self.watch_stamp=stamp
        try:cur=self.current_map_pose()
        except Exception:return None
        if math.hypot(station['face'][0]-cur[0],station['face'][1]-cur[1])>float(self.p.get('pallet_watch_range',2.0)):
            self.watch_samples=[];return None
        f=self.observation(station)
        if f is None or 'map_pose' not in f:return None
        now=time.monotonic()
        self.watch_samples=[(t,p) for t,p in getattr(self,'watch_samples',[]) if t>now-1.5][-4:]+[(now,list(f['map_pose']))]
        if len(self.watch_samples)<5:return None
        try:face=stable_face([p for _,p in self.watch_samples])
        except ValueError:return None
        if (math.hypot(face[0]-station['face'][0],face[1]-station['face'][1])>.25
                or abs(wrap(face[2]-station['approach'][2]))>.35):return None
        # 03/11/2026 18:31: thấy pallet từ SAU vật A (thấy qua khe/bên cạnh) -> bỏ X quá sớm,
        # REFINE_PICK lách sát A (hở ~1 ô), mỗi lần chạy ~1 s lại "blocked" -> ROUTE_BLOCKED.
        # Chỉ bỏ X khi đã qua vật: đoạn THẲNG từ xe tới điểm đứng trước pallet trống. Chưa
        # trống thì đi tiếp theo đường tới X (đã lập để tránh vật), scan sau xét lại.
        if not self.pallet_corridor_clear(face,cur):
            now=time.monotonic()
            if now-getattr(self,'corridor_log',0.)>3.:
                self.corridor_log=now
                self.get_logger().info('Pallet face seen but the straight approach is blocked; continuing towards X')
            return None
        self.pallet_seen=list(face)
        return self.pallet_seen
    def pallet_corridor_clear(self,face,cur):
        staging=refined_approach(face,self.staging_distance,self.p['laser_x'])
        dx,dy=staging[0]-cur[0],staging[1]-cur[1];dist=math.hypot(dx,dy);travel=math.atan2(dy,dx)
        n=max(1,int(math.ceil(dist/.05)))
        # Tiến thẳng theo hướng đi (xe skid-steer quay về hướng đó trước), rồi điểm đứng với
        # hướng pallet; điểm đứng kiểm cả diện tích footprint như khi chọn X.
        line=[(cur[0]+dx*i/n,cur[1]+dy*i/n,travel) for i in range(1,n+1)] if dist>.10 else []
        return self.footprint_path_clear(line,False) and self.footprint_path_clear([staging],False,True)
    async def look_for_pallet(self,state,timeout=2.):
        # Xe đứng yên (bị chặn/MPPI hủy/tới điểm tạm): dò mặt pallet tối đa timeout giây.
        self.watch_samples=[];end=time.monotonic()+timeout
        while time.monotonic()<end:
            face=self.watch_pallet()
            if face is not None:
                self.get_logger().warning('%s: pallet face seen at (%.2f, %.2f, %.0f deg) while stopped; skipping X, '
                    'planning straight to the pallet'%(state,face[0],face[1],math.degrees(face[2])))
                return face
            await self.pause(.05)
        return None
    async def rescan_costmap(self):
        # Xóa costmap rồi chờ một costmap phát SAU lúc xóa (global 2 Hz) để chỉ còn vật LiDAR
        # đang thấy. Vết cũ nằm sau vật khác thì tia không đi qua, không bao giờ tự mất.
        await self.select(True);await self.stationary()
        cleared=time.monotonic()
        await self.clear_carried_pallet_marks()
        end=time.monotonic()+5.
        while time.monotonic()<end and self.stamps.get('costmap',0.)<cleared+1.:await self.pause(.1)
        if not self.fresh('costmap',3.):raise MissionError('COSTMAP_STALE_RECHECK_X')
    async def refine_segment(self,start,pose):
        # 02/11/2026 xe thật: thấy pallet trên đường (watch_pallet) khi xe đang đi vòng sát vật
        # cản -> dừng ngay cạnh vật, REFINE_PICK "Starting point in lethal space" ở cả 7 planner
        # -> NO_VALID_PATH_REFINE_PICK sau 1 s, trong khi đứng lùi ra một chút là lập được.
        # Dự phòng như NAV_PICK: thẳng tới điểm đứng đo được; không được thì đường cong/lùi
        # 0,30-0,50 m (curved_pick_segment, KHÔNG quay tại chỗ: Spin kẹt bánh trên sàn); vẫn
        # không được thì chờ 1,5 s (costmap/vật động cập nhật) rồi thử lại, tối đa 3 lần.
        for attempt in range(3):
            if attempt:
                await self.pause(1.5);start=self.current_map_pose()
            try:return await self.compute_dynamic_segment(start,pose,'REFINE_PICK')
            except MissionError as e:
                if not str(e).startswith('NO_VALID_PATH_'):raise
            segment=await self.curved_pick_segment(start,{**self.dynamic_pick,'approach':list(pose)},rotate=False)
            if segment is not None:
                self.get_logger().warning('REFINE_PICK: direct path failed; using curved/back-out path (attempt %d)'%(attempt+1))
                return dict(segment,stage='REFINE_PICK')
            self.get_logger().warning('REFINE_PICK: no path to the measured staging pose yet (%d/3)'%(attempt+1))
        raise MissionError('NO_VALID_PATH_REFINE_PICK')
    async def via_x_proxy_segment(self,current,pick):
        # 02/11/2026 xe thật: không lập được đường tới X -> mission hủy dù pallet ngay đó. Hai kiểu:
        # (a) X bị ô lethal chiếm (vết cũ/vật nhìn từ xa nằm sau vật khác, tia không xóa được);
        # (b) X trống nhưng không vào được (vật thật dài ~0,3 m trên trục chỉ cách X 0,5 m: không
        # đủ chỗ vòng qua rồi về đúng trục). X chỉ là điểm vẽ đường (người dùng chốt): đi tới điểm
        # tạm GẦN X đi được (ngang/lùi sau X, lệch ngang ±0,3/±0,5 m), dọc đường và tới nơi
        # watch_pallet/look_for_pallet thấy pallet thì lập đường thẳng tới pallet; không thấy mới
        # recheck_via_x. Chỉ có ở nhánh replan_on_block của nav_to (xe thật). None = không áp dụng.
        if (not getattr(self,'replan_on_block',False) or not self.via_x('NAV_PICK')
                or not self.fresh('costmap',3.) or getattr(self,'start_blocked',None)):return None
        target=list(pick['approach']);blocker=self.footprint_blocker(self.data['costmap'],target,False,True)
        failures=0
        # Gần X trước (pallet trong tầm dò 2 m). Chỉ PalletX (đường cong tự do, ~1 s/lần) vì
        # điểm tạm không cần đoạn thẳng cuối; tối đa 10 điểm thử.
        offsets=sorted(((.05*k,lat) for k in range(25) for lat in (0.,.3,-.3,.5,-.5) if (k,lat)!=(0,0.)),
                       key=lambda o:math.hypot(*o))
        for back,lat in offsets:
            cand=compose(target,[-back,lat,0.])
            if not self.footprint_path_clear([cand],False,True):continue
            try:segment=await self.compute_dynamic_segment(current,cand,'NAV_PICK',('PalletX',))
            except MissionError as error:
                if str(error)!='NO_VALID_PATH_NAV_PICK':raise
                failures+=1
                if failures>=10:return None
                continue
            self.get_logger().warning('pick_via_x: no path into X (%.2f, %.2f)%s; driving to proxy %.2f m before X, '
                'lateral %+.2f m, to look for the pallet instead of aborting'%(target[0],target[1],
                ', X blocked at (%.2f, %.2f)'%blocker[:2] if blocker else ' (X clear but unreachable)',back,lat))
            self.via_x_target=target;pick['approach']=list(cand)
            return segment
        return None
    async def recheck_via_x(self,state):
        # Xe đứng ở điểm tạm, mặt hướng pallet. Xóa costmap (vết cũ không bị tia nào đi qua
        # thì không bao giờ tự mất), chờ scan mới đánh dấu lại vật THẬT đang thấy, rồi:
        # X trống -> vào X; X chặn -> lùi X tới chỗ trống (clear_virtual_x); vẫn chặn ->
        # STAGING_BLOCKED (có vật thật trong hành lang cắm càng, không vào thẳng được).
        target=self.via_x_target;self.via_x_target=None
        self.stage('RECHECK_X');await self.rescan_costmap()
        if self.footprint_path_clear([target],False,True):
            new=target;self.get_logger().info('RECHECK_X: X (%.2f, %.2f) is clear after LiDAR re-check'%(target[0],target[1]))
        else:
            moved,back=self.clear_virtual_x(target)
            if not back:
                blocker=self.footprint_blocker(self.data['costmap'],target,False,True)
                self.get_logger().error('RECHECK_X: X still blocked by an obstacle LiDAR sees now at %s; '
                    'clear the area in front of the pallet'%('(%.2f, %.2f)'%blocker[:2] if blocker else '?'))
                raise MissionError('STAGING_BLOCKED_NAV_PICK')
            new=moved;self.get_logger().warning('RECHECK_X: X still blocked; X moved back %.2f m to (%.2f, %.2f)'%(back,moved[0],moved[1]))
        self.dynamic_pick['approach']=list(new);self.initial_detection['initial']=list(new)
        self.corrected_pose.publish(self.map_pose(new))
        self.stage(state);await self.update_footprint()
        await self.replan_segment(state,self.current_map_pose(),new)
        return list(new)
    def clear_virtual_x(self,approach):
        # Lùi X dọc hướng pallet (5 cm/bước, tối đa 0,40 m) tới khi khung xe không chạm ô vật
        # cản trên global costmap. Không có costmap mới hoặc X đã trống: giữ nguyên.
        if not self.fresh('costmap',3.) or self.footprint_path_clear([approach],False,True):return approach,0.
        for back in (.05,.10,.15,.20,.25,.30,.35,.40):
            moved=compose(approach,[-back,0.,0.])
            if self.footprint_path_clear([moved],False,True):return moved,back
        return approach,0.
    def virtual_pick(self,click):
        # Điểm ảo X lùi staging_distance từ mặt pallet. 02/11/2026: bấm A sâu vào pallet 17 cm
        # (preview đo mặt ở 1,90 m, bấm ở 2,07 m) -> X chỉ cách mặt 0,48 m, mũi xe chạm pallet,
        # mọi planner NO_VALID_PATH_NAV_PICK. Nên nếu lúc bấm A (preview, không tốn thời gian
        # mission) LiDAR đã thấy mặt pallet thì dùng mặt đo được; vật che không thấy thì tin
        # điểm bấm (phải bấm đúng mép trước pallet).
        seen=getattr(self,'preview_face',None)
        if (seen and time.monotonic()-seen['stamp']<300. and math.hypot(seen['click'][0]-click[0],seen['click'][1]-click[1])<.01
                and abs(wrap(seen['click'][2]-click[2]))<.01):
            face=seen['face'];source='pallet face seen at A preview'
        else:face=list(click);source='clicked A (pallet not seen at preview)'
        approach=refined_approach(face,self.staging_distance,self.p['laser_x'])
        # 02/11/2026: pallet xa 3,3 m chỉ thấy 12 cm mặt -> preview không nhận, A bấm sau mặt
        # ~23 cm -> X đè lên pallet trên costmap, mọi planner NO_VALID_PATH_NAV_PICK. Khung xe ở
        # X chạm vật thì lùi X dọc mũi tên (tối đa 0,40 m) tới chỗ trống; dò mặt ở X rồi ALIGN
        # tự tiến lên đúng 0,65 m. Chỉ xét khi có costmap mới (không thì giữ nguyên X).
        approach,back=self.clear_virtual_x(approach)
        if back:self.get_logger().warning('pick_via_x: X overlaps an obstacle (A clicked behind the pallet face?); X moved back %.2f m'%back)
        # Mặt pallet dự đoán lùi theo X: X chạm vật nghĩa là mặt thật gần hơn điểm bấm, cửa sổ
        # dò ở X (±0,25 m quanh mặt dự đoán) phải đặt quanh mặt thật.
        station=station_from_approach(approach,self.staging_distance,self.pallet_width,self.p['laser_x'])
        self.initial_detection=dict(outcome='SKIPPED',initial=list(approach),mode='pallet_via_x')
        # X gốc (sau khi đã lùi vì bấm sâu) để xét "pose hiện tại dùng làm X được không".
        self.via_x_origin=list(approach);self.via_x_taken=False
        self.corrected_pose.publish(self.map_pose(approach))
        self.get_logger().info('pick_via_x: skip initial pallet scan; virtual X (%.2f, %.2f, %.0f deg) %.2f m before %s'%(
            approach[0],approach[1],math.degrees(approach[2]),self.staging_distance,source))
        return station
    async def locate_pallet_pick(self,click):
        self.stage('DETECT_PICK_INITIAL');await self.stationary()
        result=await self.find_pallet_near(click)
        if result is None:raise MissionError('PALLET_NOT_FOUND_AT_A')
        face,samples,offset=result
        # Pose đứng tính từ mặt đo được, theo pháp tuyến của chính mặt đó.
        approach=refined_approach(face,self.staging_distance,self.p['laser_x'])
        station=station_from_approach(approach,self.staging_distance,self.pallet_width,self.p['laser_x'])
        measurement=dict(initial=list(click),corrected=approach,face=face,samples=samples,mode='pallet',search_offset=offset)
        self.initial_detection=dict(outcome='DETECTED',**measurement)
        self.apply_pick_measurement(station,measurement)
        self.get_logger().info('Pallet face at (%.2f, %.2f, %.0f deg); staging pose (%.2f, %.2f)'%(
            face[0],face[1],math.degrees(face[2]),approach[0],approach[1]))
        return station
    async def refine_pick(self,station):
        self.stage('DETECT_PICK');await self.stationary()
        # Xe thật (replan_on_block): người/vật tạm che mặt pallet thì chờ rồi dò lại, tối đa 3 lần.
        attempts=3 if getattr(self,'replan_on_block',False) else 1
        for attempt in range(attempts):
            result=await self.collect_pick_face(station,5.)
            if result is not None:break
            if attempt+1<attempts:
                self.get_logger().warning('DETECT_PICK: pallet face not stable (%d/%d); something in front? waiting 2 s'%(attempt+1,attempts))
                await self.pause(2.)
        if result is None:raise MissionError('PALLET_DETECTION_NOT_STABLE')
        self.refinement=self.pick_measurement(station,*result)
        # SKIPPED (pick_via_x): xe đã đứng ở X, so mặt đo được với pose thật như dò sớm.
        early=(self.initial_detection or {}).get('outcome') in ('DETECTED','SKIPPED')
        current=self.current_map_pose() if early else None
        if early:self.refinement['arrival_error']=compose(inverse(self.refinement['corrected']),current)
        replan=needs_pick_replan(self.refinement['initial'],self.refinement['corrected'],current)
        self.apply_pick_measurement(station,self.refinement)
        # Đã sát điểm đứng thì không gọi planner lại (cùng lý do như NAV_PICK);
        # chỉ cần đường căn chỉnh LiDAR thông.
        if early and (not replan or self.near_staging(current,self.refinement['corrected'])):
            clear=await self.docking_join_valid(current,station)
            self.refinement['alignment_path_validated']=clear
            replan=not clear
        self.refinement['replan_required']=replan
        if replan:await self.nav_to(self.refinement['corrected'],'REFINE_PICK')
    def align_reach(self,current,approach):
        # Tầm dock_steer xử lý được từ điểm dừng của NAV_PICK: sau điểm đứng tới 0,45 m,
        # trước tới 0,10 m, ngang 0,15 m, góc 0,35 rad (detect_face nhận tới 0,40 rad).
        error=compose(inverse(approach),current)
        return -.45<=error[0]<=.10 and abs(error[1])<=.15 and abs(error[2])<=.35
    def at_staging(self,current,approach):
        # Đủ gần để ALIGN (tiến + bẻ lái, không đi ngang) xử lý ngay: dọc 5 cm,
        # ngang 2 cm, góc 0,05 rad. Lệch hơn thì lập đường cong cho NAV_PICK.
        error=compose(inverse(approach),current)
        return abs(error[0])<=.05 and abs(error[1])<=.02 and abs(error[2])<=.05
    async def pick_segment_from(self,start,pick):
        # Đoạn thẳng cuối dài (0,9-1,5 m) trước; phòng hẹp (drop_short_terminal, xe thật)
        # thì thử 0,7/0,5 m. Điểm đứng cách mặt pallet staging_distance nên cần trống
        # staging + terminal phía trước pallet (0,80+0,9 = 1,7 m với planner dài nhất ngắn).
        try:return await self.compute_dynamic_segment(start,pick['approach'],'NAV_PICK')
        except MissionError as error:
            if str(error)!='NO_VALID_PATH_NAV_PICK' or not getattr(self,'drop_short_terminal',False):raise
        segment=await self.compute_dynamic_segment(start,pick['approach'],'NAV_PICK',('Pallet07','Pallet05'))
        self.get_logger().info('NAV_PICK uses a short terminal approach (%s)'%segment.get('planner'))
        return segment
    async def return_home_to_pick(self):
        # Pallet đã giao: không lập được đường hoặc Nav2 bỏ dở thì xe đứng lại và mission
        # vẫn SUCCEEDED (cảnh báo). Hủy, mất phản hồi, khóa nâng vẫn là FAULT như mọi đoạn.
        # 31/10/2026: đường lập được nhưng IsPathValid báo chặn ngay sau khi hạ (vật cản
        # tạm trong costmap: pallet vừa thả/người đứng gần), vài chục giây sau lại hợp
        # lệ -> lập lại từ pose thật, tối đa 3 lần cách 2 s.
        if getattr(self,'replan_on_block',False):
            self.stage('RETURN_HOME');await self.select(True);await self.stationary();await self.update_footprint()
            try:
                await self.replan_segment('RETURN_HOME',self.current_map_pose(),self.dynamic_pick['approach'])
                while True:
                    await self.follow_monitored('RETURN_HOME',lambda current:self.replan_segment('RETURN_HOME',current,self.dynamic_pick['approach']))
                    if not await self.finish_pivot('RETURN_HOME',self.dynamic_pick['approach']):break
            except MissionError as e:
                if not (str(e).startswith(('NO_VALID_PATH_','ROUTE_BLOCKED_','ACTION_FAILED_','PLANNED_ROUTE_'))):raise
                await self.select(True)
                self.get_logger().warning('RETURN_HOME gave up (%s): robot stays here, pallet delivered'%e)
            return
        error=None
        for attempt in range(3):
            if attempt:await self.pause(2.)
            self.stage('RETURN_HOME');await self.select(True);await self.stationary()
            await self.update_footprint()
            try:
                segment=await self.pick_segment_from(self.current_map_pose(),self.dynamic_pick)
                self.dynamic_segments['RETURN_HOME']=dict(segment,stage='RETURN_HOME')
                self.publish_dynamic_route();self.stage('RETURN_HOME')
                await self.follow_approved_route('RETURN_HOME');return
            except MissionError as e:
                if not (str(e) in ('NO_VALID_PATH_NAV_PICK','PLANNED_ROUTE_BLOCKED','PLANNED_ROUTE_START_MISMATCH')
                        or str(e).startswith('ACTION_FAILED_')):raise
                error=e;await self.select(True)
                self.get_logger().warning('RETURN_HOME %s (attempt %d/3)'%(e,attempt+1))
        self.get_logger().warning('RETURN_HOME gave up (%s): robot stays here, pallet delivered'%error)
    async def reverse_clear(self,current,distance):
        back=compose(current,[-distance,0.,0.]);reverse=straight_samples(current,back)
        if not self.path_valid.service_is_ready():return None
        path=NavPath();path.header=self.map_pose(current).header;path.poses=[self.map_pose(p) for p in reverse]
        if not (await self.wait(self.path_valid.call_async(IsPathValid.Request(path=path)),5.)).is_valid:return None
        return back,reverse
    async def rotated_pick_segment(self,start,pick):
        # Planner Reeds-Shepp không quay tại chỗ; xe mecanum quay được. Hướng xuất phát
        # khác cho phép đường cong ngắn hơn trong phòng hẹp. Góc nhỏ trước; vùng quét
        # kiểm bằng footprint không tải (chưa nhấc pallet).
        for degrees in (30,-30,60,-60,90,-90,135,-135,180):
            delta=math.radians(degrees)
            if not self.rotation_clear(start[0],start[1],start[2],delta,loaded=False):continue
            try:segment=await self.pick_segment_from([start[0],start[1],wrap(start[2]+delta)],pick)
            except MissionError as error:
                if str(error)!='NO_VALID_PATH_NAV_PICK':raise
                continue
            return degrees,delta,segment
        return None
    async def curved_pick_segment(self,current,pick,rotate=True):
        # Đường cong Reeds-Shepp (có thể lùi) tới điểm đứng. Không lập được (thường do
        # xe quá sát pallet/tường hoặc phòng ngắn) thì lần lượt: lùi thẳng 0,30/0,50 m;
        # quay tại chỗ; lùi rồi quay. None = không có đường nào; người gọi quyết định
        # bỏ NAV_PICK hay báo lỗi. rotate=False khi lập lại giữa chừng (không quay thêm).
        try:return await self.pick_segment_from(current,pick)
        except MissionError as error:
            if str(error)!='NO_VALID_PATH_NAV_PICK':raise
        backs=[]
        for distance in (.30,.50):
            clear=await self.reverse_clear(current,distance)
            if clear is None:continue
            back,reverse=clear;backs.append((distance,back,reverse))
            try:segment=await self.pick_segment_from(back,pick)
            except MissionError as error:
                if str(error)!='NO_VALID_PATH_NAV_PICK':raise
                continue
            self.get_logger().info('NAV_PICK: backing out %.2f m before the curved approach'%distance)
            # Ghép đoạn lùi thẳng (giữ hướng xe) với đường cong; MPPI bám được điểm
            # đảo chiều (enforce_path_inversion).
            segment['poses']=reverse+segment['poses'][1:]
            segment['statistics']=path_statistics(segment['poses'])
            return segment
        if not rotate:return None
        found=await self.rotated_pick_segment(current,pick)
        if found is not None:
            degrees,delta,segment=found
            self.get_logger().info('NAV_PICK needs in-place rotation %+.0f deg before path'%degrees)
            # Đường lập cho góc lý tưởng; sau khi quay sẽ lập lại từ pose thật.
            return {**segment,'pre_rotation':delta}
        for distance,back,reverse in backs:
            found=await self.rotated_pick_segment(back,pick)
            if found is None:continue
            degrees,delta,segment=found
            self.get_logger().info('NAV_PICK: backing out %.2f m, then rotating %+.0f deg'%(distance,degrees))
            # Hiển thị = lùi + đường cong; thực thi: chỉ bám đoạn lùi, quay, rồi lập lại.
            return {**segment,'poses':reverse+segment['poses'],'statistics':path_statistics(reverse+segment['poses']),
                    'pre_reverse':reverse,'pre_rotation':delta}
        return None
    async def pick_maneuver(self):
        # Thực hiện lùi/quay đã lập ở PLAN_MISSION rồi lập lại NAV_PICK từ pose thật
        # (Spin dừng lệch vài độ; không bám đường tính cho góc lý tưởng).
        segment=self.dynamic_segments.get('NAV_PICK',{})
        if segment.get('kind')=='pivot':
            await self.start_pivot('NAV_PICK',self.dynamic_pick['approach'],segment);return
        if 'pre_rotation' not in segment:return
        if 'pre_reverse' in segment:
            self.dynamic_segments['NAV_PICK']={**segment,'poses':segment['pre_reverse'],
                                               'statistics':path_statistics(segment['pre_reverse'])}
            self.publish_dynamic_route();await self.follow_approved_route('NAV_PICK')
        await TaskManager.update_footprint(self)
        await self.rotate_in_place(segment['pre_rotation'],'ROTATE_PICK')
        self.stage('NAV_PICK');start=self.current_map_pose()
        self.dynamic_segments['NAV_PICK']=await self.pick_segment_from(start,self.dynamic_pick)
        self.publish_dynamic_route()
    def near_staging(self,current,approach):
        # Sai số trong khung điểm đứng: lùi sau tối đa 0,45 m, vượt tối đa 5 cm,
        # lệch ngang 12 cm, lệch góc 0,25 rad — trong tầm ALIGN tự sửa.
        error=compose(inverse(approach),current)
        return -.45<=error[0]<=.05 and abs(error[1])<=.12 and abs(error[2])<=.25
    async def docking_join_valid(self,current,station):
        if not self.path_valid.service_is_ready():return False
        points=alignment_samples(current,station['approach'])
        pre=refined_approach([*station['face'],station['approach'][2]],self.p['preinsert_distance'],self.p['laser_x'])
        points+=straight_samples(points[-1],pre)[1:]
        path=NavPath();path.header=self.map_pose(current).header
        path.poses=[self.map_pose(p) for p in points]
        result=await self.wait(self.path_valid.call_async(IsPathValid.Request(path=path)),5.)
        return result.is_valid
    def clear_sweep(self,vx):
        # Check the end opposite the intended pallet. Target face is handled by
        # docking geometry; unknown points alongside the insertion corridor stop.
        scan=self.data['scan']
        # Đổi tia sang base_link bằng TF thật: LiDAR xe thật lắp xoay +90° nên
        # không được coi góc scan là góc thân xe. Thiếu TF thì coi như bị chặn.
        try:t=self.tf.lookup_transform('base_link',scan.header.frame_id,rclpy.time.Time()).transform
        except Exception:return False
        tx,ty,ts=t.translation.x,t.translation.y,yaw(t.rotation)
        for i,r in enumerate(scan.ranges):
            if not math.isfinite(r):continue
            a=ts+scan.angle_min+i*scan.angle_increment;x=tx+r*math.cos(a);y=ty+r*math.sin(a)
            if vx<0 and self.g['footprint_xmin']-.12<x<self.g['footprint_xmin'] and abs(y)<self.g['footprint_halfwidth']:return False
        return True
    async def orient(self,station,state):
        # LiDAR supplies the desired correction; Nav2 Spin checks the full
        # costmap footprint while executing it. Stop translation at standoff.
        self.stage(state);await self.stationary()
        self.filtered_face=None;self.fit_stamp=None
        for attempt in range(7):
            end=time.monotonic()+1.5;f=None
            while f is None and time.monotonic()<end:
                f=self.observation(station)
                if f is None:await self.pause()
            if f is None:raise MissionError('PARCEL_FACE_NOT_OBSERVABLE')
            # Xe thật: lệch nhỏ hơn orient_min_yaw thì để ALIGN xử lý (lidar_pulse
            # 0,15 rad/s dừng theo LiDAR). Spin ≥0,30 rad/s qua smoother lố ~0,14 rad: 29/10
            # xin -0,10 rad thì đổi ~0,24 rad, lắc ±0,1 tới PALLET_ORIENTATION_TIMEOUT.
            if abs(f['yaw'])<max(.015,getattr(self,'orient_min_yaw',0.)):return
            if attempt==6:
                # Hết lượt Spin: còn trong tầm ALIGN (detect_face nhận tới 0,40 rad)
                # thì giao cho ALIGN thay vì báo lỗi. SIM (orient_min_yaw=0) giữ lỗi.
                if getattr(self,'orient_min_yaw',0.)>0. and abs(f['yaw'])<.30:return
                break
            # Under-step the measured angle to absorb the smoother/gate stop
            # delay; re-observe LiDAR after each collision-checked rotation.
            await self.select(False)
            # 08/11/2026: 12 -> 8 s. Góc xin nhỏ (< ~0,3 rad), lúc bánh kẹt ma sát chờ 12 s vô ích.
            goal=Spin.Goal();goal.target_yaw=float(.75*f['yaw']);goal.time_allowance.sec=8
            await self.action(self.spin_action,goal,11.)
            await self.select(True);await self.stationary()
            self.filtered_face=None;self.fit_stamp=None
        raise MissionError('PALLET_ORIENTATION_TIMEOUT')
    async def dock(self,station,insert=False):
        if getattr(self,'dock_no_strafe',False):return await self.dock_steer(station,insert)
        self.stage('INSERT' if insert else 'ALIGN');target=self.p['inserted_distance' if insert else 'preinsert_distance']
        end=time.monotonic()+45;last_good=time.monotonic();stable=None;self.debug_saved=False;self.filtered_face=None;self.fit_stamp=None
        while time.monotonic()<end:
            f=self.observation(station)
            if f is None:
                self.command()
                if time.monotonic()-last_good>1.5:raise MissionError('PARCEL_FACE_NOT_OBSERVABLE')
                await self.pause();continue
            last_good=time.monotonic();ex=f['x']-station['marker_offset_x']*math.cos(f['yaw'])+station['marker_offset_y']*math.sin(f['yaw'])-target
            # Correct the fiducial's lateral offset in its own rotated frame.
            ey=f['y']-station['marker_offset_x']*math.sin(f['yaw'])-station['marker_offset_y']*math.cos(f['yaw'])
            lateral_tolerance=getattr(self,'dock_lateral_tol',0.) or station.get('lateral_tolerance',.006)
            if insert and (abs(ey)>max(.014,2*lateral_tolerance) or abs(f['yaw'])>.055):raise MissionError('INSERT_ALIGNMENT_LOST')
            aligned=can_advance(ex,ey,f['yaw'],lateral_tolerance,insert,self.dock_yaw_tolerance())
            # Xe thật: nhiễu LiDAR ~±5 mm, 3 mm của SIM không ổn định được.
            if abs(ex)<max(station.get('distance_tolerance',.003),getattr(self,'dock_distance_tolerance',0.)) and aligned:
                self.command();stable=stable or time.monotonic()
                if time.monotonic()-stable>.5:return
            else:
                stable=None
                vx=clamp(.8*ex,self.p.get('insertion_speed',.015) if insert else self.p.get('approach_speed',.035)) if aligned else 0.
                # Cùng ngưỡng với can_advance: không có khoảng 'không tiến cũng không quay'.
                real=getattr(self,'dock_min_angular',0.)>0.
                if not insert and abs(f['yaw'])>(self.dock_yaw_tolerance() if getattr(self,'dock_yaw_tol',0.) else .015):
                    if real:
                        # Xe thật: quay một xung rồi dừng hẳn, đo lại bằng LiDAR. Lệnh
                        # liên tục tới bánh trễ ~1 s (smoother + ramp firmware) nên đảo
                        # chiều trước khi xe kịp theo lệnh cũ.
                        await self.lidar_pulse(station,'yaw',f['yaw'])
                        self.filtered_face=None;self.fit_stamp=None;last_good=time.monotonic();stable=None
                        continue
                    self.drive(0.,0.,clamp(f['yaw'],.08))
                elif real and not insert and not aligned:
                    # Góc đã đạt, chỉ lệch ngang: xung đi ngang dừng theo LiDAR. Lệnh
                    # liên tục 0,04 m/s thì bánh dính rồi giật 7,5 cm qua đích (29/10).
                    await self.lidar_pulse(station,'y',ey)
                    self.filtered_face=None;self.fit_stamp=None;last_good=time.monotonic();stable=None
                    continue
                else:
                    # Xe thật: góc đã trong dung sai thì không chỉnh khi đang tiến/ngang;
                    # lệnh nhỏ bị nâng lên dock_min_angular sẽ lắc qua lại quanh đích.
                    wz=0. if insert or real else clamp(f['yaw'],.04)
                    self.drive(vx,0. if insert or real else clamp(.8*ey,.02),wz)
                if not self.clear_sweep(vx):raise MissionError('DOCK_SWEEP_BLOCKED')
            await self.pause()
        raise MissionError('DOCK_TIMEOUT')
    async def dock_steer(self,station,insert=False):
        # Xe thật KHÔNG đi ngang (30/10/2026, sàn trơn + motor yếu ở tốc độ thấp): lệnh
        # vy 0,09-0,15 m/s chỉ dịch 0,4-8 cm mà xoay 2-52°. Chỉ dùng tiến/lùi (chạy được
        # từ 0,06 m/s) và quay (từ 0,20 rad/s): vừa tiến vừa bẻ lái để nhập đường tâm
        # pallet (Stanley), hết quãng mà còn lệch ngang thì lùi 0,20 m rồi vào lại.
        self.stage('INSERT' if insert else 'ALIGN');target=self.p['inserted_distance' if insert else 'preinsert_distance']
        end=time.monotonic()+self.dock_steer_timeout;last_good=time.monotonic();stable=None;self.filtered_face=None;self.fit_stamp=None
        lat_tol=getattr(self,'dock_lateral_tol',0.) or station.get('lateral_tolerance',.006)
        yaw_tol=self.dock_yaw_tolerance()
        dist_tol=max(station.get('distance_tolerance',.003),getattr(self,'dock_distance_tolerance',0.))
        speed=self.dock_min_linear;backing=False
        lost_lat=max(.014,2*lat_tol);lost_yaw=max(.055,yaw_tol+.03)
        # Mũi càng nằm trước LiDAR fork_tip_x - laser_x; dist nhỏ hơn thế + 3 cm nghĩa
        # là càng đã qua mặt pallet LiDAR thấy. 30/10/2026 hủy INSERT_ALIGNMENT_LOST
        # khi càng đã vào ~9 cm: lúc đó lùi ra hay hủy đều tệ hơn căn chỉnh tại chỗ.
        tip=self.g['fork_tip_x']-self.p['laser_x'];inside_margin=.03
        corrections=0;last_log=0.;last=None
        # ALIGN (insert=False) trước đây không ghi log nào: 04/11/2026 hai lần chạy
        # đứng 85 s trong vòng lặp này rồi DOCK_TIMEOUT/DOCK_SWEEP_BLOCKED mà không
        # biết đang sửa trục nào. Đếm số lần lùi-vào-lại và số xung quay để dừng sớm
        # với lỗi rõ nghĩa thay vì chờ hết dock_steer_timeout.
        align_pulses=0;align_stall=0;yaw_before=None;reapproaches=0
        # Theo dõi tiến độ của nhánh "tiến" (dock_no_motion).
        fwd_ref=None;fwd_since=0.;fwd_sign=0;fwd_stalls=0;settles=0
        # Lệnh bánh trễ ~1 s (smoother + ramp firmware) nên ở 0,12 m/s xe lố
        # ~0,1 m > dist_tol 0,03: nếu lùi ngay khi ex<-dist_tol thì tiến/lùi
        # luân phiên vô hạn và không bao giờ xét tới lệch ngang/góc. Chỉ lùi khi
        # lố quá back_margin; lố ít hơn thì coi như đạt điểm chuẩn bị (INSERT
        # tính lại quãng từ dist đo được nên không cần đúng 0,55 m).
        back_margin=max(dist_tol,.06)
        def log_face(tag,f,dist,y_f,ex,tips_in):
            self.get_logger().info('%s: dist %.3f ex %.3f lateral %.3f yaw %.3f tips_in %.3f width %.3f pts %d'%(
                tag,dist,ex,y_f,f['yaw'],tips_in,f.get('width',float('nan')),f.get('points',-1)))
        while time.monotonic()<end:
            f=self.observation(station)
            if f is None:
                if insert and last is not None and last[1]>inside_margin:
                    # Càng đã trong pallet mà mất mặt (quá gần, bị che, X4 rớt tia):
                    # đi thẳng nốt quãng còn lại theo odom từ lần đo tốt cuối.
                    o=self.data['odom'].pose.pose;ox,oy,oa,lex=last[2]
                    travelled=(o.position.x-ox)*math.cos(oa)+(o.position.y-oy)*math.sin(oa)
                    remaining=lex-travelled;last_good=time.monotonic()
                    if remaining<=dist_tol:
                        self.command();stable=stable or time.monotonic()
                        if time.monotonic()-stable>.5:
                            self.get_logger().info('INSERT done on odom: travelled %.3f m after face lost'%travelled);return
                    else:
                        stable=None;self.command(speed,0.,0.)
                        if not self.clear_sweep(speed):self.command();raise MissionError('DOCK_SWEEP_BLOCKED')
                    if time.monotonic()-last_log>.5:
                        last_log=time.monotonic();self.get_logger().info('INSERT face lost, odom remaining %.3f m'%remaining)
                    await self.pause();continue
                self.command()
                if time.monotonic()-last_good>1.5:raise MissionError('PARCEL_FACE_NOT_OBSERVABLE')
                await self.pause();continue
            last_good=time.monotonic()
            # Vị trí xe trong khung mặt pallet (x vào pallet, f = mặt trong base_link):
            # dist = khoảng cách dọc pháp tuyến, y_f = lệch khỏi đường tâm, psi = hướng xe.
            c,s=math.cos(f['yaw']),math.sin(f['yaw'])
            dist=c*f['x']+s*f['y'];y_f=s*f['x']-c*f['y'];psi=-f['yaw'];ex=dist-target
            tips_in=tip-dist
            if insert:
                o=self.data['odom'].pose.pose
                last=(dist,tips_in,(o.position.x,o.position.y,yaw(o.orientation),ex))
                if time.monotonic()-last_log>.5:last_log=time.monotonic();log_face('INSERT',f,dist,y_f,ex,tips_in)
            elif time.monotonic()-last_log>.5:
                last_log=time.monotonic();log_face('ALIGN'+(' (backing)' if backing else ''),f,dist,y_f,ex,tips_in)
            if insert and tips_in<=inside_margin and (abs(y_f)>lost_lat or abs(f['yaw'])>lost_yaw):
                # Càng chưa vào pallet: lệch gấp đôi dung sai ALIGN thì hủy như cũ.
                self.command();log_face('INSERT_ALIGNMENT_LOST',f,dist,y_f,ex,tips_in)
                raise MissionError('INSERT_ALIGNMENT_LOST')
            if insert and tips_in>inside_margin:
                # Càng đã trong pallet: lệch quá xa thì có thể đang đẩy pallet hoặc đo
                # nhầm vật khác -> hủy; còn lại dừng hẳn, xoay hoặc đi ngang một xung
                # (dừng theo LiDAR) rồi đo lại. Ưu tiên xoay: đi ngang trên sàn trơn
                # vừa dịch vừa xoay (30/10), nên chỉ đi ngang khi góc đã đạt.
                if abs(y_f)>.08 or abs(f['yaw'])>.25 or corrections>=4:
                    self.command();log_face('INSERT_ALIGNMENT_LOST (inside, corrections %d)'%corrections,f,dist,y_f,ex,tips_in)
                    raise MissionError('INSERT_ALIGNMENT_LOST')
                axis=None
                if abs(f['yaw'])>yaw_tol:axis,error='yaw',f['yaw']
                # Không xung đi ngang: dock_steer chỉ chạy khi xe không đi ngang được, gate
                # (holonomic=false) bỏ vy nên xe đứng im. 31/10/2026: lệch 5,2 cm > lost_lat
                # 5 cm, 2 xung 'y' không dịch gì -> hết 4 lượt -> INSERT_ALIGNMENT_LOST dù
                # càng đã vào 16 cm. Lệch ngang trong pallet chỉ sửa bằng bẻ lái khi còn
                # tiến; ngưỡng hủy 0,08 m ở trên vẫn giữ.
                if axis:
                    corrections+=1;self.command();await self.stationary()
                    log_face('INSERT correct %s #%d'%(axis,corrections),f,dist,y_f,ex,tips_in)
                    await self.lidar_pulse(station,axis,error)
                    self.filtered_face=None;self.fit_stamp=None;last_good=time.monotonic();stable=None;continue
            vx=wz=0.
            if backing:
                # Lùi thẳng, chỉ giữ hướng vuông góc mặt; lần tiến sau sửa lệch ngang.
                if ex>=self.dock_reapproach_distance:
                    backing=False;reapproaches+=1;align_pulses=align_stall=0;yaw_before=None
                    self.get_logger().info('Re-approach #%d: dist %.2f m, lateral %.3f m'%(reapproaches,dist,y_f))
                    # Ba lượt lùi-vào-lại mà vẫn lệch: bánh trượt/đo sai, báo lỗi ngay.
                    if reapproaches>3:
                        self.command();log_face('DOCK_ALIGN_NO_PROGRESS',f,dist,y_f,ex,tips_in)
                        raise MissionError('DOCK_ALIGN_NO_PROGRESS')
                else:vx=-speed;wz=clamp(1.2*f['yaw'],.20)
            elif ex>dist_tol:
                # Hướng mong muốn nhập đường tâm sau quãng L; L giảm khi tới gần nhưng
                # không dưới 0,20 m (bẻ lái gắt thì một bên bánh gần đứng, kẹt).
                L=max(ex+.10,.20);psi_d=clamp(-math.atan2(y_f,L),.35)
                vx=speed;wz=clamp(1.2*(psi_d-psi),.08 if insert else .20)
            elif not insert and ex<-back_margin:
                # Lố sát pallet hơn điểm chuẩn bị cắm quá back_margin: lùi về.
                vx=-speed;wz=clamp(1.2*f['yaw'],.20)
            elif not insert and abs(y_f)>lat_tol:
                self.get_logger().info('ALIGN: lateral %.3f m > %.3f, backing out %.2f m'%(y_f,lat_tol,self.dock_reapproach_distance))
                backing=True;continue
            elif not insert and abs(f['yaw'])>yaw_tol:
                # Đúng đường tâm, chỉ lệch góc: quay tại chỗ không đổi y_f (quay quanh tâm).
                # Xung 0,35 rad/s cho 0,1 rad chỉ dài ~0,3 s, có khi không bứt được ma
                # sát tĩnh -> lặp mãi không đổi góc. Hai xung liền không giảm được
                # 0,01 rad thì lùi ra vào lại (vừa tiến vừa bẻ lái mới sửa được góc).
                if yaw_before is not None and abs(f['yaw'])>yaw_before-.01:align_stall+=1
                else:align_stall=0
                yaw_before=abs(f['yaw']);align_pulses+=1
                if align_stall>=2:
                    self.get_logger().warning('ALIGN: %d yaw pulses, yaw stuck at %.3f rad; backing out'%(align_pulses,f['yaw']))
                    align_pulses=align_stall=0;yaw_before=None;backing=True;continue
                log_face('ALIGN pulse yaw #%d'%align_pulses,f,dist,y_f,ex,tips_in)
                await self.lidar_pulse(station,'yaw',f['yaw'])
                self.filtered_face=None;self.fit_stamp=None;last_good=time.monotonic();stable=None
                fwd_ref=None;continue
            else:
                self.command();stable=stable or time.monotonic();fwd_ref=None
                if time.monotonic()-stable>.5:
                    # 04/11/2026 chuyến 2: ALIGN báo đạt ở dist 0,568 lateral 0,023 yaw 0,014,
                    # nhưng lệnh bánh trễ ~1 s + ramp firmware nên 0,46 s sau xe còn trôi tới
                    # dist 0,484 và xoay tới yaw 0,097. INSERT nhận đúng pose đã trôi đó, góc
                    # tiếp tục tăng và vượt lost_yaw 0,11 -> INSERT_ALIGNMENT_LOST (0,112 rad
                    # thật sự không cắm được: 0,2 m chiều sâu x 0,112 = 22 mm lệch ở mũi càng,
                    # cộng lệch ngang 13 mm > khe hở ~32 mm mỗi bên).
                    if self.dock_settle<=0. or not insert and settles>=3:return
                    await self.wait_drift(self.dock_settle)
                    self.filtered_face=None;self.fit_stamp=None
                    g=self.observation(station)
                    if g is None:return
                    gc,gs=math.cos(g['yaw']),math.sin(g['yaw'])
                    gd=gc*g['x']+gs*g['y'];gl=gs*g['x']-gc*g['y']
                    log_face('ALIGN settled',g,gd,gl,gd-target,tip-gd)
                    if insert or (abs(gl)<=lat_tol and abs(g['yaw'])<=yaw_tol and
                                  -back_margin<=gd-target<=dist_tol):return
                    settles+=1
                    self.get_logger().warning(
                        'ALIGN: troi sau khi gui lenh 0 (lateral %.3f -> %.3f, yaw %.3f -> %.3f, '
                        'dist %.3f -> %.3f); sua tiep lan %d/3'%(y_f,gl,f['yaw'],g['yaw'],dist,gd,settles))
                    stable=None;last_good=time.monotonic();continue
                await self.pause();continue
            stable=None
            # 04/11/2026: ALIGN gửi vx 0,12 m/s liên tục 78 s mà dist không đổi 1 mm rồi mới
            # DOCK_TIMEOUT. Nhánh tiến ưu tiên cao hơn nhánh sửa góc nên 6 mm sai số dọc
            # (ex 0,036 vs dist_tol 0,030) khóa luôn việc sửa 21° lệch góc, và không chỗ nào
            # kiểm xe có chạy thật hay không. Nguyên nhân có thể ở ngoài mission (gate co tốc
            # độ xuống dưới mức bánh bứt được ma sát tĩnh, bánh trượt, chạm vật) nên ở đây chỉ
            # phát hiện và báo: lần đầu lùi ra vào lại, lần sau báo lỗi rõ nghĩa.
            # Đổi chiều lệnh thì đặt lại mốc; |Δdist| để lùi cũng tính là có tiến độ.
            if self.dock_no_motion>0. and vx:
                stalled=time.monotonic();sign=1 if vx>0 else -1
                if fwd_ref is None or fwd_sign!=sign or abs(dist-fwd_ref)>.005:
                    fwd_ref,fwd_since,fwd_sign=dist,stalled,sign
                elif stalled-fwd_since>=self.dock_no_motion:
                    fwd_stalls+=1;self.command();await self.stationary()
                    log_face('DOCK_NO_MOTION #%d (vx %+.2f m/s, %.1f s khong doi khoang cach)'%(
                        fwd_stalls,vx,stalled-fwd_since),f,dist,y_f,ex,tips_in)
                    # Lùi ra là chính cách cứu, nên stall lúc đang lùi thì không còn gì để thử.
                    if insert or backing or fwd_stalls>2:
                        self.get_logger().error(
                            'Xe khong chay du da gui lenh: xem /safety/scan_status (SLOW/STOP), '
                            '/esp32/status (ARMED/mcu) va canh bao stall banh trong /diagnostics')
                        raise MissionError('DOCK_NO_MOTION')
                    fwd_ref=None;align_pulses=align_stall=0;yaw_before=None;backing=True;continue
            # Không nâng wz lên dock_min_angular: bánh đang lăn tiến nên chênh tốc độ nhỏ
            # vẫn bẻ lái được, không cần bứt khỏi ma sát tĩnh.
            self.command(vx,0.,wz)
            if not self.clear_sweep(vx):self.command();raise MissionError('DOCK_SWEEP_BLOCKED')
            await self.pause()
        raise MissionError('DOCK_TIMEOUT')
    async def lidar_pulse(self,station,axis,error):
        # Một xung quay ('yaw') hoặc đi ngang ('y') ở tốc độ tối thiểu, DỪNG THEO
        # LIDAR: odom đếm cả phần bánh trượt (29/10/2026 odom 19° vs thật ~4°).
        # Ngắt sớm một đoạn "lead" vì xe còn trôi khi lệnh về 0 (giảm tốc smoother/
        # firmware ~0,3 s); vòng ALIGN đo lại sau khi đứng yên và sửa phần dư.
        speed=self.dock_min_angular if axis=='yaw' else (getattr(self,'dock_min_lateral',0.) or self.dock_min_linear)
        sign=math.copysign(1.,error)
        # Quay: có IMU thì dừng theo góc EKF (gyro, 30 Hz, không trượt bánh) thay vì
        # LiDAR 5 Hz trễ ~0,25 s. Đi ngang vẫn theo LiDAR vì odom ngang là bánh (trượt).
        gyro=axis=='yaw' and getattr(self,'pulse_yaw_from_odom',False)
        # Quãng trôi sau khi lệnh về 0 = v²/(2a), a = gia tốc dừng thật của cả chuỗi
        # (smoother, gate, ramp firmware), cộng quãng đi trong lúc phản hồi còn trễ.
        # 30/10/2026: ước 0,3 s cố định quá thấp, xung 0,25 rad/s lố ~9°. 0 = công thức cũ.
        accel=getattr(self,'dock_stop_accel_angular' if axis=='yaw' else 'dock_stop_accel_lateral',0.)
        coast=speed*speed/(2.*accel)+speed*(.08 if gyro else .25) if accel>0. else .3*speed
        lead=min(.5*abs(error),coast)
        end=time.monotonic()+min(6.,3.*abs(error)/speed+2.);seen=time.monotonic()
        start=yaw(self.data['odom'].pose.pose.orientation)
        while time.monotonic()<end:
            if gyro:
                # Không đo LiDAR khi đang quay: scan bị nhòe theo chuyển động và có
                # thể làm lệch neo bám pallet.
                if abs(wrap(yaw(self.data['odom'].pose.pose.orientation)-start))>=abs(error)-lead:break
            else:
                # Đo thô từng scan (bỏ bộ lọc 0,5) để không thêm trễ vào điểm dừng.
                self.filtered_face=None;self.fit_stamp=None
                f=self.observation(station)
                if f is not None:
                    seen=time.monotonic()
                    if sign*(f['yaw'] if axis=='yaw' else f['y'])<=lead:break
                elif time.monotonic()-seen>1.:break
            self.command(0.,sign*speed if axis=='y' else 0.,sign*speed if axis=='yaw' else 0.)
            if not self.clear_sweep(0.):self.command();raise MissionError('DOCK_SWEEP_BLOCKED')
            await self.pause()
        await self.wait_drift()
    async def wait_drift(self,timeout=2.):
        # Chờ hết trôi (không báo lỗi như stationary()) để scan sau không đo lúc xe còn
        # chuyển động. Lệnh bánh trễ ~1 s qua Wi-Fi + ramp firmware nên "gửi lệnh 0" không
        # có nghĩa là xe đã dừng.
        self.command();stable=None;end=time.monotonic()+timeout
        while time.monotonic()<end:
            t=self.data['odom'].twist.twist
            if abs(t.angular.z)<.01 and math.hypot(t.linear.x,t.linear.y)<.005:
                stable=stable or time.monotonic()
                if time.monotonic()-stable>.3:return True
            else:stable=None
            self.command();await self.pause()
        return False
    async def retreat(self,state,station):
        self.stage(state);o=self.data['odom'].pose.pose;start=(o.position.x,o.position.y,yaw(o.orientation));end=time.monotonic()+40
        required=max(self.p['retreat_distance'],self.g['fork_tip_x']-self.p['laser_x']-self.p['inserted_distance']+.10)
        if not self.dynamic:
            _,_,target=station_poses(self.p,station)
            transform=self.tf.lookup_transform('map','base_link',rclpy.time.Time()).transform
            a=station['approach'][2]
            required=max(required,(transform.translation.x-target[0])*math.cos(a)+(transform.translation.y-target[1])*math.sin(a))
        while time.monotonic()<end:
            o=self.data['odom'].pose.pose
            distance=-((o.position.x-start[0])*math.cos(start[2])+(o.position.y-start[1])*math.sin(start[2]))
            if distance>=required:self.command();await self.stationary();return
            if not self.clear_sweep(-.04):
                # 31/10/2026: B sát vật phía sau, lùi được 0,21-0,22 m/0,35 m thì LiDAR chặn dù
                # càng đã ra khỏi pallet. Đã thả (RETREAT_DROP) và càng rút hết thì dừng ở đây;
                # RETURN_HOME tự lập đường từ pose thật.
                clear=self.g['fork_tip_x']-self.p['laser_x']-self.p['inserted_distance']+.03
                if state=='RETREAT_DROP' and distance>=clear:
                    self.command();await self.stationary()
                    self.get_logger().warning('RETREAT_DROP blocked after %.2f m (forks clear at %.2f m); continuing'%(distance,clear))
                    return
                raise MissionError('RETREAT_BLOCKED')
            self.drive(-min(self.p.get('retreat_speed',.045),max(.008,.6*(required-distance))),0.,clamp(wrap(start[2]-yaw(o.orientation)),.04))
            await self.pause()
        raise MissionError('RETREAT_TIMEOUT')
    async def execute(self,handle):
        self.mission_goal=handle;self.events=[];self.code='';self.not_ready_since=None;self.start_blocked=None;self.started=time.monotonic();self.deadline=self.started+(handle.request.timeout or 420.)
        self.dynamic=handle.request.mission=='pick_and_place';self.refinement=None;self.initial_detection=None
        self.route_segments=[]
        try:
            self.stage('CHECK_READY');self.check();await self.select(True);await self.stationary()
            if not self.data['lift'].homed:
                self.stage('HOME_LIFT');await self.action(self.home,HomeLift.Goal(timeout=60.),65.)
                await self.wait_lift_settled('HOME_LIFT')
            # Càng đã home và đang ở 0 thì không hạ nữa. Xe thật (01/11/2026): công tắc dưới
            # tác động sát đáy cứng, nằm yên thì nhả ra (bottom=0, steps=0); lệnh H thừa ép
            # motor bước vào đáy ~7 s tới khi công tắc chập (trượt bước, kêu "kẹt kẹt").
            # Mốc 0 chỉ được đặt khi chạm công tắc nên homed + height≈0 là đáng tin.
            lift=self.data['lift']
            if lift.homed and abs(lift.height)<=.002:
                self.get_logger().info('LOWER_FOR_PICK skipped: lift already homed at %.4f m'%lift.height)
            else:await self.move_lift(lift.home_offset,'LOWER_FOR_PICK')
            if self.dynamic:
                drop=station_from_approach(pose_values(handle.request.drop_pose),self.p['inserted_distance'],self.pallet_width,self.p['laser_x'])
                if getattr(self,'pick_pose_mode','staging')=='pallet':
                    if getattr(self,'pick_via_x',False):pick=self.virtual_pick(pose_values(handle.request.pick_pose))
                    else:pick=await self.locate_pallet_pick(pose_values(handle.request.pick_pose))
                else:
                    pick=station_from_approach(pose_values(handle.request.pick_pose),self.staging_distance,self.pallet_width,self.p['laser_x'])
                    await self.prepare_dynamic_pick(pick)
                await self.plan_dynamic_mission(pick,drop)
            else:
                pick,drop=self.p['a'],self.p['b']
                await self.plan_mission()
            await self.nav_to(pick['approach'],'NAV_PICK')
            if self.dynamic:await self.refine_pick(pick)
            await self.orient(pick,'ORIENT_PICK')
            await self.dock(pick);await self.dock(pick,True)
            await self.move_lift(self.p['carry_joint_position'],'LIFT_PICK')
            if getattr(self,'load_feedback','contacts')!='none' and not self.has_load():raise MissionError('NO_PALLET_CONTACT')
            self.load_required=True;self.loaded.publish(Bool(data=True));self.stage('VERIFY_LOAD');await self.pause(.8)
            await self.retreat('RETREAT_PICK',pick)
            await self.nav_to(drop['approach'],'NAV_DROP')
            if not self.dynamic:
                await self.orient(drop,'ORIENT_DROP')
                await self.dock(drop);await self.dock(drop,True)
            self.load_required=False
            await self.move_lift(self.data['lift'].home_offset,'LOWER_RELEASE')
            end=time.monotonic()+3.
            while self.has_load() and time.monotonic()<end:await self.pause()
            if self.has_load():raise MissionError('LOAD_NOT_RELEASED')
            self.loaded.publish(Bool(data=False));await self.retreat('RETREAT_DROP',drop)
            if not self.dynamic:await self.nav_to(self.p['home'],'RETURN_HOME')
            elif self.return_to_pick:await self.return_home_to_pick()
            await self.update_footprint()
            self.stage('DONE');self.code='SUCCEEDED';handle.succeed()
        except Exception as e:
            self.code=str(e)
            # Luồng xử lý bên trong vẫn dùng NO_VALID_PATH_* (chờ/thử lại như cũ); chỉ mã báo
            # cuối cùng nói rõ là chính chỗ xe đứng bị coi là va chạm, không phải đích bị chặn.
            if self.code.startswith('NO_VALID_PATH_') and getattr(self,'start_blocked',None):
                self.code='START_IN_COLLISION_'+self.code[len('NO_VALID_PATH_'):]
            self.failed=True;self.stage('CANCELLED' if self.code=='CANCELED' else 'FAULT')
            self.command()
            if self.client_goal is not None and self.client_goal.accepted:
                self.client_goal.cancel_goal_async()
            self.lift_stop.call_async(Trigger.Request())
            # Reserve docking source at zero so a canceled Nav2 goal cannot replay.
            self.mux.call_async(SetBool.Request(data=True))
            if handle.is_cancel_requested:handle.canceled()
            else:handle.abort()
            self.get_logger().error('P6 '+self.code)
        finally:
            self.command();self.active=False;self.mission_goal=None;self.client_goal=None
        return RunMission.Result(success=self.code=='SUCCEEDED',code=self.code,final_state=self.state)
    def tick(self):
        now=time.monotonic()
        for deadline,f in self.sleepers[:]:
            if now>=deadline:
                self.sleepers.remove((deadline,f))
                if not f.done():f.set_result(True)
        if not self.active:self.command()
        msg=dict(dynamic=self.dynamic,refinement=self.refinement,initial_detection=self.initial_detection,route_ready=bool(self.route_segments),route_revision=self.route_revision,state=self.state,code=self.code,active=self.active,load_detected=self.has_load(),
                 carrying=self.load_required,contacts=self.contact_debug,contact_messages=self.contact_messages,face=self.face,elapsed=now-self.started if self.started else 0.,events=self.events)
        self.status.publish(String(data=json.dumps(msg)))
        if self.mission_goal is not None:
            self.mission_goal.publish_feedback(RunMission.Feedback(state=self.state,load_detected=self.has_load(),lift_height=self.data.get('lift',LiftState()).height))

def main():
    rclpy.init();n=TaskManager()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():n.command()
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
