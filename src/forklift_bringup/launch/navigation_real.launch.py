# SPDX-License-Identifier: Apache-2.0
"""Localization + dynamic missions with existing hardware /scan_raw, /odom, lift.
Does not connect serial ports or enable the hardware command gate.
"""
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def description(context):
    # Cùng góc lắp LiDAR thật như mapping_real.launch.py (+90°); lệch nhau thì
    # scan không khớp map đã dựng. SIM giữ mặc định 0 trong xacro.
    # wheel_type đổi bán kính bánh và độ cao khung (LiDAR/IMU/càng) trong TF.
    robot=xacro.process_file(str(Path(share('forklift_description'))/'urdf/forklift.urdf.xacro'),
                             mappings={'laser_yaw':'1.5707963267948966',
                                       'wheel_type':LaunchConfiguration('wheel_type').perform(context)}).toxml()
    return [Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'use_sim_time':False,'robot_description':robot}],
                 condition=IfCondition(LaunchConfiguration('publish_robot_description')),output='screen')]


def generate_launch_description():
    def include(package,file,arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share(package))/'launch'/file)),launch_arguments=arguments.items())
    return LaunchDescription([
        DeclareLaunchArgument('map',description='Absolute path to the saved navigation map YAML'),
        DeclareLaunchArgument('rviz',default_value='true',choices=['true','false']),
        # AMCL xe thật không tự đặt pose (set_initial_pose=False) nên chưa có frame map và
        # global costmap chờ "frame does not exist" tới khi bấm 2D Pose Estimate (02/11/2026).
        # true: so khớp LiDAR với toàn bản đồ một lần lúc khởi động rồi gửi /initialpose.
        DeclareLaunchArgument('auto_localize',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('publish_robot_description',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('controller',default_value='mppi',choices=['mppi','rpp']),
        # 0,80 m: mep truoc footprint (0,461) con hoi 0,34 m toi mat pallet cho sai
        # lech AMCL; 0,65 cu chi hoi 0,19 m, 29/10 do con 4 cm -> MPPI blocked.
        # 02/11/2026 pick_via_x: diem ao X cach mat pallet 0,65 m (nguoi dung chon) de vat
        # can dat cach pallet >= 1 m van con cho dung. NAV_PICK dung truoc vai cm thi
        # nav_pick_fallback giao cho ALIGN. Muon an toan hon: staging_distance:=0.80.
        DeclareLaunchArgument('staging_distance',default_value='0.65'),
        # Bo do pallet luc xuat phat, di vong vat toi diem ao X roi moi do (02/11/2026).
        DeclareLaunchArgument('pick_via_x',default_value='true',choices=['true','false']),
        # Phai khop real.launch.py imu:= (mac dinh true: MPU + EKF tren Pi).
        DeclareLaunchArgument('use_ekf',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('pallet_face_width',default_value='0.24'),
        # Xe thật chưa có cảm biến tải: none bỏ kiểm tra tải, người vận hành tự quan sát.
        DeclareLaunchArgument('load_feedback',default_value='none',choices=['external','none']),
        # Goal nâng ESP32 pulse mode = lift_raise_steps/lift_steps_per_m trong esp32.yaml
        # (20000/200000). Lệch giá trị này thì bridge từ chối goal LIFT_PICK.
        DeclareLaunchArgument('carry_joint_position',default_value='0.1'),
        # Xe thật: bấm A lên pallet, LiDAR tự tìm mặt và tính điểm đứng 0,65 m.
        DeclareLaunchArgument('pick_pose_mode',default_value='pallet',choices=['staging','pallet']),
        # Hàng đợi ô trên RViz: 3 cặp = ô 1..3 lấy, ô 4..6 đặt. autoload nạp lại danh sách
        # ô đã Save của phiên trước (mặc định ~/.ros/forklift_slots.yaml).
        DeclareLaunchArgument('slot_pairs',default_value='3'),
        DeclareLaunchArgument('slots_file',default_value=''),
        DeclareLaunchArgument('autoload_slots',default_value='false',choices=['true','false']),
        # Chỗ chật: pallet còn trên sàn chặn NAV_DROP lúc lập trước -> hoãn tới sau khi nhấc.
        DeclareLaunchArgument('require_drop_preflight',default_value='false',choices=['true','false']),
        # Be rong LiDAR do duoc thay doi theo goc nhin (0,18-0,29 m da gap): nhan trong khoang.
        DeclareLaunchArgument('pallet_face_min_width',default_value='0.20'),
        # 01/11/2026: thung 35x25x15 dung dung, mat 35 cm quay ve xe (SIM bays do 0,34 m):
        # 0,35 sat nguong, sai so vai mm la loai -> nang len 0,42. Van nhan mat 25 cm cu.
        DeclareLaunchArgument('pallet_face_max_width',default_value='0.42'),
        # Banh khong lan deu duoi ~0,6 rad/s: 0,04 m/s ~ 0,8 rad/s banh; 0,15 rad/s quay ~ 0,8 rad/s banh.
        # 30/10: ALIGN 47 s ca 4 banh stall lien tuc (0,04 m/s ~ 0,8 rad/s banh) -> DOCK_TIMEOUT.
        # 0,06 m/s ~ 1,2 rad/s banh. Xung dung theo LiDAR nen toc do cao hon khong lam lo dich.
        # 30/10 khuya: ALIGN tien + be lai o 0,06 van dung khi mot ben banh cham -> 0,08.
        # 04/11/2026: 0,08 -> 0,12 m/s. Khi dock_steer be lai (wz kep 0,20 rad/s), banh phia
        # trong chi nhan (0,08 - 0,2265*0,20)/0,0425 = 0,82 rad/s -> banh cao su khong thang
        # duoc ma sat tinh, firmware bao WHEEL_TRACKING_WARNING [FL]/[FR] va xe di thang thay
        # vi be lai. Hau qua: lech ngang 7,4 cm lap y het qua 8 vong Re-approach -> DOCK_TIMEOUT.
        # 0,12 m/s dua banh trong len 1,76 rad/s. Van la toc do bo, chua do lai tren san khac.
        DeclareLaunchArgument('dock_min_linear',default_value='0.12'),
        # 0,15 rad/s ~ 0,8 rad/s banh: chay deu tren san sau ff 0,19+0,028w (29/10).
        # 0,30 cu + tre lenh ~1 s lam ALIGN dao dong goc +-15 do toi DOCK_TIMEOUT.
        # 30/10: 0,15 cung stall ca 4 banh khi quay tai cho -> 0,25 rad/s (~1,3 rad/s banh).
        # Do san 30/10: quay chay tu 0,20 rad/s; 0,30 cho du luc khi co tai/pin yeu.
        # 03/11: banh cao su quay tai cho 0,30 van stall ca 4 banh (WHEEL_TRACKING_WARNING lien
        # tuc) khi max_duty con 0,70 -> 0,45. Sau khi nap max_duty 1,0 (03/11 00:45) quay du luc
        # nhung lo goc -> 0,35. Chua hieu chuan.
        DeclareLaunchArgument('dock_min_angular',default_value='0.35'),
        # 0,06 m/s con troi ~0,3 s (~1,8 cm) sau khi lenh ve 0: dung sai 8 mm se tien/lui
        # qua lai quanh dich. 15 mm du cho dung truoc pallet va do sau cam cang.
        DeclareLaunchArgument('dock_distance_tolerance',default_value='0.03'),
        # Do tren san 29/10: quay tai cho tu dung yen o 0,2 rad/s ket ca 4 banh, >=0,3 quay duoc.
        # Lech goc < 0,05 rad (~3 do) khong Spin; ALIGN tien khi lech < 0,05 rad.
        # Pallet tu lam to, khe cang rong (30/10): ngang 25 mm, goc 0,08 rad (4,6 do),
        # khoang cach 30 mm. INSERT huy khi lech gap doi (50 mm / 0,11 rad).
        DeclareLaunchArgument('dock_yaw_tol',default_value='0.08'),
        # 0,20 rad: chi lech lon moi dung Nav2 Spin (>=0,30 rad/s, lo ~0,14 rad);
        # lech nho hon de ALIGN lidar_pulse (0,15 rad/s, dung theo LiDAR) xu ly.
        DeclareLaunchArgument('orient_min_yaw',default_value='0.20'),
        # Banh cao su (khong di ngang): con lech ngang thi lui quang nay roi vao lai.
        # 0,20 cu chi sua ~2 mm/luot -> DOCK_TIMEOUT (30/10/2026). Can cho trong phia sau.
        DeclareLaunchArgument('dock_reapproach_distance',default_value='0.35'),
        DeclareLaunchArgument('dock_steer_timeout',default_value='90.0'),
        DeclareLaunchArgument('dock_no_motion',default_value='2.0'),
        DeclareLaunchArgument('dock_settle',default_value='1.5'),
        # Phải khớp real.launch.py wheel_type:= trên Pi.
        DeclareLaunchArgument('wheel_type',default_value='rubber',choices=['rubber','mecanum']),
        OpaqueFunction(function=description),
        # use_ekf phai khop real.launch.py imu:=...; true: guard doi ca /imu/data_raw va /wheel/odometry.
        include('forklift_navigation','navigation.launch.py',{'mode':'localization','wheel_type':LaunchConfiguration('wheel_type'),'use_sim_time':'false','use_ekf':LaunchConfiguration('use_ekf'),'p6':'true',
            'controller':LaunchConfiguration('controller'),'map':LaunchConfiguration('map'),
            # Wi-Fi Pi->laptop tre toi ~0,45 s; Spin can TF footprint trong dung sai nay.
            'behavior_transform_tolerance':'0.5',
            # AMCL bo scan qua Wi-Fi: TF map->odom co khoang trong toi 1,46 s (do 29/10).
            'guard_map_tf_timeout':'2.0',
            # AMCL tin scan hon odom (banh mecanum truot); khong tu dat pose (0,0,0).
            'amcl_profile':'real',
            # MPPI 600 quy dao (SIM 1000) de giam tai CPU.
            'mppi_batch_size':'600',
            # Smoother kep di ngang 0,03 m/s < toc do toi thieu docking 0,04.
            # Tran di ngang phai >= dock_min_lateral, neu khong smoother thu nho ca lenh.
            'smoother_max_lateral':'0.10',
            # Khop firmware wheel_accel 6 (30/10): khong de smoother thanh cho nghen khi dung.
            'smoother_accel':'0.3,0.3,1.2',
            # Spin: tu dung yen 0,2 rad/s khong but duoc (do tren san); toi da phai >= toi thieu.
            # 03/11: Spin -0,64 rad o 0,30-0,50 het 14 s chua xong (4 banh stall, max_duty 0,70)
            # -> 0,50-0,60. Sau max_duty 1,0: mission 00:48 quay 90 do lo ~35 do (tre odom Wi-Fi)
            # -> 0,35-0,45, va task_manager bu spin_overshoot. Chua hieu chuan.
            # 08/11/2026: bag align_test_130716, Spin -91 do o 0,45 rad/s: 4 banh stall roi giat
            # (wheel_warning lap lai), xe chi quay ~0,06 rad/s trung binh, Spin het gio (ACTION_FAILED_6)
            # sau ~21 s, tam xe troi ~9 cm. Nang len 0,55-0,60 (tran max_wz cua command gate = 0,6,
            # teleop ban phim cung 0,6). Chua do lai tren xe: kiem tra lai spin_overshoot.
            # 08/11/2026: 0,55-0,60 -> 0,80-1,00 -> 0,35-0,40. Thử 0,8-1,0 thì xe kẹt cả 4 bánh trên
            # sàn (WHEEL_TRACKING_WARNING [FL,FR,RL,RR]). Đo gyro: quay thật chỉ ~0,15 rad/s ở mọi
            # lệnh 0,23-0,84 rad/s (mô-men giới hạn). Trần 0,40 = max_wz của command gate (người
            # dùng chọn thử); min <= max. Chưa đo lại: kiểm tra spin_overshoot.
            'spin_min_rotational_vel':'0.35','spin_max_rotational_vel':'0.40',
            # Footprint robot.yaml đã gồm bánh và mũi càng; padding 1 cm của SIM chỉ ăn
            # vào khe hở sát tường vốn đã bị lệch bản đồ/AMCL 3-5 cm (02/11/2026).
            'footprint_padding':'0.0'}),
        include('forklift_bringup','mission.launch.py',{'use_sim_time':'false','scan_topic':'/scan_raw','load_feedback':LaunchConfiguration('load_feedback'),
            'carry_joint_position':LaunchConfiguration('carry_joint_position'),
            'pick_pose_mode':LaunchConfiguration('pick_pose_mode'),
            'slot_pairs':LaunchConfiguration('slot_pairs'),
            'slots_file':LaunchConfiguration('slots_file'),
            'autoload_slots':LaunchConfiguration('autoload_slots'),
            'require_drop_preflight':LaunchConfiguration('require_drop_preflight'),
            'pallet_face_min_width':LaunchConfiguration('pallet_face_min_width'),
            'pallet_face_max_width':LaunchConfiguration('pallet_face_max_width'),
            'dock_min_linear':LaunchConfiguration('dock_min_linear'),
            'dock_min_angular':LaunchConfiguration('dock_min_angular'),
            # Do 03/11 (Spin 1,0 rad, san 0,50 rad/s): xe con quay 0,28-0,37 rad/s luc Spin bao
            # xong, troi them 0,16-0,20 rad. Do lai 03/11 01:00 o 0,35-0,45 rad/s, 4 lan +-90 do:
            # lo 0,058-0,124, TB 0,097 rad. Bu 0,10: 4 lan +-90/+-45 do sai so <= 2,3 do, khong
            # can quay sua. task_manager tu hoc tiep trong phien.
            # 08/11/2026: nang Spin len 0,55-0,60 rad/s; quang troi ti le ~omega^2 nen uoc luong
            # 0,10 * (0,6/0,45)^2 ~ 0,18 (do cu 03/11 o 0,50 rad/s: 0,16-0,20). CHUA DO o toc do moi;
            # task_manager tu hoc tiep, do lai 4 lan +-90 do roi chinh.
            'spin_overshoot':'0.17',
            'dock_distance_tolerance':LaunchConfiguration('dock_distance_tolerance'),
            'dock_yaw_tol':LaunchConfiguration('dock_yaw_tol'),
            'orient_min_yaw':LaunchConfiguration('orient_min_yaw'),
            'require_gate_autonomous':'true','nav_pick_fallback':'true',
            # Odom truot khi quay/di ngang -> bam pallet theo LiDAR. Di ngang 0,04 m/s
            # thi banh dinh roi giat; 30/10 0,06 van stall ca 4 banh -> 0,09 (tran smoother 0,10). Dung sai ngang
            # 25 mm (SIM 6 mm, pallet to 30/10); INSERT huy o max(14, 2x) = 50 mm.
            # Gia toc dung that ~min(firmware 1,15 rad/s2 | 0,3 m/s2, smoother, gate) co du phong.
            'dock_stop_accel_angular':'1.0','dock_stop_accel_lateral':'0.25',
            # Co IMU (use_ekf) thi xung quay dung theo goc EKF tu gyro.
            'pulse_yaw_from_odom':LaunchConfiguration('use_ekf'),
            # 30/10: di ngang vy 0,09-0,15 chi dich 0,4-8 cm ma xoay 2-52 do (san tron,
            # motor yeu) -> can chinh bang tien + be lai, khong dung vy.
            'dock_no_strafe':'true',
            'dock_reapproach_distance':LaunchConfiguration('dock_reapproach_distance'),
            'dock_steer_timeout':LaunchConfiguration('dock_steer_timeout'),
            # 2 s: xung 0,12 m/s dang chay thi 2 s di 0,24 m, nen 5 mm trong 2 s chac chan la
            # khong chay. Nho hon thi tre lenh/ramp ~1 s de bao dong gia.
            'dock_no_motion':LaunchConfiguration('dock_no_motion'),
            # 1,5 s: do 04/11 xe con troi 8 cm va 0,08 rad trong 0,46 s sau khi gui lenh 0.
            'dock_settle':LaunchConfiguration('dock_settle'),
            # Duong cong toi diem dung thang hang pallet thay vi bo NAV_PICK roi de ALIGN
            # quay tai cho + di ngang; sat pallet thi lui ra truoc.
            'pick_via_x':LaunchConfiguration('pick_via_x'),
            'nav_pick_curved':'true','drop_short_terminal':'true','return_to_pick':'true','ready_grace':'1.0','replan_on_block':'true','pivot_paths':'true',
            # dock_lateral_tol 0,025 -> 0,04 (04/11/2026): 0,025 la nguong tu thoi banh mecanum
            # (di ngang duoc, sua truc tiep). Banh cao su phai lui 0,35 m roi tien lai de sua
            # lech ngang; do lap lai cua chu trinh do do duoc ~0,03 m nen 0,025 khong bao gio
            # chot duoc. CHUA do khe pallet vs be ngang cang: neu khe hep hon 2x0,04 thi ha lai.
            'track_face':'true','dock_min_lateral':'0.09','dock_lateral_tol':'0.04',
            'staging_distance':LaunchConfiguration('staging_distance'),'pallet_face_width':LaunchConfiguration('pallet_face_width')}),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(Path(share('forklift_bringup'))/'rviz/mission.rviz')],
             parameters=[{'use_sim_time':False}],condition=IfCondition(LaunchConfiguration('rviz')),output='screen'),
        # Chạy một lần rồi thoát. --strict: phòng đối xứng (hai tư thế gần ngang điểm) hoặc
        # khớp < 50 % thì không gửi, người vận hành đặt tay. --if-unlocalized: đã đặt tay
        # trước khi scan tới thì không ghi đè. 120 s vì scan qua Wi-Fi có lúc tới rất muộn.
        Node(package='forklift_navigation',executable='auto_localize.py',
             arguments=['--global','--strict','--if-unlocalized','--wait','120'],
             parameters=[{'use_sim_time':False}],condition=IfCondition(LaunchConfiguration('auto_localize')),output='screen'),
    ])
