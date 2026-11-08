# SPDX-License-Identifier: Apache-2.0
"""Mission nodes for an existing Nav2/lift stack; no hardware is started here.
Nav2 velocity_smoother input must be /nav/cmd_vel_selected (p6:=true).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    common={'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('scan_topic',default_value='/scan_raw'),
        DeclareLaunchArgument('load_feedback',default_value='external',choices=['external','contacts','none']),
        # -1: dùng stations.yaml; REAL pulse mode đặt bằng goal preset của bridge.
        DeclareLaunchArgument('carry_joint_position',default_value='-1.0'),
        # staging: A là pose robot đứng (SIM/P6); pallet: A bấm lên pallet, LiDAR tự tìm.
        DeclareLaunchArgument('pick_pose_mode',default_value='staging',choices=['staging','pallet']),
        # true: NAV_DROP phải lập được trước khi chạy (SIM). false: hoãn tới sau khi nhấc.
        DeclareLaunchArgument('require_drop_preflight',default_value='true',choices=['true','false']),
        # >0 ca hai: nhan mat pallet khi be rong LiDAR do duoc trong [min,max] (xe that).
        DeclareLaunchArgument('pallet_face_min_width',default_value='0.0'),
        DeclareLaunchArgument('pallet_face_max_width',default_value='0.0'),
        # Xe that: toc do toi thieu docking/lui (m/s, rad/s) va dung sai khoang cach. 0 = SIM.
        DeclareLaunchArgument('dock_min_linear',default_value='0.0'),
        DeclareLaunchArgument('dock_min_angular',default_value='0.0'),
        DeclareLaunchArgument('dock_distance_tolerance',default_value='0.0'),
        # Xe that: dung sai goc khi ALIGN (rad, <0,055) va goc toi thieu de ORIENT goi Spin. 0 = SIM.
        DeclareLaunchArgument('dock_yaw_tol',default_value='0.0'),
        DeclareLaunchArgument('orient_min_yaw',default_value='0.0'),
        # Xe that: goc troi (rad) sau khi Spin bao xong do tre odom/lenh qua Wi-Fi; Spin dat
        # som hon chung do roi tu hoc lai. 0 = SIM (khong bu, khong quay sua).
        DeclareLaunchArgument('spin_overshoot',default_value='0.0'),
        DeclareLaunchArgument('staging_distance',default_value='0.65'),
        DeclareLaunchArgument('pallet_face_width',default_value='0.24'),
        # Hang doi o pallet tren RViz: so cap o (3 = o 1..3 lay, o 4..6 dat), chieu dai
        # khung o ve ra, va file luu o. inserted_distance chi de ve o dat cho dung cho
        # pallet se nam: phai bang gia tri inserted_distance trong stations.yaml.
        DeclareLaunchArgument('slot_pairs',default_value='3'),
        DeclareLaunchArgument('slot_length',default_value='0.30'),
        DeclareLaunchArgument('inserted_distance',default_value='0.28'),
        DeclareLaunchArgument('slots_file',default_value=''),
        DeclareLaunchArgument('autoload_slots',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('initial_detection_timeout',default_value='2.0'),
        # Xe that: tu choi mission khi /command_gate/autonomous khong phai true. false = SIM.
        DeclareLaunchArgument('require_gate_autonomous',default_value='false'),
        # Xe that: NAV_PICK bi huy khi da gan diem dung -> do LiDAR/ALIGN. false = SIM.
        DeclareLaunchArgument('nav_pick_fallback',default_value='false'),
        # Xe that: bam pallet theo lan do LiDAR truoc, toc do toi thieu di ngang (m/s),
        # dung sai ngang ALIGN (m). false/0 = SIM.
        DeclareLaunchArgument('track_face',default_value='false'),
        DeclareLaunchArgument('dock_min_lateral',default_value='0.0'),
        DeclareLaunchArgument('dock_lateral_tol',default_value='0.0'),
        # Xe that: gia toc dung that cua chuoi lenh (rad/s2, m/s2) de tinh quang troi cua
        # xung chinh; xung quay dung theo goc EKF (gyro). 0/false = SIM.
        DeclareLaunchArgument('dock_stop_accel_angular',default_value='0.0'),
        DeclareLaunchArgument('dock_stop_accel_lateral',default_value='0.0'),
        DeclareLaunchArgument('pulse_yaw_from_odom',default_value='false'),
        # Xe that: can chinh khong di ngang (tien + be lai). false = SIM.
        DeclareLaunchArgument('dock_no_strafe',default_value='false'),
        # Khi dock_no_strafe: quang lui truoc khi vao lai (m) va thoi gian toi da ALIGN/INSERT (s).
        DeclareLaunchArgument('dock_reapproach_distance',default_value='0.35'),
        DeclareLaunchArgument('dock_steer_timeout',default_value='90.0'),
        # Xe that: dang gui lenh tien ma dist khong giam bao lau thi coi la khong tien duoc
        # (gate co toc do, banh truot). 0 = SIM, bo qua nhu truoc.
        DeclareLaunchArgument('dock_no_motion',default_value='0.0'),
        # Xe that: ALIGN cho odom het troi roi do lai mat pallet truoc khi giao cho INSERT.
        # 0 = SIM (xe dung ngay).
        DeclareLaunchArgument('dock_settle',default_value='0.0'),
        # Xe that: NAV_PICK luon lap duong cong (lui ra neu sat pallet). false = SIM.
        DeclareLaunchArgument('nav_pick_curved',default_value='false'),
        # Xe that: NAV_DROP thu doan thang cuoi ngan (0,7/0,5 m) truoc khi quay tai cho.
        DeclareLaunchArgument('drop_short_terminal',default_value='false'),
        # Xe that: tha xong quay ve diem dung truoc A (A/B dong). false = SIM nhu cu.
        DeclareLaunchArgument('return_to_pick',default_value='false'),
        # Xe that: chiu mat phan hoi ngan qua Wi-Fi truoc khi bao loi. 0 = SIM nhu cu.
        DeclareLaunchArgument('ready_grace',default_value='0.0'),
        # Xe that: tranh vat bang costmap + replan khi dang bam duong. false = SIM nhu cu.
        DeclareLaunchArgument('replan_on_block',default_value='false'),
        DeclareLaunchArgument('pick_via_x',default_value='false'),
        # Xe that (skid-steer): duong quay -> di thang -> quay truoc Reeds-Shepp.
        DeclareLaunchArgument('pivot_paths',default_value='false'),
        Node(package='forklift_task_manager',executable='motion_mux.py',parameters=[common],output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='Mission mux exited'))),
        Node(package='forklift_task_manager',executable='task_manager.py',parameters=[common,{
            'approved_route_file':'', 'scan_topic':LaunchConfiguration('scan_topic'),
            'load_feedback':LaunchConfiguration('load_feedback'),
            'carry_joint_position':ParameterValue(LaunchConfiguration('carry_joint_position'),value_type=float),
            'pick_pose_mode':LaunchConfiguration('pick_pose_mode'),
            'require_drop_preflight':ParameterValue(LaunchConfiguration('require_drop_preflight'),value_type=bool),
            'pallet_face_min_width':ParameterValue(LaunchConfiguration('pallet_face_min_width'),value_type=float),
            'pallet_face_max_width':ParameterValue(LaunchConfiguration('pallet_face_max_width'),value_type=float),
            'dock_min_linear':ParameterValue(LaunchConfiguration('dock_min_linear'),value_type=float),
            'dock_min_angular':ParameterValue(LaunchConfiguration('dock_min_angular'),value_type=float),
            'dock_distance_tolerance':ParameterValue(LaunchConfiguration('dock_distance_tolerance'),value_type=float),
            'dock_yaw_tol':ParameterValue(LaunchConfiguration('dock_yaw_tol'),value_type=float),
            'orient_min_yaw':ParameterValue(LaunchConfiguration('orient_min_yaw'),value_type=float),
            'spin_overshoot':ParameterValue(LaunchConfiguration('spin_overshoot'),value_type=float),
            'staging_distance':ParameterValue(LaunchConfiguration('staging_distance'),value_type=float),
            'initial_detection_timeout':ParameterValue(LaunchConfiguration('initial_detection_timeout'),value_type=float),
            'require_gate_autonomous':ParameterValue(LaunchConfiguration('require_gate_autonomous'),value_type=bool),
            'nav_pick_fallback':ParameterValue(LaunchConfiguration('nav_pick_fallback'),value_type=bool),
            'track_face':ParameterValue(LaunchConfiguration('track_face'),value_type=bool),
            'dock_min_lateral':ParameterValue(LaunchConfiguration('dock_min_lateral'),value_type=float),
            'dock_lateral_tol':ParameterValue(LaunchConfiguration('dock_lateral_tol'),value_type=float),
            'dock_stop_accel_angular':ParameterValue(LaunchConfiguration('dock_stop_accel_angular'),value_type=float),
            'dock_stop_accel_lateral':ParameterValue(LaunchConfiguration('dock_stop_accel_lateral'),value_type=float),
            'pulse_yaw_from_odom':ParameterValue(LaunchConfiguration('pulse_yaw_from_odom'),value_type=bool),
            'dock_no_strafe':ParameterValue(LaunchConfiguration('dock_no_strafe'),value_type=bool),
            'dock_reapproach_distance':ParameterValue(LaunchConfiguration('dock_reapproach_distance'),value_type=float),
            'dock_steer_timeout':ParameterValue(LaunchConfiguration('dock_steer_timeout'),value_type=float),
            'dock_no_motion':ParameterValue(LaunchConfiguration('dock_no_motion'),value_type=float),
            'dock_settle':ParameterValue(LaunchConfiguration('dock_settle'),value_type=float),
            'nav_pick_curved':ParameterValue(LaunchConfiguration('nav_pick_curved'),value_type=bool),
            'drop_short_terminal':ParameterValue(LaunchConfiguration('drop_short_terminal'),value_type=bool),
            'return_to_pick':ParameterValue(LaunchConfiguration('return_to_pick'),value_type=bool),
            'ready_grace':ParameterValue(LaunchConfiguration('ready_grace'),value_type=float),
            'replan_on_block':ParameterValue(LaunchConfiguration('replan_on_block'),value_type=bool),
            'pick_via_x':ParameterValue(LaunchConfiguration('pick_via_x'),value_type=bool),
            'pivot_paths':ParameterValue(LaunchConfiguration('pivot_paths'),value_type=bool),
            'pallet_face_width':ParameterValue(LaunchConfiguration('pallet_face_width'),value_type=float)}],output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='Mission manager exited'))),
        Node(package='forklift_task_manager',executable='mission_input.py',parameters=[common,{
            'pick_pose_mode':LaunchConfiguration('pick_pose_mode'),
            'slot_pairs':ParameterValue(LaunchConfiguration('slot_pairs'),value_type=int),
            'slot_length':ParameterValue(LaunchConfiguration('slot_length'),value_type=float),
            'slot_width':ParameterValue(LaunchConfiguration('pallet_face_width'),value_type=float),
            'staging_distance':ParameterValue(LaunchConfiguration('staging_distance'),value_type=float),
            'inserted_distance':ParameterValue(LaunchConfiguration('inserted_distance'),value_type=float),
            'slots_file':LaunchConfiguration('slots_file'),
            'autoload_slots':ParameterValue(LaunchConfiguration('autoload_slots'),value_type=bool)}],output='screen',
             on_exit=EmitEvent(event=Shutdown(reason='Mission input exited'))),
    ])
